#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <Adafruit_LSM6DS3.h>
#include <Adafruit_LSM6DSL.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_SHT4x.h>
#include <Adafruit_SHT31.h>
#include "roki_protocol.h"

constexpr uint8_t CH=1,SDA_PIN=6,SCL_PIN=7,LED_PIN=10,BATTERY_ADC_PIN=A0;
constexpr float G_MS2=9.80665f,G_MILLI_MS2=9806.65f;
constexpr uint32_t IMU_PERIOD_US=4500,INS_ALIGN_US=7000000UL,ZUPT_US=650000UL;
constexpr float ZUPT_BARO_RATE_MPS=.08f,ZUPT_BARO_SPAN_M=.12f;
constexpr uint32_t CAL_MAGIC=0x52434135UL; // v5: calibration is bound to the detected IMU
const uint8_t BCAST[]={255,255,255,255,255,255};
struct ImuCalibration{uint32_t magic;float accBias[3],accScale[3],gyroBias[3];int16_t refTempCenti;uint8_t quality,imuType,imuWho;};
enum ImuKind:uint8_t{IMU_NONE,IMU_LSM6DS3,IMU_LSM6DSL,IMU_LSM6DSOX,IMU_MPU6050};enum BaroKind:uint8_t{BARO_NONE,BARO_BMP280,BARO_BME280,BARO_BMP3};enum HumidityKind:uint8_t{HUM_NONE,HUM_SHT4,HUM_SHT31,HUM_BME280};
Adafruit_LSM6DS3 imu;Adafruit_LSM6DSL imuDsl;Adafruit_LSM6DSOX imuOds;Adafruit_BMP280 bmp;Adafruit_BME280 bme;Adafruit_BMP3XX bmp3;Adafruit_SHT4x sht;Adafruit_SHT31 sht31;Preferences prefs;Sensors sensor{};
ImuCalibration cal{CAL_MAGIC,{0,0,0},{1,1,1},{0,0,0},0,0,0,0};
bool imuOK=false,bmpOK=false,shtOK=false,calValid=false,calActive=false,calCollecting=false,attitudeReady=false,rebootPending=false;
bool inertialReady=false,inertialStationary=false,baroReady=false,accelSaturated=false,velocityClamped=false,dtFault=false,attitudeDynamic=false;
RokiMode mode=NORMAL;
uint32_t seq=0,lastTx=0,lastImuUs=0,lastBmp=0,lastSht=0,lastBattery=0,sweepAt=0,sweepSeconds=30,lastFeedback=0,calAt=0,lastInertialPublish=0,rebootAt=0;
uint32_t alignStableAt=0,stationaryAt=0,lastBaroUs=0;
uint8_t calStep=0;float calAcc[6][3]{},calGyro[6][3]{},calTemp[6]{},sumAcc[3]{},sumGyro[3]{},sumTemp=0;uint16_t calSamples=0;
uint8_t imuAddr=0,imuWhoAmI=0;ImuKind imuKind=IMU_NONE;BaroKind baroKind=BARO_NONE;HumidityKind humidityKind=HUM_NONE;float rollRad=0,pitchRad=0,yawRad=0;
// Unit quaternion rotating body FRD vectors into world NED.
float q0=1,q1=0,q2=0,q3=0;
float startupGyroSum[3]{},startupGyroBias[3]{};
// Vertical-only INS state.  Horizontal velocity is intentionally not exported:
// without an external reference it is not a trustworthy measurement.
float baroReferencePressure=0,filteredBaroHeight=0,previousRawBaroHeight=0,baroVerticalSpeed=0;
float verticalHeight=0,verticalVelocity=0,verticalAccelBias=0,verticalAccelerationUp=0,previousVerticalAccelerationUp=0,worldAccelerationMagnitude=0;
float stationaryBaroHeight=0,baroCandidateHeight=0;uint8_t baroCandidateCount=0;
uint32_t lastGoodBaroUs=0;bool baroOutlier=false;uint8_t verticalQuality=0;uint16_t startupGyroSamples=0;
void scan(){for(uint8_t address=8;address<120;address++){Wire.beginTransmission(address);if(Wire.endTransmission()==0)Serial.printf("I2C,addr=0x%02X\n",address);}}
int16_t clamp16(float x){return (int16_t)constrain(x,-32768.f,32767.f);}
uint16_t clampU16(float x){return (uint16_t)constrain(x,0.f,65535.f);}
void resetInertial(){inertialReady=false;inertialStationary=false;baroReady=false;accelSaturated=false;velocityClamped=false;dtFault=false;attitudeDynamic=false;baroOutlier=false;verticalQuality=0;q0=1;q1=q2=q3=0;worldAccelerationMagnitude=baroReferencePressure=filteredBaroHeight=previousRawBaroHeight=baroVerticalSpeed=verticalHeight=verticalVelocity=verticalAccelBias=verticalAccelerationUp=previousVerticalAccelerationUp=stationaryBaroHeight=baroCandidateHeight=0;baroCandidateCount=0;alignStableAt=stationaryAt=lastBaroUs=lastGoodBaroUs=0;startupGyroSamples=0;for(uint8_t i=0;i<3;i++)startupGyroSum[i]=startupGyroBias[i]=0;sensor.speedMpsCenti=ROKI_SPEED_UNAVAILABLE;sensor.accelerationCenti=sensor.batteryMilliVolts=0;sensor.verticalSpeedCenti=0;sensor.inertialStatus=0;}
uint8_t readRegister(uint8_t address,uint8_t reg){Wire.beginTransmission(address);Wire.write(reg);if(Wire.endTransmission(false)!=0||Wire.requestFrom((int)address,1)!=1)return 0;return Wire.read();}
bool writeRegister(uint8_t address,uint8_t reg,uint8_t value){Wire.beginTransmission(address);Wire.write(reg);Wire.write(value);return Wire.endTransmission()==0;}
bool beginMpu6050(){
 const uint8_t addresses[]={0x68,0x69};
 for(uint8_t address:addresses){
  // SY-104 boards are often fitted with MPU-compatible clones.  They use the
  // MPU6050 register map but may return a non-standard WHO_AM_I value.  An
  // I2C ACK plus a successful register configuration is the useful test here;
  // do not reject an otherwise working IMU solely by its marketing ID.
  uint8_t who=readRegister(address,0x75);if(who==0x00||who==0xFF)continue;
  imuAddr=address;imuWhoAmI=who;
  // Wake, 200 Hz sample rate, DLPF enabled, ±16 g and ±2000 °/s.
  if(writeRegister(imuAddr,0x6B,0x00)&&writeRegister(imuAddr,0x1A,0x03)&&writeRegister(imuAddr,0x19,0x04)&&writeRegister(imuAddr,0x1B,0x18)&&writeRegister(imuAddr,0x1C,0x18))return true;
 }
 return false;
}
int16_t readI16(){uint8_t hi=Wire.read(),lo=Wire.read();return (int16_t)((hi<<8)|lo);}
bool readImu(float rawA[3],float rawG[3],float &temperature){
 if(imuKind==IMU_LSM6DS3||imuKind==IMU_LSM6DSL||imuKind==IMU_LSM6DSOX){sensors_event_t a,g,t;if(imuKind==IMU_LSM6DS3)imu.getEvent(&a,&g,&t);else if(imuKind==IMU_LSM6DSL)imuDsl.getEvent(&a,&g,&t);else imuOds.getEvent(&a,&g,&t);rawA[0]=a.acceleration.x*1000.f;rawA[1]=a.acceleration.y*1000.f;rawA[2]=a.acceleration.z*1000.f;rawG[0]=g.gyro.x*1000.f;rawG[1]=g.gyro.y*1000.f;rawG[2]=g.gyro.z*1000.f;temperature=t.temperature;return true;}
 if(imuKind!=IMU_MPU6050)return false;
 Wire.beginTransmission(imuAddr);Wire.write(0x3B);if(Wire.endTransmission(false)!=0||Wire.requestFrom((int)imuAddr,14)!=14)return false;
 int16_t ax=readI16(),ay=readI16(),az=readI16(),tr=readI16(),gx=readI16(),gy=readI16(),gz=readI16();
 rawA[0]=ax*(G_MILLI_MS2/2048.f);rawA[1]=ay*(G_MILLI_MS2/2048.f);rawA[2]=az*(G_MILLI_MS2/2048.f);
 rawG[0]=gx*(1000.f*DEG_TO_RAD/16.4f);rawG[1]=gy*(1000.f*DEG_TO_RAD/16.4f);rawG[2]=gz*(1000.f*DEG_TO_RAD/16.4f);temperature=tr/340.f+36.53f;return true;
}
void loadCalibration(){prefs.begin("roki",false);if(prefs.getBytesLength("imuCal")==sizeof(cal)){ImuCalibration saved{};prefs.getBytes("imuCal",&saved,sizeof(saved));if(saved.magic==CAL_MAGIC&&isfinite(saved.accScale[0])&&isfinite(saved.accScale[1])&&isfinite(saved.accScale[2])){cal=saved;calValid=cal.quality>=70;}}}
void startCalibration(){calActive=true;calCollecting=false;calStep=0;calSamples=0;}
void cancelCalibration(){calActive=false;calCollecting=false;calStep=0;calSamples=0;}
void beginCapture(uint32_t now){if(!calActive||calCollecting||calStep>=6)return;calCollecting=true;calAt=now;calSamples=0;sumTemp=0;for(uint8_t i=0;i<3;i++){sumAcc[i]=0;sumGyro[i]=0;}}
float dot3(const float*a,const float*b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
bool unit3(float*v){float n=sqrtf(dot3(v,v));if(n<.1f)return false;for(uint8_t i=0;i<3;i++)v[i]/=n;return true;}
void cross3(const float*a,const float*b,float*out){out[0]=a[1]*b[2]-a[2]*b[1];out[1]=a[2]*b[0]-a[0]*b[2];out[2]=a[0]*b[1]-a[1]*b[0];}
uint8_t finishCalibration(){
 float lo[3]={1e9f,1e9f,1e9f},hi[3]={-1e9f,-1e9f,-1e9f};
 for(uint8_t p=0;p<6;p++)for(uint8_t a=0;a<3;a++){lo[a]=min(lo[a],calAcc[p][a]);hi[a]=max(hi[a],calAcc[p][a]);}
 for(uint8_t a=0;a<3;a++)if(hi[a]-lo[a]<1.6f*G_MILLI_MS2)return 0;
 ImuCalibration next{CAL_MAGIC,{0,0,0},{1,1,1},{0,0,0},0,0,0,0};
 for(uint8_t a=0;a<3;a++){next.accBias[a]=(hi[a]+lo[a])*.5f;next.accScale[a]=2.f*G_MILLI_MS2/(hi[a]-lo[a]);}
 float v[6][3]{},rms=0,gyroMean=0,tempMean=0;
 for(uint8_t p=0;p<6;p++){
  for(uint8_t a=0;a<3;a++){v[p][a]=(calAcc[p][a]-next.accBias[a])*next.accScale[a];next.gyroBias[a]+=calGyro[p][a]/6.f;}
  float n=sqrtf(dot3(v[p],v[p]));rms+=(n-G_MILLI_MS2)*(n-G_MILLI_MS2);tempMean+=calTemp[p]/6.f;
  gyroMean+=sqrtf(dot3(calGyro[p],calGyro[p]))/6.f;if(!unit3(v[p]))return 0;
 }
 // Pose order is also an installation check, not merely a six-face fit.
 // The declared mounting is sensor +X=body right, +Y=USB-C nose, +Z=up.
 float opposite[3]={dot3(v[0],v[1]),dot3(v[2],v[3]),dot3(v[4],v[5])};
 float orth=fabsf(dot3(v[2],v[4]))+fabsf(dot3(v[2],v[0]))+fabsf(dot3(v[4],v[0]));
 float handed[3];cross3(v[4],v[2],handed);float determinant=dot3(handed,v[0]);
 const float expected[6][3]={{0,0,1},{0,0,-1},{0,1,0},{0,-1,0},{1,0,0},{-1,0,0}};
 for(uint8_t p=0;p<6;p++)if(dot3(v[p],expected[p])<.82f)return 0;
 if(opposite[0]>-.75f||opposite[1]>-.75f||opposite[2]>-.75f||orth>.45f||determinant<.75f)return 0;
 rms=sqrtf(rms/6.f);float posePenalty=(3.f+opposite[0]+opposite[1]+opposite[2])*40.f+orth*35.f+(1.f-determinant)*45.f;
 next.refTempCenti=(int16_t)tempMean;next.imuType=(uint8_t)imuKind;next.imuWho=imuWhoAmI;next.quality=(uint8_t)constrain(100.f-rms/G_MILLI_MS2*400.f-gyroMean/1000.f*20.f-posePenalty,0.f,100.f);
 if(next.quality<70)return 0;
 cal=next;calValid=true;attitudeReady=false;yawRad=0;resetInertial();prefs.putBytes("imuCal",&cal,sizeof(cal));return cal.quality;
}
void finishCapture(){if(calSamples<80){calCollecting=false;return;}for(uint8_t a=0;a<3;a++){calAcc[calStep][a]=sumAcc[a]/calSamples;calGyro[calStep][a]=sumGyro[a]/calSamples;}calTemp[calStep]=sumTemp/calSamples;calCollecting=false;calStep++;if(calStep>=6){uint8_t q=finishCalibration();calActive=false;calStep=0;if(!q){cal.quality=0;calValid=false;}}}
void setQuaternionFromEuler(float roll,float pitch,float yaw){float cr=cosf(roll*.5f),sr=sinf(roll*.5f),cp=cosf(pitch*.5f),sp=sinf(pitch*.5f),cy=cosf(yaw*.5f),sy=sinf(yaw*.5f);q0=cr*cp*cy+sr*sp*sy;q1=sr*cp*cy-cr*sp*sy;q2=cr*sp*cy+sr*cp*sy;q3=cr*cp*sy-sr*sp*cy;}
void updateEulerFromQuaternion(){rollRad=atan2f(2.f*(q0*q1+q2*q3),1.f-2.f*(q1*q1+q2*q2));pitchRad=asinf(constrain(2.f*(q0*q2-q3*q1),-1.f,1.f));yawRad=atan2f(2.f*(q0*q3+q1*q2),1.f-2.f*(q2*q2+q3*q3));if(yawRad<0)yawRad+=2*PI;}
void updateAttitude(const float bodyA[3],const float bodyG[3],float dt){
 float accRoll=atan2f(-bodyA[1],-bodyA[2]),accPitch=atan2f(bodyA[0],sqrtf(bodyA[1]*bodyA[1]+bodyA[2]*bodyA[2]));
 if(!attitudeReady){setQuaternionFromEuler(accRoll,accPitch,0);updateEulerFromQuaternion();attitudeReady=true;return;}
 float gx=bodyG[0]/1000.f,gy=bodyG[1]/1000.f,gz=bodyG[2]/1000.f,magnitude=sqrtf(dot3(bodyA,bodyA));
 // Mahony-style gravity correction.  It is disabled away from 1 g, so strong
 // linear acceleration does not masquerade as a tilt correction.
 if(fabsf(magnitude-G_MILLI_MS2)<.16f*G_MILLI_MS2){float ax=-bodyA[0]/magnitude,ay=-bodyA[1]/magnitude,az=-bodyA[2]/magnitude;float ex=2.f*(q1*q3-q0*q2),ey=2.f*(q0*q1+q2*q3),ez=q0*q0-q1*q1-q2*q2+q3*q3;float erx=ey*az-ez*ay,ery=ez*ax-ex*az,erz=ex*ay-ey*ax;gx+=2.2f*erx;gy+=2.2f*ery;gz+=2.2f*erz;}
 float h=.5f*dt,n0=q0+h*(-q1*gx-q2*gy-q3*gz),n1=q1+h*(q0*gx+q2*gz-q3*gy),n2=q2+h*(q0*gy-q1*gz+q3*gx),n3=q3+h*(q0*gz+q1*gy-q2*gx),norm=1.f/sqrtf(n0*n0+n1*n1+n2*n2+n3*n3);q0=n0*norm;q1=n1*norm;q2=n2*norm;q3=n3*norm;updateEulerFromQuaternion();
}
bool isStill(const float bodyA[3],const float bodyG[3]){return fabsf(sqrtf(dot3(bodyA,bodyA))-G_MILLI_MS2)<.12f*G_MILLI_MS2&&sqrtf(dot3(bodyG,bodyG))<80.f;}
void bodyToNed(const float body[3],float ned[3]){
 float cr=cosf(rollRad),sr=sinf(rollRad),cp=cosf(pitchRad),sp=sinf(pitchRad),cy=cosf(yawRad),sy=sinf(yawRad);
 ned[0]=cp*cy*body[0]+(sr*sp*cy-cr*sy)*body[1]+(cr*sp*cy+sr*sy)*body[2];
 ned[1]=cp*sy*body[0]+(sr*sp*sy+cr*cy)*body[1]+(cr*sp*sy-sr*cy)*body[2];
 ned[2]=-sp*body[0]+sr*cp*body[1]+cr*cp*body[2];
}
void updateInertial(const float bodyA[3],float bodyG[3],float dt,uint32_t nowUs){
 bool still=isStill(bodyA,bodyG);
 if(!inertialReady){
  if(!attitudeReady){float zeroGyro[3]{};updateAttitude(bodyA,zeroGyro,dt);}else updateAttitude(bodyA,bodyG,dt);
  if(!still){alignStableAt=0;startupGyroSamples=0;for(uint8_t i=0;i<3;i++)startupGyroSum[i]=0;return;}
  if(!alignStableAt)alignStableAt=nowUs;
  for(uint8_t i=0;i<3;i++)startupGyroSum[i]+=bodyG[i];
  startupGyroSamples++;
  if(nowUs-alignStableAt<INS_ALIGN_US)return;
  for(uint8_t i=0;i<3;i++)startupGyroBias[i]=startupGyroSum[i]/startupGyroSamples;
  verticalVelocity=verticalAccelBias=previousVerticalAccelerationUp=0;verticalHeight=filteredBaroHeight;
  float zeroGyro[3]{};attitudeReady=false;updateAttitude(bodyA,zeroGyro,dt);inertialReady=true;stationaryAt=nowUs;return;
 }
 for(uint8_t i=0;i<3;i++)bodyG[i]-=startupGyroBias[i];
 still=isStill(bodyA,bodyG);attitudeDynamic=sqrtf(dot3(bodyG,bodyG))>700.f;updateAttitude(bodyA,bodyG,dt);
 float specificBody[3]={bodyA[0]/1000.f,bodyA[1]/1000.f,bodyA[2]/1000.f},worldAccel[3]{};bodyToNed(specificBody,worldAccel);worldAccel[2]+=G_MS2;
 accelSaturated=fabsf(bodyA[0])>15.7f*G_MILLI_MS2||fabsf(bodyA[1])>15.7f*G_MILLI_MS2||fabsf(bodyA[2])>15.7f*G_MILLI_MS2;
 worldAccelerationMagnitude=sqrtf(dot3(worldAccel,worldAccel));if(worldAccelerationMagnitude<.055f*G_MS2){for(uint8_t i=0;i<3;i++)worldAccel[i]=0;worldAccelerationMagnitude=0;}
 dtFault=dt<.002f||dt>.030f;verticalAccelerationUp=-worldAccel[2]-verticalAccelBias;
 if(fabsf(verticalAccelerationUp)<.045f*G_MS2)verticalAccelerationUp=0;
 if(!dtFault&&!accelSaturated&&!attitudeDynamic){verticalHeight+=verticalVelocity*dt+.5f*(previousVerticalAccelerationUp+verticalAccelerationUp)*.5f*dt*dt;verticalVelocity+=(previousVerticalAccelerationUp+verticalAccelerationUp)*.5f*dt;previousVerticalAccelerationUp=verticalAccelerationUp;}
 else{previousVerticalAccelerationUp=0;if(attitudeDynamic)verticalVelocity=.75f*verticalVelocity+.25f*baroVerticalSpeed;}
 // Accelerometer/gyro alone cannot distinguish uniform motion from rest.
 // A ZUPT is therefore allowed only when barometric height is also stable.
 bool baroStill=baroReady&&fabsf(baroVerticalSpeed)<ZUPT_BARO_RATE_MPS&&fabsf(filteredBaroHeight-stationaryBaroHeight)<ZUPT_BARO_SPAN_M;
 if(still&&baroStill){if(!stationaryAt)stationaryAt=nowUs;if(nowUs-stationaryAt>=ZUPT_US){verticalAccelBias=.98f*verticalAccelBias+.02f*(-worldAccel[2]);verticalVelocity=0;previousVerticalAccelerationUp=0;inertialStationary=true;}}
 else{stationaryAt=0;inertialStationary=false;if(baroReady)stationaryBaroHeight=filteredBaroHeight;}
 if(fabsf(verticalVelocity)>300.f){velocityClamped=true;verticalVelocity=0;}
}
void updateBarometer(uint32_t n){
 if(!bmpOK||n-lastBmp<50)return;
 lastBmp=n;float pressure=0;if(baroKind==BARO_BMP280)pressure=bmp.readPressure();else if(baroKind==BARO_BME280)pressure=bme.readPressure();else if(baroKind==BARO_BMP3&&bmp3.performReading())pressure=bmp3.pressure;if(pressure<30000||pressure>110000)return;
 sensor.pressurePa=(int32_t)pressure;uint32_t nowUs=micros();if(!baroReferencePressure)baroReferencePressure=pressure;
 float rawHeight=44330.f*(1.f-powf(pressure/baroReferencePressure,.190294957f));
 if(!baroReady){filteredBaroHeight=previousRawBaroHeight=stationaryBaroHeight=rawHeight;lastBaroUs=lastGoodBaroUs=nowUs;baroReady=true;return;}
 float dt=(nowUs-lastBaroUs)/1000000.f;lastBaroUs=nowUs;if(dt<=0||dt>.5f)return;
 // Reject isolated pressure glitches before they can enter height or velocity.
 // 300 m/s is deliberately above the intended operating envelope.  It keeps
 // genuine fast motion while rejecting a one-read pressure discontinuity.
 bool discontinuity=fabsf(rawHeight-previousRawBaroHeight)>max(12.f,300.f*dt);bool disagreement=inertialReady&&fabsf(rawHeight-verticalHeight)>80.f;
 baroOutlier=discontinuity||disagreement;
 if(baroOutlier){
  // A single jump is rejected.  Several mutually consistent readings are a
  // legitimate barometric relock after prolonged IMU drift, not permanent bad data.
  if(!baroCandidateCount||fabsf(rawHeight-baroCandidateHeight)>2.f){baroCandidateHeight=rawHeight;baroCandidateCount=1;}else baroCandidateCount++;
  if(baroCandidateCount<4)return;
  filteredBaroHeight=previousRawBaroHeight=baroCandidateHeight;verticalHeight=baroCandidateHeight;verticalVelocity=baroVerticalSpeed=0;verticalAccelBias=0;baroCandidateCount=0;baroOutlier=false;
 }else baroCandidateCount=0;
 previousRawBaroHeight=rawHeight;float before=filteredBaroHeight;filteredBaroHeight=.82f*filteredBaroHeight+.18f*rawHeight;
 float measuredVz=(filteredBaroHeight-before)/dt;baroVerticalSpeed=.88f*baroVerticalSpeed+.12f*measuredVz;lastGoodBaroUs=nowUs;
 if(inertialReady){float innovation=filteredBaroHeight-verticalHeight;verticalHeight+=.16f*innovation;verticalVelocity+=.08f*(baroVerticalSpeed-verticalVelocity);verticalAccelBias-=.0008f*innovation;verticalAccelBias=constrain(verticalAccelBias,-1.5f,1.5f);}
}
void updateBattery(uint32_t n){
 if(n-lastBattery<1000)return;
 lastBattery=n;uint32_t sum=0;uint8_t valid=0;
 for(uint8_t i=0;i<8;i++){uint32_t mv=analogReadMilliVolts(BATTERY_ADC_PIN);if(mv){sum+=mv;valid++;}}
 if(valid)sensor.batteryMilliVolts=clampU16(2.f*sum/valid);
}
uint8_t verticalEstimateQuality(){
 if(!imuOK||!calValid||!attitudeReady||!inertialReady)return 0; // INVALID
 uint32_t age=lastGoodBaroUs?(micros()-lastGoodBaroUs):UINT32_MAX;
 if(!accelSaturated&&!dtFault&&!attitudeDynamic&&baroReady&&age<750000UL)return 2; // VALID
 if(baroReady&&age<3000000UL)return 1; // DEGRADED
 return 0;
}
uint16_t inertialFlags(){verticalQuality=verticalEstimateQuality();uint16_t f=0;if(imuOK&&!inertialReady)f|=1;if(inertialReady)f|=2;if(baroReady)f|=4;if(inertialStationary)f|=8;if(accelSaturated)f|=16;if(dtFault)f|=32;if(velocityClamped)f|=64;if(baroOutlier)f|=128;if(attitudeDynamic)f|=1024;f|=(uint16_t)(verticalQuality&3)<<8;return f;}
void publishInertial(uint32_t n){
 if(n-lastInertialPublish<50)return;
 lastInertialPublish=n;
 // Do not present a drifting 3D IMU integral as a measured air/ground speed.
 sensor.speedMpsCenti=ROKI_SPEED_UNAVAILABLE;sensor.verticalSpeedCenti=clamp16(verticalVelocity*100.f);sensor.accelerationCenti=clampU16(worldAccelerationMagnitude*100.f);sensor.inertialStatus=inertialFlags();
}
void sample(uint32_t n){
 if(imuOK){uint32_t nowUs=micros();if(!lastImuUs||nowUs-lastImuUs>=IMU_PERIOD_US){float dt=lastImuUs?(nowUs-lastImuUs)/1000000.f:IMU_PERIOD_US/1000000.f;lastImuUs=nowUs;float rawA[3]{},rawG[3]{},imuTemp=0;if(!readImu(rawA,rawG,imuTemp))return;
  sensor.rawAx=clamp16(rawA[0]/10.f);sensor.rawAy=clamp16(rawA[1]/10.f);sensor.rawAz=clamp16(rawA[2]/10.f);sensor.rawGx=clamp16(rawG[0]);sensor.rawGy=clamp16(rawG[1]);sensor.rawGz=clamp16(rawG[2]);
  if(calCollecting){for(uint8_t i=0;i<3;i++){sumAcc[i]+=rawA[i];sumGyro[i]+=rawG[i];}sumTemp+=imuTemp*100.f;calSamples++;if(n-calAt>=1000)finishCapture();}
  float correctedA[3],correctedG[3];for(uint8_t i=0;i<3;i++){correctedA[i]=(rawA[i]-cal.accBias[i])*cal.accScale[i];correctedG[i]=rawG[i]-cal.gyroBias[i];}
  // The only sensor-to-body mapping: sensor X=right, Y=forward, Z=up -> body FRD.
  float bodyA[3]={correctedA[1],correctedA[0],-correctedA[2]},bodyG[3]={correctedG[1],correctedG[0],-correctedG[2]};updateInertial(bodyA,bodyG,dt,nowUs);
  sensor.ax=clamp16(bodyA[0]/10.f);sensor.ay=clamp16(bodyA[1]/10.f);sensor.az=clamp16(bodyA[2]/10.f);sensor.gx=clamp16(bodyG[0]);sensor.gy=clamp16(bodyG[1]);sensor.gz=clamp16(bodyG[2]);sensor.rollCenti=clamp16(degrees(rollRad)*100.f);sensor.pitchCenti=clamp16(degrees(pitchRad)*100.f);float yawDeg=degrees(yawRad);if(yawDeg>180.f)yawDeg-=360.f;sensor.yawCenti=clamp16(yawDeg*100.f);
 }}
 updateBarometer(n);updateBattery(n);publishInertial(n);if(shtOK&&n-lastSht>=500){lastSht=n;if(humidityKind==HUM_SHT4){sensors_event_t h,t;sht.getEvent(&h,&t);sensor.tempCenti=clamp16(t.temperature*100);sensor.humidityCenti=(uint16_t)constrain(h.relative_humidity*100,0.f,10000.f);}else if(humidityKind==HUM_SHT31){sensor.tempCenti=clamp16(sht31.readTemperature()*100);sensor.humidityCenti=(uint16_t)constrain(sht31.readHumidity()*100,0.f,10000.f);}else if(humidityKind==HUM_BME280){sensor.tempCenti=clamp16(bme.readTemperature()*100);sensor.humidityCenti=(uint16_t)constrain(bme.readHumidity()*100,0.f,10000.f);}}
}
void recv(const esp_now_recv_info_t*,const uint8_t*d,int n){if(n<(int)sizeof(Header))return;const Header*h=(const Header*)d;if(h->magic!=ROKI_MAGIC||h->version!=ROKI_VERSION)return;if(h->kind==FEEDBACK){lastFeedback=millis();return;}if(h->kind!=COMMAND)return;if(h->mode==REBOOT_MOBILE){rebootPending=true;rebootAt=millis()+120;return;}if(h->mode==CALIBRATE){Serial.printf("CMD,cal=%u\n",(unsigned)h->flags);if(h->flags==CAL_START)startCalibration();else if(h->flags==CAL_CAPTURE)beginCapture(millis());else if(h->flags==CAL_CANCEL)cancelCalibration();return;}if(h->mode<=STRESS)mode=(RokiMode)h->mode;else if(h->mode==AUTO_SWEEP){mode=NORMAL;sweepAt=millis();sweepSeconds=h->flags?h->flags:30;}}
void statusLed(uint32_t n){digitalWrite(LED_PIN,n-lastFeedback<2500?HIGH:((n/250)%2));}
uint16_t telemetryFlags(){uint16_t f=(imuOK?1:0)|(bmpOK?2:0)|(shtOK?4:0);if(calActive)f|=1<<3;f|=(uint16_t)(calStep&7)<<4;if(calCollecting)f|=1<<7;f|=(uint16_t)cal.quality<<8;return f;}
uint16_t estimatorFlags(){uint16_t f=0;if(imuOK)f|=1;if(attitudeReady)f|=2;if(calValid)f|=4;if(calActive)f|=8;return f;}
void probeSensors(){
 imuOK=false;imuKind=IMU_NONE;imuAddr=0;imuWhoAmI=readRegister(0x6A,0x0F);imuAddr=(imuWhoAmI==0x69||imuWhoAmI==0x6A||imuWhoAmI==0x6C)?0x6A:0;
 if(!imuAddr){imuWhoAmI=readRegister(0x6B,0x0F);imuAddr=(imuWhoAmI==0x69||imuWhoAmI==0x6A||imuWhoAmI==0x6C)?0x6B:0;}
 if(imuAddr){if(imuWhoAmI==0x69){imuKind=IMU_LSM6DS3;imuOK=imu.begin_I2C(imuAddr);if(imuOK){imu.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);imu.setAccelDataRate(LSM6DS_RATE_208_HZ);imu.setGyroDataRate(LSM6DS_RATE_208_HZ);}}else if(imuWhoAmI==0x6A){imuKind=IMU_LSM6DSL;imuOK=imuDsl.begin_I2C(imuAddr);if(imuOK){imuDsl.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);imuDsl.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);imuDsl.setAccelDataRate(LSM6DS_RATE_208_HZ);imuDsl.setGyroDataRate(LSM6DS_RATE_208_HZ);}}else{imuKind=IMU_LSM6DSOX;imuOK=imuOds.begin_I2C(imuAddr);if(imuOK){imuOds.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);imuOds.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);imuOds.setAccelDataRate(LSM6DS_RATE_208_HZ);imuOds.setGyroDataRate(LSM6DS_RATE_208_HZ);}}}
 else if(beginMpu6050()){imuKind=IMU_MPU6050;imuOK=true;}
 baroKind=BARO_NONE;humidityKind=HUM_NONE;
 if(bmp.begin(0x76)||bmp.begin(0x77))baroKind=BARO_BMP280;
 else if(bme.begin(0x76)||bme.begin(0x77)){baroKind=BARO_BME280;humidityKind=HUM_BME280;}
 else if(bmp3.begin_I2C(0x76)||bmp3.begin_I2C(0x77)){baroKind=BARO_BMP3;bmp3.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);bmp3.setPressureOversampling(BMP3_OVERSAMPLING_4X);bmp3.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);bmp3.setOutputDataRate(BMP3_ODR_50_HZ);}
 bmpOK=baroKind!=BARO_NONE;
 if(humidityKind==HUM_NONE){if(sht.begin())humidityKind=HUM_SHT4;else if(sht31.begin(0x44)||sht31.begin(0x45))humidityKind=HUM_SHT31;}
 shtOK=humidityKind!=HUM_NONE;
}
void setup(){Serial.begin(115200);pinMode(LED_PIN,OUTPUT);digitalWrite(LED_PIN,LOW);analogReadResolution(12);analogSetPinAttenuation(BATTERY_ADC_PIN,ADC_11db);loadCalibration();resetInertial();Wire.begin(SDA_PIN,SCL_PIN,400000);probeSensors();if(!imuOK){Wire.end();Wire.begin(SDA_PIN,SCL_PIN,100000);probeSensors();}if(!imuOK&&!bmpOK&&!shtOK){Wire.end();Wire.begin(SCL_PIN,SDA_PIN,100000);probeSensors();}if(!imuOK||cal.imuType!=(uint8_t)imuKind||cal.imuWho!=imuWhoAmI){calValid=false;cal.quality=0;}scan();WiFi.mode(WIFI_STA);WiFi.setSleep(false);esp_wifi_set_channel(CH,WIFI_SECOND_CHAN_NONE);esp_wifi_set_protocol(WIFI_IF_STA,WIFI_PROTOCOL_LR);esp_now_init();esp_now_register_recv_cb(recv);esp_now_peer_info_t p{};memcpy(p.peer_addr,BCAST,6);p.channel=CH;p.ifidx=WIFI_IF_STA;esp_now_add_peer(&p);if(Serial)Serial.printf("SENSORS,imu=%d,type=%u,who=0x%02X,bmp=%d,sht=%d,cal_q=%u\n",imuOK,imuKind,imuWhoAmI,bmpOK,shtOK,cal.quality);}
void loop(){uint32_t n=millis();if(rebootPending&&(int32_t)(n-rebootAt)>=0){delay(10);ESP.restart();}statusLed(n);sample(n);sensor.estimatorStatus=estimatorFlags();if(sweepAt&&n-sweepAt>=sweepSeconds*1000UL){mode=(RokiMode)(((uint8_t)mode+1)%3);sweepAt=n;}if(n-lastTx<20)return;lastTx=n;Packet p{};uint8_t len=packetSize(mode);p.h={ROKI_MAGIC,ROKI_VERSION,TELEMETRY,(uint8_t)mode,(uint8_t)(len-sizeof(Header)),seq++,n,0,telemetryFlags()};memcpy(p.payload,&sensor,sizeof(sensor));p.h.crc=crc16(p.payload,p.h.payloadLen);esp_now_send(BCAST,(uint8_t*)&p,len);}
