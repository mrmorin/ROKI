#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <Adafruit_LSM6DS3.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SHT4x.h>
#include "roki_protocol.h"
constexpr uint8_t CH=1,SDA_PIN=6,SCL_PIN=7,BUTTON_PIN=9,LED_PIN=10; const uint8_t BCAST[]={255,255,255,255,255,255};
Adafruit_LSM6DS3 imu; Adafruit_BMP280 bmp; Adafruit_SHT4x sht; bool imuOK,bmpOK,shtOK; Sensors sensor{}; RokiMode mode=NORMAL;uint32_t seq=0,lastTx=0,lastImu=0,lastBmp=0,lastSht=0,sweepAt=0,sweepSeconds=30,lastFeedback=0,buttonAt=0;
void scan(){for(uint8_t a=1;a<127;a++){Wire.beginTransmission(a);if(!Wire.endTransmission())Serial.printf("I2C,addr=0x%02X\n",a);}}
int16_t clamp16(float x){return (int16_t)constrain(x,-32768.f,32767.f);}
void sample(uint32_t n){if(imuOK&&n-lastImu>=5){lastImu=n;sensors_event_t a,g,t;imu.getEvent(&a,&g,&t);sensor.ax=clamp16(a.acceleration.x*1000);sensor.ay=clamp16(a.acceleration.y*1000);sensor.az=clamp16(a.acceleration.z*1000);sensor.gx=clamp16(g.gyro.x*1000);sensor.gy=clamp16(g.gyro.y*1000);sensor.gz=clamp16(g.gyro.z*1000);}if(bmpOK&&n-lastBmp>=50){lastBmp=n;sensor.pressurePa=(int32_t)bmp.readPressure();}if(shtOK&&n-lastSht>=500){lastSht=n;sensors_event_t h,t;sht.getEvent(&h,&t);sensor.tempCenti=clamp16(t.temperature*100);sensor.humidityCenti=(uint16_t)constrain(h.relative_humidity*100,0.f,10000.f);}}
void recv(const esp_now_recv_info_t*,const uint8_t*d,int n){if(n<20)return;const Header*h=(const Header*)d;if(h->magic!=ROKI_MAGIC)return;if(h->kind==FEEDBACK){lastFeedback=millis();return;}if(h->kind!=COMMAND)return;if(h->mode<=STRESS)mode=(RokiMode)h->mode;else if(h->mode==AUTO_SWEEP){mode=NORMAL;sweepAt=millis();sweepSeconds=h->flags?h->flags:30;}}
void powerButton(uint32_t n){if(!digitalRead(BUTTON_PIN)){if(!buttonAt)buttonAt=n;if(n-buttonAt>=1500){digitalWrite(LED_PIN,LOW);WiFi.mode(WIFI_OFF);delay(20);esp_deep_sleep_start();}}else buttonAt=0;}
void statusLed(uint32_t n){digitalWrite(LED_PIN,n-lastFeedback<2500?HIGH:((n/250)%2));}
void setup(){Serial.begin(115200);pinMode(BUTTON_PIN,INPUT_PULLUP);pinMode(LED_PIN,OUTPUT);digitalWrite(LED_PIN,LOW);Wire.begin(SDA_PIN,SCL_PIN,400000);scan();imuOK=imu.begin_I2C(0x6A)||imu.begin_I2C(0x6B);if(imuOK){imu.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);imu.setAccelDataRate(LSM6DS_RATE_208_HZ);imu.setGyroDataRate(LSM6DS_RATE_208_HZ);}bmpOK=bmp.begin(0x76)||bmp.begin(0x77);shtOK=sht.begin();WiFi.mode(WIFI_STA);WiFi.setSleep(false);esp_wifi_set_channel(CH,WIFI_SECOND_CHAN_NONE);esp_wifi_set_protocol(WIFI_IF_STA,WIFI_PROTOCOL_LR);esp_now_init();esp_now_register_recv_cb(recv);esp_now_peer_info_t p{};memcpy(p.peer_addr,BCAST,6);p.channel=CH;p.ifidx=WIFI_IF_STA;esp_now_add_peer(&p);Serial.printf("SENSORS,imu=%d,bmp=%d,sht=%d\n",imuOK,bmpOK,shtOK);}
void loop(){uint32_t n=millis();powerButton(n);statusLed(n);sample(n);if(sweepAt&&n-sweepAt>=sweepSeconds*1000UL){mode=(RokiMode)(((uint8_t)mode+1)%3);sweepAt=n;}if(n-lastTx<20)return;lastTx=n;Packet p{};uint8_t len=packetSize(mode);uint16_t flags=(imuOK?1:0)|(bmpOK?2:0)|(shtOK?4:0);p.h={ROKI_MAGIC,ROKI_VERSION,TELEMETRY,(uint8_t)mode,(uint8_t)(len-sizeof(Header)),seq++,n,0,flags};memcpy(p.payload,&sensor,sizeof(sensor));p.h.crc=crc16(p.payload,p.h.payloadLen);esp_now_send(BCAST,(uint8_t*)&p,len);}
