#pragma once
#include <stdint.h>
constexpr uint32_t ROKI_MAGIC=0x524F4B49; constexpr uint8_t ROKI_VERSION=1;
enum RokiKind:uint8_t{TELEMETRY=1,COMMAND=2,FEEDBACK=3}; enum RokiMode:uint8_t{NORMAL=0,RESERVE=1,STRESS=2,AUTO_SWEEP=3,CALIBRATE=4,REBOOT_MOBILE=5};
enum CalibrationCommand:uint16_t{CAL_START=1,CAL_CAPTURE=2,CAL_CANCEL=3};
struct __attribute__((packed)) Header{uint32_t magic;uint8_t version,kind,mode,payloadLen;uint32_t seq,txMs;uint16_t crc,flags;};
// SI values are scaled: acceleration = milli-m/s², angular rate = milli-rad/s,
// attitude = centidegrees.  The raw values remain in the sensor's native axes.
struct __attribute__((packed)) Sensors{
  int16_t ax,ay,az,gx,gy,gz;             // body FRD
  int16_t rawAx,rawAy,rawAz,rawGx,rawGy,rawGz;
  int32_t pressurePa;int16_t tempCenti;uint16_t humidityCenti;
  // Yaw is wrapped to -180..+180 degrees before centidegree encoding.
  int16_t rollCenti,pitchCenti,yawCenti;uint16_t estimatorStatus;
  // Inertial estimate: speed in centi-m/s, climb rate positive up.
  // km/h is derived on the ground; the saved two bytes carry battery voltage.
  uint16_t speedMpsCenti,batteryMilliVolts;int16_t verticalSpeedCenti;
  // inertialStatus bits 8..9: vertical-speed quality (0 INVALID, 1 DEGRADED,
  // 2 VALID). Bit 7 marks a rejected barometric pressure outlier; bit 10
  // marks rapid rotation, while the vertical IMU integral is inhibited.
  uint16_t accelerationCenti,inertialStatus;
};
struct __attribute__((packed)) Packet{Header h;uint8_t payload[130];};
static_assert(sizeof(Header)==20&&sizeof(Sensors)==50&&sizeof(Packet)==150,"Protocol size");
inline uint16_t crc16(const uint8_t*d,uint16_t n){uint16_t c=0xFFFF;while(n--){c^=*d++;for(uint8_t i=0;i<8;i++)c=c&1?(c>>1)^0xA001:c>>1;}return c;}
inline uint8_t packetSize(RokiMode m){return m==NORMAL?70:m==RESERVE?100:150;}
