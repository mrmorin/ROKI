#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdarg.h>
#include "roki_protocol.h"
constexpr uint8_t CH=1,SDA_PIN=6,SCL_PIN=7;const uint8_t BCAST[]={255,255,255,255,255,255}; Adafruit_SSD1306 oled(128,64,&Wire,-1);bool oledOK=false;struct Rx{Packet p;uint8_t src[6];int8_t rssi;uint32_t at;uint8_t len;};QueueHandle_t q;uint8_t mobile[6]{};bool peer=false,have=false;uint16_t sensorFlags=0;Sensors telemetry{};volatile uint32_t queueOverflow=0,protocolErr=0,usbDrops=0;uint32_t last=0,lastTxMs=0,txRestarts=0,rx=0,lost=0,dups=0,ooo=0,crcErr=0,maxGap=0,winRx=0,winBytes=0,winAt=0,lastAt=0,lastScreen=0,feedbackAt=0;float lastKbps=0,lastPps=0;int rssi=-127;RokiMode active=NORMAL;
void recv(const esp_now_recv_info_t*i,const uint8_t*d,int n){if(n<(int)sizeof(Header)||n>150)return;Rx r{};r.len=n;r.rssi=i->rx_ctrl->rssi;r.at=millis();memcpy(r.src,i->src_addr,6);memcpy(&r.p,d,n);if(r.p.h.magic!=ROKI_MAGIC||r.p.h.version!=ROKI_VERSION||r.p.h.kind!=TELEMETRY){protocolErr++;return;}if(xQueueSend(q,&r,0)!=pdTRUE)queueOverflow=queueOverflow+1;}
void addPeer(const uint8_t*m){if(peer)return;memcpy(mobile,m,6);esp_now_peer_info_t p{};memcpy(p.peer_addr,m,6);p.channel=CH;p.ifidx=WIFI_IF_STA;peer=esp_now_add_peer(&p)==ESP_OK||esp_now_is_peer_exist(m);}
void usbPrint(const char*fmt,...);
volatile bool usbRxPending=false;
void usbEventCallback(void*,esp_event_base_t base,int32_t event,void*){if(base==ARDUINO_HW_CDC_EVENTS&&event==ARDUINO_HW_CDC_RX_EVENT)usbRxPending=true;}
void command(RokiMode m,uint16_t arg=0){if(!peer){usbPrint("CMD,drop=no_peer\n");return;}Packet p{};p.h={ROKI_MAGIC,ROKI_VERSION,COMMAND,(uint8_t)m,0,0,millis(),0,arg};esp_err_t rc=esp_now_send(BCAST,(uint8_t*)&p,sizeof(Header));usbPrint("CMD,mode=%u,arg=%u,rc=%d\n",(unsigned)m,(unsigned)arg,(int)rc);}
void feedback(){if(!peer||millis()-feedbackAt<1000)return;feedbackAt=millis();Header h{ROKI_MAGIC,ROKI_VERSION,FEEDBACK,0,0,0,millis(),0,0};esp_now_send(mobile,(uint8_t*)&h,sizeof(h));}
void usbPrint(const char*fmt,...){char out[420];va_list ap;va_start(ap,fmt);int n=vsnprintf(out,sizeof(out),fmt,ap);va_end(ap);if(n>0&&n<(int)sizeof(out)&&Serial&&Serial.availableForWrite()>=n)Serial.write((const uint8_t*)out,n);else if(n>0)usbDrops++;}
void serial(){static String s;while(Serial.available()){char c=Serial.read();if(c=='\n'){s.trim();usbPrint("USB,line=%s\n",s.c_str());if(s=="MODE NORMAL")command(NORMAL);else if(s=="MODE RESERVE")command(RESERVE);else if(s=="MODE STRESS")command(STRESS);else if(s.startsWith("AUTO "))command(AUTO_SWEEP,(uint16_t)s.substring(5).toInt());else if(s=="CAL START")command(CALIBRATE,CAL_START);else if(s=="CAL CAPTURE")command(CALIBRATE,CAL_CAPTURE);else if(s=="CAL CANCEL")command(CALIBRATE,CAL_CANCEL);else if(s=="REBOOT")command(REBOOT_MOBILE);s="";}else s+=c;}}
void screen(float loss){
  if(!oledOK||millis()-lastScreen<300)return;
  lastScreen=millis();bool link=lastAt&&millis()-lastAt<1000;
  oled.clearDisplay();oled.setTextColor(SSD1306_WHITE);
  if(!link){
    // Do not show stale radio statistics as though they were live measurements.
    if((millis()/450)%2==0){oled.setTextSize(2);oled.setCursor(52,19);oled.print("NO");oled.setCursor(28,41);oled.print("SIGNAL");}
    oled.display();return;
  }
  oled.setTextSize(1);int pct=constrain((rssi+100)*100/70,0,100);oled.drawRect(0,0,128,9,SSD1306_WHITE);for(int i=0;i<8;i++)if(pct>(i*100/8))oled.fillRect(2+i*15,2,12,5,SSD1306_WHITE);oled.setCursor(0,13);oled.printf("RSSI %d dBm  %d%%",rssi,pct);oled.setCursor(0,29);oled.printf("PPS %.0f  %.1f kb/s",lastPps,lastKbps);oled.setCursor(0,45);oled.printf("LOSS %.2f%% GAP %lums",loss,(unsigned long)maxGap);oled.setCursor(0,55);if(telemetry.batteryMilliVolts)oled.printf("BATT %.3f V",telemetry.batteryMilliVolts/1000.f);else oled.print("BATT --");oled.display();
}
void setup(){Serial.begin(921600);Serial.setTxTimeoutMs(0);Serial.setRxBufferSize(512);Serial.onEvent(ARDUINO_HW_CDC_RX_EVENT,usbEventCallback);Wire.begin(SDA_PIN,SCL_PIN,400000);oledOK=oled.begin(SSD1306_SWITCHCAPVCC,0x3C)||oled.begin(SSD1306_SWITCHCAPVCC,0x3D);WiFi.mode(WIFI_STA);WiFi.setSleep(false);esp_wifi_set_channel(CH,WIFI_SECOND_CHAN_NONE);esp_wifi_set_protocol(WIFI_IF_STA,WIFI_PROTOCOL_LR);q=xQueueCreate(32,sizeof(Rx));esp_now_init();esp_now_register_recv_cb(recv);esp_now_peer_info_t b{};memcpy(b.peer_addr,BCAST,6);b.channel=CH;b.ifidx=WIFI_IF_STA;esp_now_add_peer(&b);usbPrint("ROKI_RX_READY,oled=%d\n",oledOK);}
void loop(){
  // On the C3 the console is USB Serial/JTAG.  The event makes reception of
  // host commands deterministic even while ESP-NOW traffic is continuous.
  if(usbRxPending||Serial.available()){usbRxPending=false;serial();}feedback(); Rx r;
  while(xQueueReceive(q,&r,0)==pdTRUE){
    addPeer(r.src); bool ok=r.p.h.payloadLen==r.len-sizeof(Header)&&crc16(r.p.payload,r.p.h.payloadLen)==r.p.h.crc;
    if(!ok){crcErr++;continue;}
    if(r.p.h.payloadLen>=sizeof(Sensors))memcpy(&telemetry,r.p.payload,sizeof(Sensors));
    sensorFlags=r.p.h.flags;
    uint32_t gap=lastAt?r.at-lastAt:0; if(gap>maxGap)maxGap=gap; lastAt=r.at; rssi=r.rssi; active=(RokiMode)r.p.h.mode;
    bool restarted=have&&r.p.h.txMs<lastTxMs;
    if(restarted){last=r.p.h.seq;txRestarts++;rx=0;lost=0;dups=0;ooo=0;crcErr=0;queueOverflow=0;winRx=winBytes=0;usbPrint("EVENT,tx_restart=%lu\n",(unsigned long)txRestarts);}
    else {if(have){if(r.p.h.seq==last+1){}else if(r.p.h.seq>last+1)lost+=r.p.h.seq-last-1;else if(r.p.h.seq==last)dups++;else ooo++;}if(!have||r.p.h.seq>last)last=r.p.h.seq;}
    lastTxMs=r.p.h.txMs;have=true;rx++;winRx++;winBytes+=r.len;float loss=rx+lost?100.f*lost/(rx+lost):0;
    usbPrint("RX,s=%lu,x=%lu,r=%d,n=%u,m=%u,g=%lu,o=%.3f,f=%u,a=%d/%d/%d,w=%d/%d/%d,ra=%d/%d/%d,rw=%d/%d/%d,rpy=%d/%d/%d,e=%u,p=%ld,t=%d,h=%u,i=%u/%u/%d/%u/%u\n",(unsigned long)r.p.h.seq,(unsigned long)r.p.h.txMs,r.rssi,r.len,r.p.h.mode,(unsigned long)gap,loss,sensorFlags,telemetry.ax,telemetry.ay,telemetry.az,telemetry.gx,telemetry.gy,telemetry.gz,telemetry.rawAx,telemetry.rawAy,telemetry.rawAz,telemetry.rawGx,telemetry.rawGy,telemetry.rawGz,telemetry.rollCenti,telemetry.pitchCenti,telemetry.yawCenti,telemetry.estimatorStatus,(long)telemetry.pressurePa,telemetry.tempCenti,telemetry.humidityCenti,telemetry.speedMpsCenti,telemetry.batteryMilliVolts,telemetry.verticalSpeedCenti,telemetry.accelerationCenti,telemetry.inertialStatus);
  }
  uint32_t n=millis();float loss=rx+lost?100.f*lost/(rx+lost):0;
  if(n-winAt>=1000){uint32_t windowMs=n-winAt;winAt=n;lastKbps=winBytes*8.f/windowMs;lastPps=winRx*1000.f/windowMs;usbPrint("STATUS,rx=%lu,lost=%lu,dups=%lu,ooo=%lu,crc_err=%lu,queue_overflow=%lu,usb_drops=%lu,protocol_err=%lu,tx_restarts=%lu,flags=%u,pps=%.1f,kbps=%.2f,max_gap_ms=%lu\n",(unsigned long)rx,(unsigned long)lost,(unsigned long)dups,(unsigned long)ooo,(unsigned long)crcErr,(unsigned long)queueOverflow,(unsigned long)usbDrops,(unsigned long)protocolErr,(unsigned long)txRestarts,sensorFlags,lastPps,lastKbps,(unsigned long)maxGap);winRx=0;winBytes=0;maxGap=0;}
  screen(loss);
}
