#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "roki_protocol.h"
constexpr uint8_t CH=1; struct Rx{Packet p;uint8_t src[6];int8_t rssi;uint32_t at;uint8_t len;}; QueueHandle_t q;uint8_t mobile[6]{};bool peer=false,have=false;uint32_t last=0,rx=0,lost=0,dups=0,ooo=0,crcErr=0,maxGap=0,winRx=0,winBytes=0,winAt=0,lastAt=0;
void recv(const esp_now_recv_info_t*i,const uint8_t*d,int n){if(n<20||n>150)return;Rx r{};r.len=n;r.rssi=i->rx_ctrl->rssi;r.at=millis();memcpy(r.src,i->src_addr,6);memcpy(&r.p,d,n);if(r.p.h.magic==ROKI_MAGIC&&r.p.h.kind==TELEMETRY)xQueueSend(q,&r,0);}
void addPeer(const uint8_t*mac){if(peer)return;memcpy(mobile,mac,6);esp_now_peer_info_t p{};memcpy(p.peer_addr,mac,6);p.channel=CH;p.ifidx=WIFI_IF_STA;peer=esp_now_add_peer(&p)==ESP_OK||esp_now_is_peer_exist(mac);}
void command(RokiMode m,uint16_t arg=0){if(!peer)return;Packet p{};p.h={ROKI_MAGIC,ROKI_VERSION,COMMAND,(uint8_t)m,0,0,millis(),0,arg};esp_now_send(mobile,(uint8_t*)&p,sizeof(Header));}
void parseSerial(){static String s;while(Serial.available()){char c=Serial.read();if(c=='\n'){s.trim();if(s=="MODE NORMAL")command(NORMAL);else if(s=="MODE RESERVE")command(RESERVE);else if(s=="MODE STRESS")command(STRESS);else if(s.startsWith("AUTO "))command(AUTO_SWEEP,(uint16_t)s.substring(5).toInt());s="";}else s+=c;}}
void setup(){Serial.begin(115200);WiFi.mode(WIFI_STA);WiFi.setSleep(false);esp_wifi_set_channel(CH,WIFI_SECOND_CHAN_NONE);esp_wifi_set_protocol(WIFI_IF_STA,WIFI_PROTOCOL_LR);q=xQueueCreate(32,sizeof(Rx));esp_now_init();esp_now_register_recv_cb(recv);Serial.println("ROKI_RX_READY,espnow=LR");}
void loop(){parseSerial();Rx r;while(xQueueReceive(q,&r,0)==pdTRUE){if(!peer)addPeer(r.src);bool ok=r.p.h.payloadLen==r.len-sizeof(Header)&&crc16(r.p.payload,r.p.h.payloadLen)==r.p.h.crc;if(!ok)crcErr++;else{uint32_t gap=lastAt?r.at-lastAt:0;if(gap>maxGap)maxGap=gap;lastAt=r.at;if(have){if(r.p.h.seq>last+1)lost+=r.p.h.seq-last-1;else if(r.p.h.seq==last)dups++;else ooo++;}if(!have||r.p.h.seq>last)last=r.p.h.seq;have=true;rx++;winRx++;winBytes+=r.len;float loss=rx+lost?100.f*lost/(rx+lost):0;Serial.printf("RX,seq=%lu,rssi=%d,len=%u,crc=%s,mode=%u,gap=%lu,loss=%.3f\n",(unsigned long)r.p.h.seq,r.rssi,r.len,ok?"OK":"BAD",r.p.h.mode,(unsigned long)gap,loss);}}uint32_t n=millis();if(n-winAt>=1000){winAt=n;Serial.printf("STATUS,rx=%lu,lost=%lu,dups=%lu,ooo=%lu,crc_err=%lu,pps=%lu,kbps=%.2f,max_gap_ms=%lu\n",(unsigned long)rx,(unsigned long)lost,(unsigned long)dups,(unsigned long)ooo,(unsigned long)crcErr,(unsigned long)winRx,winBytes*8/1000.f,(unsigned long)maxGap);winRx=winBytes=0;maxGap=0;}}
