#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "roki_protocol.h"
constexpr uint8_t CH=1; const uint8_t BCAST[]={0xff,0xff,0xff,0xff,0xff,0xff};
RokiMode mode=NORMAL; uint32_t seq=0,lastTx=0,sent=0,queued=0,sendFail=0,sweepAt=0,sweepSeconds=30;
uint32_t rng=0x72A4B19D; uint32_t nextRand(){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
void txcb(const esp_now_send_info_t*,esp_now_send_status_t st){if(st!=ESP_NOW_SEND_SUCCESS)sendFail++;}
void recv(const esp_now_recv_info_t*,const uint8_t*d,int n){if(n<20)return;const Header*h=(const Header*)d;if(h->magic!=ROKI_MAGIC||h->kind!=COMMAND)return;if(h->mode<=STRESS)mode=(RokiMode)h->mode;else if(h->mode==AUTO_SWEEP){mode=NORMAL;sweepAt=millis();sweepSeconds=h->flags?h->flags:30;}}
void setup(){Serial.begin(115200);WiFi.mode(WIFI_STA);WiFi.setSleep(false);esp_wifi_set_channel(CH,WIFI_SECOND_CHAN_NONE);esp_wifi_set_protocol(WIFI_IF_STA,WIFI_PROTOCOL_LR);esp_now_init();esp_now_register_send_cb(txcb);esp_now_register_recv_cb(recv);esp_now_peer_info_t p{};memcpy(p.peer_addr,BCAST,6);p.channel=CH;p.ifidx=WIFI_IF_STA;esp_now_add_peer(&p);Serial.println("ROKI_TX_READY,mode=NORMAL,espnow=LR,broadcast=on");}
void loop(){uint32_t now=millis();if(sweepAt&&now-sweepAt>=sweepSeconds*1000UL){mode=(RokiMode)(((uint8_t)mode+1)%3);sweepAt=now;}if(now-lastTx<20)return;lastTx=now;Packet p{};uint8_t len=packetSize(mode);p.h={ROKI_MAGIC,ROKI_VERSION,TELEMETRY,(uint8_t)mode,(uint8_t)(len-sizeof(Header)),seq++,now,0,0};Sensors s{(int16_t)nextRand(),(int16_t)nextRand(),(int16_t)nextRand(),(int16_t)nextRand(),(int16_t)nextRand(),(int16_t)nextRand(),101325+(int32_t)(nextRand()%800)-400,(int16_t)(2200+nextRand()%300),(uint16_t)(4000+nextRand()%1500)};memcpy(p.payload,&s,sizeof(s));for(uint8_t i=sizeof(s);i<p.h.payloadLen;i++)p.payload[i]=(uint8_t)nextRand();p.h.crc=crc16(p.payload,p.h.payloadLen);if(esp_now_send(BCAST,(uint8_t*)&p,len)==ESP_OK){sent++;queued++;}else sendFail++;}
