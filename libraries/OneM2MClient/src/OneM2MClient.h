/**
 * Copyright (c) 2017, OCEAN
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products derived from this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
    OneM2MClient.h - Library for oneM2M Client interacting to oneM2M server named Mobius
    Created by Ilyeup Ahn in KETI Korea, March 19, 2017.
    Released into the public domain.
*/

#ifndef ONEM2MCLIENT_H
#define ONEM2MCLIENT_H

#include "Arduino.h"

#include <WiFi101.h>
#include <WiFiMDNSResponder.h>

#include <PubSubClient.h>
#include <ArduinoJson.h>

const String CSE_ID = "Mobius";

typedef struct _resource_t {
  String ty;
  String to;
  String rn;
  int status;
} resource_t;

typedef struct _topic_t {
  String req;
  String resp;
  String noti;
  String noti_resp;
} topic_t;

#define _MQTT_INIT 1
#define _MQTT_CONNECT 2
#define _MQTT_CONNECTED 3
#define _MQTT_RECONNECT 4
#define _MQTT_READY 5
#define _MQTT_IDLE 6

#define AE_COUNT 1
#define CNT_COUNT 10
#define SUB_COUNT 5

void mqttMessageHandler(char* topic_in, byte* payload, unsigned int length);

class OneM2MClient : public WiFiClass
{
  public:
    OneM2MClient(String broker, uint16_t port);
    void createAE(String rqi, int index, String api);
    void createCnt(String rqi, int index);
    void deleteSub(String rqi, int index);
    void createSub(String rqi, int index);
    void createCin(String rqi, String to, String value);

    void response(String body_str);

    void setCallback(void (*callback1)(String topic, JsonObject& root), void (*callback2)(String topic, JsonObject& root));

    uint8_t getAeCount();
    uint8_t getCntCount();
    uint8_t getSubCount();

	String getAeid();

    void MQTT_init(String _aeid);
    void MQTT_ready(PubSubClient _mqtt, char* ip, uint16_t port, uint8_t mac[6]);
    void MQTT_chkconnect();

    void configResource(uint8_t ty, String to, String rn);
    String validSur(String sur);

    uint8_t MQTT_State;

  private:
    char _ssid[M2M_MAX_SSID_LEN];
    char _password[M2M_MAX_SSID_LEN];

    void initTopic();
    void request(String body_str);

	String AE_ID;
	String MOBIUS_MQTT_BROKER_IP;
	uint16_t MOBIUS_MQTT_PORT;

    char mqtt_id[11];

    unsigned long mqtt_previousMillis;
    unsigned long mqtt_interval; // count
    unsigned long mqtt_led_interval; // ms
    uint16_t mqtt_wait_count;

    resource_t ae[AE_COUNT];
    int ae_count;
    resource_t cnt[CNT_COUNT];
    int cnt_count;
    resource_t sub[SUB_COUNT];
	int sub_count;

    topic_t _topic;

    PubSubClient mqtt;
};

#endif // ONEM2MCLIENT_H
