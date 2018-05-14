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

typedef struct _resource_t {
  String ty;
  String to;
  String rn;
  int status;
} resource_t;

#define AE_COUNT 1
#define CNT_COUNT 10
#define SUB_COUNT 5

class OneM2MClient : public WiFiClass
{
  public:
    OneM2MClient();
    String createAE(PubSubClient mqtt, String rqi, int index, String api);
    String createCnt(PubSubClient mqtt, String rqi, int index);
    String deleteSub(PubSubClient mqtt, String rqi, int index);
    String createSub(PubSubClient mqtt, String rqi, int index);
    String createCin(PubSubClient mqtt, String rqi, String to, String value);

    String heartbeat(PubSubClient mqtt);
    void reset_heartbeat();
    unsigned long get_sequence();

    bool response(PubSubClient mqtt, String body_str);

    uint8_t getAeCount();
    uint8_t getCntCount();
    uint8_t getSubCount();

	String getAeid();
    //String getRespTopic();
    String getReqTopic();
    //String getNotiTopic();
    String getNotiRespTopic();
    String getHeartbeatTopic();

    void Init(String _cseid, String _brokerip, String _aeid);

    void configResource(uint8_t ty, String to, String rn);
    String validSur(String sur);

  private:
    void initTopic();
    bool request(PubSubClient mqtt, String body_str);

	String AE_ID;
    String CSE_ID;
    String BROKER_IP;

    String body_str;
    char req_topic[48];
    //char resp_topic[48];
    //char noti_topic[48];
    char noti_resp_topic[48];
    char heartbeat_topic[48];
    char out_message[MQTT_MAX_PACKET_SIZE];

    resource_t ae[AE_COUNT];
    int ae_count;
    resource_t cnt[CNT_COUNT];
    int cnt_count;
    resource_t sub[SUB_COUNT];
	int sub_count;

    unsigned long sequence;
};

#endif // ONEM2MCLIENT_H
