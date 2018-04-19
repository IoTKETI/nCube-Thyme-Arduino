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
OneM2MClient.cpp - Library for oneM2M Client interacting to oneM2M server named Mobius
Created by Ilyeup Ahn in KETI Korea, March 19, 2017.
Released into the public domain.
*/

#include "Arduino.h"

#include "OneM2MClient.h"

const int ledPin = 13; // LED pin for connectivity status indicator

void (*_mqtt_resp_callback)(String topic, JsonObject& root);
void (*_mqtt_noti_callback)(String topic, JsonObject& root);

char in_message[MQTT_MAX_PACKET_SIZE];
char out_message[MQTT_MAX_PACKET_SIZE];

StaticJsonBuffer<MQTT_MAX_PACKET_SIZE*2> jsonBuffer;

void mqttMessageHandler(char* topic_in, byte* payload, unsigned int length) {
	String topic = String(topic_in);

	Serial.print("Message arrived [");
	Serial.print(topic);
	Serial.print("] <---- ");
	Serial.println(length);

	for (unsigned int i = 0; i < length; i++) {
		Serial.print((char)payload[i]);
	}
	Serial.println();

	if (topic.substring(8,12) == "resp") {
		memset((char*)in_message, '\0', length+1);
		memcpy((char*)in_message, payload, length);
		JsonObject& resp_root = jsonBuffer.parseObject(in_message);

		if (!resp_root.success()) {
			Serial.println(F("parseObject() failed"));
			return;
		}

		if(_mqtt_resp_callback != NULL) {
			_mqtt_resp_callback(topic, resp_root);
		}

		jsonBuffer.clear();
	}
	else if(topic.substring(8,11) == "req") {
		memset((char*)in_message, '\0', length+1);
		memcpy((char*)in_message, payload, length);
		JsonObject& req_root = jsonBuffer.parseObject(in_message);

		if (!req_root.success()) {
			Serial.println(F("parseObject() failed"));
			return;
		}

		if(_mqtt_noti_callback != NULL) {
			_mqtt_noti_callback(topic, req_root);
		}

		jsonBuffer.clear();
	}
}

OneM2MClient::OneM2MClient(String broker, uint16_t port)
{
	MOBIUS_MQTT_BROKER_IP = broker;
	MOBIUS_MQTT_PORT = port;

	_mqtt_resp_callback = NULL;
	_mqtt_noti_callback = NULL;

	mqtt_previousMillis = 0;
    mqtt_interval = 10; // count
    mqtt_led_interval = 500; // ms
    mqtt_wait_count = 0;

	ae_count = 0;
	cnt_count = 0;
	sub_count = 0;

	MQTT_State = _MQTT_INIT;
}

uint8_t OneM2MClient::getAeCount() {
	return ae_count;
}
uint8_t OneM2MClient::getCntCount() {
	return cnt_count;
}
uint8_t OneM2MClient::getSubCount() {
	return sub_count;
}

void OneM2MClient::MQTT_ready(PubSubClient _mqtt, char* ip, uint16_t port, uint8_t mac[6]) {
	mqtt = _mqtt;

	mqtt.setServer(ip, port);

	sprintf(mqtt_id, "nCube-%.2X%.2X", mac[1], mac[0]);

	Serial.println("mqtt.setServer - _MQTT_READY");

	MQTT_State = _MQTT_CONNECT;
	mqtt_previousMillis = 0;
	mqtt_wait_count = 0;
}

void OneM2MClient::MQTT_init(String _aeid)
{
	AE_ID = _aeid;

	initTopic();

	MQTT_State = _MQTT_INIT;
}

void OneM2MClient::MQTT_chkconnect() {
    if(MQTT_State == _MQTT_CONNECT) {
        unsigned long currentMillis = millis();
        if (currentMillis - mqtt_previousMillis >= mqtt_led_interval) {
            mqtt_previousMillis = currentMillis;
            if(mqtt_wait_count++ >= (mqtt_interval)) {
                mqtt_wait_count = 0;
                Serial.println();
        		Serial.print("Current MQTT state : ");
        		Serial.println(mqtt.state());
        		Serial.print("Attempting MQTT connection...");

                if (mqtt.connect(mqtt_id)) {
        			Serial.println("MQTT connected");

        			mqtt.setCallback(mqttMessageHandler);

                    char resp_topic[_topic.resp.length()+1];
                    char noti_topic[_topic.noti.length()+1];
                    _topic.resp.toCharArray(resp_topic, _topic.resp.length()+1);
                    _topic.noti.toCharArray(noti_topic, _topic.noti.length()+1);

					mqtt.unsubscribe(resp_topic);
					mqtt.unsubscribe(noti_topic);
        			if (mqtt.subscribe(resp_topic)) {
        				Serial.println(_topic.resp + " Successfully subscribed");
                        if (mqtt.subscribe(noti_topic)) {
            				Serial.println(_topic.noti + " Successfully subscribed");
                            MQTT_State = _MQTT_CONNECTED;
            			}
        			}
        		}
        		else {
        			Serial.print("failed, rc=");
        			Serial.print(mqtt.state());
        			Serial.println(" try again in 2 seconds");
                }
            }
            else {
                if(mqtt_wait_count % 2) {
                    digitalWrite(ledPin, HIGH);
                }
                else {
                    digitalWrite(ledPin, LOW);
                }
            }
        }
    }
    else if(MQTT_State == _MQTT_CONNECTED) {
        if (mqtt.connected()) { // Stop if already connected.
    		mqtt.loop();
    		return;
    	}

        MQTT_State = _MQTT_CONNECT;
    }
}

void OneM2MClient::configResource(uint8_t ty, String to, String rn) {
	if(ty == 2) {
		ae[ae_count].ty = ty;
	    ae[ae_count].to = to;
	    ae[ae_count].rn = rn;
	    ae[ae_count].status = 0;
		ae_count++;
	}
	else if(ty == 3) {
		cnt[cnt_count].ty = ty;
	    cnt[cnt_count].to = to;
	    cnt[cnt_count].rn = rn;
	    cnt[cnt_count].status = 0;
		cnt_count++;
	}
	else if(ty == 23) {
		sub[sub_count].ty = ty;
	    sub[sub_count].to = to;
	    sub[sub_count].rn = rn;
	    sub[sub_count].status = 0;
		sub_count++;
	}
}

String OneM2MClient::validSur(String sur) {
	int i = 0;
	for(i = 0; i < sub_count; i++) {
		if(sur == sub[i].to + "/" + sub[i].rn) {
			return sub[i].to;
		}
	}
	return "empty";
}

void OneM2MClient::createAE(String rqi, int index, String api)
{
	String body_str =
	"{"
	"\"op\":\"1\","
	"\"to\":\"" + ae[index].to + "?rcn=0\","
	"\"fr\":\"" + AE_ID + "\","
	"\"rqi\":\"" + rqi + "\","
	"\"ty\":\"" + ae[index].ty + "\","
	"\"pc\":{"
	"\"m2m:ae\":{"
	"\"rn\":\"" + ae[index].rn + "\","
	"\"api\":\"" + api + "\","
	"\"rr\":\"true\""
	"}"
	"}"
	"}";

	request(body_str);
}

void OneM2MClient::createCnt(String rqi, int index)
{
	String body_str =
	"{"
	"\"op\":\"1\","
	"\"to\":\"" + cnt[index].to + "?rcn=0\","
	"\"fr\":\"" + AE_ID + "\","
	"\"rqi\":\"" + rqi + "\","
	"\"ty\":\"" + cnt[index].ty + "\","
	"\"pc\":{"
	"\"m2m:cnt\":{"
	"\"rn\":\"" + cnt[index].rn + "\""
	"}"
	"}"
	"}";

	request(body_str);
}

void OneM2MClient::deleteSub(String rqi, int index)
{
	String body_str =
	"{"
	"\"op\":\"4\","
	"\"to\":\"" + sub[index].to + "/" + sub[index].rn + "?rcn=0\","
	"\"fr\":\"" + AE_ID + "\","
	"\"rqi\":\"" + rqi + "\","
	"\"pc\":{"
	"}"
	"}";

	request(body_str);
}

void OneM2MClient::createSub(String rqi, int index)
{
	String body_str =
	"{"
	"\"op\":\"1\","
	"\"to\":\"" + sub[index].to + "?rcn=0\","
	"\"fr\":\"" + AE_ID + "\","
	"\"rqi\":\"" + rqi + "\","
	"\"ty\":\"" + sub[index].ty + "\","
	"\"pc\":{"
	"\"m2m:sub\":{"
	"\"rn\":\"" + sub[index].rn + "\","
	"\"enc\":{"
	"\"net\":[3]},"
	"\"nu\":"
	"[\"mqtt://" + String(MOBIUS_MQTT_BROKER_IP)  + ":1883/" + AE_ID + "?ct=json&rcn=9\"],"
	"\"nct\":\"2\""
	"}"
	"}"
	"}";

	request(body_str);
}

void OneM2MClient::createCin(String rqi, String to, String value)
{
	String body_str =
	"{"
	"\"op\":\"1\","
	"\"to\":\"" + to +  "?rcn=0\","
	"\"fr\":\"" + AE_ID + "\","
	"\"rqi\":\"" + rqi + "\","
	"\"ty\":\"4\","
	"\"pc\":{"
	"\"m2m:cin\":{"
	"\"con\":" + value + ""
	"}"
	"}"
	"}";

	request(body_str);
}

void OneM2MClient::request(String body_str)
{
	char req_topic[_topic.req.length()+1];
	_topic.req.toCharArray(req_topic, _topic.req.length()+1);

	unsigned int length = body_str.length();

	memset(out_message, '\0', MQTT_MAX_PACKET_SIZE);
	body_str.toCharArray(out_message, length+1);

	if (!mqtt.publish(req_topic, out_message)) {
		Serial.println(F("REQUEST Failed"));
	}
	else {
		Serial.print("Request [");
		Serial.print(_topic.req);
		Serial.print("] ----> ");
		Serial.println(length+1);
		Serial.println(out_message);
	}
}

void OneM2MClient::response(String body_str)
{
	char noti_resp_topic[_topic.noti_resp.length()+1];
	_topic.noti_resp.toCharArray(noti_resp_topic, _topic.noti_resp.length()+1);

	unsigned int length = body_str.length();

	memset(out_message, '\0', MQTT_MAX_PACKET_SIZE);
	body_str.toCharArray(out_message, length+1);

	if (!mqtt.publish(noti_resp_topic, out_message)) {
		Serial.println(F("RESPONSE Failed"));
	}
	else {
		Serial.print("Response [");
		Serial.print(_topic.noti_resp);
		Serial.print("] ----> ");
		Serial.println(length+1);
		Serial.println(out_message);
	}
}

void OneM2MClient::setCallback(void (*callback1)(String topic, JsonObject& root), void (*callback2)(String topic, JsonObject& root)) {
	_mqtt_resp_callback = callback1;
	_mqtt_noti_callback = callback2;
}

void OneM2MClient::initTopic() {
	_topic.req        = "/oneM2M/req/" + AE_ID + "/Mobius/json";
	_topic.resp       = "/oneM2M/resp/" + AE_ID + "/Mobius/json";

	_topic.noti       = "/oneM2M/req/Mobius/" + AE_ID + "/json";
	_topic.noti_resp  = "/oneM2M/resp/Mobius/" + AE_ID + "/json";
}

String OneM2MClient::getAeid() {
	return AE_ID;
}
