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

char out_message[MQTT_MAX_PACKET_SIZE];

OneM2MClient::OneM2MClient()
{
	ae_count = 0;
	cnt_count = 0;
	sub_count = 0;
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

void OneM2MClient::Init(String _brokerip, String _aeid)
{
	BROKER_IP = _brokerip;
	AE_ID = _aeid;

	initTopic();
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

void OneM2MClient::createAE(PubSubClient mqtt, String rqi, int index, String api)
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

	request(mqtt, body_str);
}

void OneM2MClient::createCnt(PubSubClient mqtt, String rqi, int index)
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

	request(mqtt, body_str);
}

void OneM2MClient::deleteSub(PubSubClient mqtt, String rqi, int index)
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

	request(mqtt, body_str);
}

void OneM2MClient::createSub(PubSubClient mqtt, String rqi, int index)
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
	"[\"mqtt://" + BROKER_IP + ":1883/" + AE_ID + "?ct=json&rcn=9\"],"
	"\"nct\":\"2\""
	"}"
	"}"
	"}";

	request(mqtt, body_str);
}

void OneM2MClient::createCin(PubSubClient mqtt, String rqi, String to, String value)
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

	request(mqtt, body_str);
}

void OneM2MClient::request(PubSubClient mqtt, String body_str)
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

void OneM2MClient::response(PubSubClient mqtt, String body_str)
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

void OneM2MClient::heartbeat(PubSubClient mqtt) {
	String heartbeatTopic = "/nCube/heartbeat/" + AE_ID;
	char heartbeat_topic[heartbeatTopic.length()+1];
	heartbeatTopic.toCharArray(heartbeat_topic, heartbeatTopic.length()+1);

	String heartbeatMessage = String(sequence++);
	char heartbeat_message[heartbeatMessage.length()+1];
	heartbeatMessage.toCharArray(heartbeat_message, heartbeatMessage.length()+1);

	unsigned int length = heartbeatMessage.length();

	if (!mqtt.publish(heartbeat_topic, heartbeat_message)) {
		Serial.println(F("Send heartbeat Failed"));
	}
	else {
		Serial.print("Send heartbeat [");
		Serial.print(heartbeatTopic);
		Serial.print("] ----> ");
		Serial.println(length+1);
		Serial.println(heartbeatMessage);
	}
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

String OneM2MClient::getRespTopic() {
	return _topic.resp;
}

String OneM2MClient::getReqTopic() {
	return _topic.req;
}

String OneM2MClient::getNotiTopic() {
	return _topic.noti;
}

String OneM2MClient::getNotiRespTopic() {
	return _topic.noti_resp;
}
