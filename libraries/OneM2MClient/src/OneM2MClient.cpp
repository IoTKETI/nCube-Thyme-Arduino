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

void OneM2MClient::Init(String _cseid, String _brokerip, String _aeid)
{
	CSE_ID = _cseid;
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

String OneM2MClient::createAE(PubSubClient mqtt, String rqi, int index, String api)
{
	body_str =
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

	if(request(mqtt, body_str)) {
		return body_str;
	}
	else {
		return "0";
	}
}

String OneM2MClient::createCnt(PubSubClient mqtt, String rqi, int index)
{
	body_str =
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

	if(request(mqtt, body_str)) {
		return body_str;
	}
	else {
		return "0";
	}
}

String OneM2MClient::deleteSub(PubSubClient mqtt, String rqi, int index)
{
	body_str =
	"{"
	"\"op\":\"4\","
	"\"to\":\"" + sub[index].to + "/" + sub[index].rn + "?rcn=0\","
	"\"fr\":\"" + AE_ID + "\","
	"\"rqi\":\"" + rqi + "\","
	"\"pc\":{"
	"}"
	"}";

	if(request(mqtt, body_str)) {
		return body_str;
	}
	else {
		return "0";
	}
}

String OneM2MClient::createSub(PubSubClient mqtt, String rqi, int index)
{
	body_str =
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

	if(request(mqtt, body_str)) {
		return body_str;
	}
	else {
		return "0";
	}
}

String OneM2MClient::createCin(PubSubClient mqtt, String rqi, String to, String value)
{
	body_str =
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

	if(request(mqtt, body_str)) {
		return body_str;
	}
	else {
		return "0";
	}
}

bool OneM2MClient::request(PubSubClient mqtt, String body_str)
{
	body_str.toCharArray(out_message, MQTT_MAX_PACKET_SIZE);

	if (!mqtt.publish(req_topic, out_message, body_str.length())) {
		return false;
	}
	else {
		return true;
	}
}

bool OneM2MClient::response(PubSubClient mqtt, String body_str)
{
	body_str.toCharArray(out_message, MQTT_MAX_PACKET_SIZE);

	if (!mqtt.publish(noti_resp_topic, out_message, body_str.length())) {
		return false;
	}
	else {
		return true;
	}
}

String OneM2MClient::heartbeat(PubSubClient mqtt) {
	body_str = AE_ID + ":" + String(sequence++);
	body_str.toCharArray(out_message, MQTT_MAX_PACKET_SIZE);

	if (!mqtt.publish(heartbeat_topic, out_message, body_str.length())) {
		return "Failed";
	}
	else {
		return body_str;
	}
}

void OneM2MClient::reset_heartbeat() {
	sequence = 0;
}

void OneM2MClient::initTopic() {
	String topic = "/oneM2M/req/" + AE_ID + CSE_ID + "/json";
	topic.toCharArray(req_topic, 48);

	//topic = "/oneM2M/resp/" + AE_ID + "/Mobius/json";
	//topic.toCharArray(resp_topic, 48);

	//topic = "/oneM2M/req/Mobius/" + AE_ID + "/json";
	//topic.toCharArray(noti_topic, 48);

	topic = "/oneM2M/resp" + CSE_ID + "/" + AE_ID + "/json";
	topic.toCharArray(noti_resp_topic,48);

	topic = "/nCube/heartbeat";
	topic.toCharArray(heartbeat_topic, 48);
}

String OneM2MClient::getAeid() {
	return AE_ID;
}

// String OneM2MClient::getRespTopic() {
// 	return String(resp_topic);
// }

String OneM2MClient::getReqTopic() {
	return String(req_topic);
}

// String OneM2MClient::getNotiTopic() {
// 	return String(noti_topic);
// }

String OneM2MClient::getNotiRespTopic() {
	return String(noti_resp_topic);
}

String OneM2MClient::getHeartbeatTopic() {
	return String(heartbeat_topic);
}

unsigned long OneM2MClient::get_sequence() {
	return sequence;
}
