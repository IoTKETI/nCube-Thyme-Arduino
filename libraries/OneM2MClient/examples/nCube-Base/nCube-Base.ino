#include <Arduino.h>
#include <SPI.h>

/**
   Copyright (c) 2017, OCEAN
   All rights reserved.
   Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
   1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
   2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   3. The name of the author may not be used to endorse or promote products derived from this software without specific prior written permission.
   THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ArduinoJson.h>

#include "OneM2MClient.h"
#include "m0_ota.h"

#include "TasCO2.h"

#define LEDPIN 13

#define LED_RED_PIN 10
#define LED_GREEN_PIN 9
#define LED_BLUE_PIN 6
#define LED_GND_PIN 5

TasCO2 TasCO2Sensor;

const String FIRMWARE_VERSION = "1.0.0.1";

const String AE_ID = "Sedu5";
const String AE_NAME = "edu5";
const String MQTT_BROKER_IP = "203.253.128.161";
const uint16_t MQTT_BROKER_PORT = 1883;
OneM2MClient nCube(MQTT_BROKER_IP, MQTT_BROKER_PORT, AE_ID); // AE-ID

unsigned long req_previousMillis = 0;
const long req_interval = 2000;

unsigned long co2_sensing_previousMillis = 0;
const long co2_sensing_interval = (1000 * 5);

short action_flag = 0;
short sensing_flag = 0;
short control_flag = 0;

String noti_con = "";

char body_buff[400];  //for inputting data to publish
char req_id[10];       //for generating random number for request packet id

String resp_rqi = "";

String state = "init";

uint8_t g_idx = 0;

String curValue = "";

/*************************** Sketch Code ************************************/

void rand_str(char *dest, size_t length) {
    char charset[] = "0123456789"
            "abcdefghijklmnopqrstuvwxyz"
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ";

    while (length-- > 0) {
        size_t index = (double) rand() / RAND_MAX * (sizeof charset - 1);
        *dest++ = charset[index];
    }
    *dest = '\0';
}

void TasCO2Sensor_upload_callback(String con) {
    if (state == "create_cin") {
        curValue = "\"" + TasCO2Sensor.curValue + "\"";
        sensing_flag = 1;
    }
}

void resp_callback(String topic, JsonObject &root) {
    int response_code = root["rsc"];
    String request_id = String(req_id);
    String response_id = root["rqi"];

    if (request_id == response_id) {
        if (action_flag == 0) {
            if (response_code == 2000 || response_code == 2001 || response_code == 2002 || response_code == 4105) {
                action_flag = 1;
                if(nCube.resource[g_idx].status == 0) {
                    nCube.resource[g_idx].status = 1;
                }
                else {
                    nCube.resource[g_idx].status = 2;
                }
            }
            else if (response_code == 4004) {
                if(state == "delete_sub") {
                    action_flag = 1;
                    if(nCube.resource[g_idx].status == 0) {
                        nCube.resource[g_idx].status = 1;
                    }
                    else {
                        nCube.resource[g_idx].status = 2;
                    }
                }
            }
        }

        Serial.print(topic);
        Serial.println(F(" - RESP_TOPIC receive a message."));
    }
}

void noti_callback(String topic, JsonObject &root) {
    if (state == "create_cin") {
        if (root["pc"]["m2m:sgn"]["sur"] == (nCube.resource[5].to + "/" + nCube.resource[5].rn)) { // guide: uri of subscription resource for notification
            String con = root["pc"]["m2m:sgn"]["nev"]["rep"]["m2m:cin"]["con"];
            noti_con = con;

            const char *rqi = root["rqi"];
            resp_rqi = String(rqi);
            control_flag = 2;
        }
        else if (root["pc"]["m2m:sgn"]["sur"] == (nCube.resource[4].to + "/" + nCube.resource[4].rn)) {
            String con = root["pc"]["m2m:sgn"]["nev"]["rep"]["m2m:cin"]["con"];
            noti_con = con;

            const char *rqi = root["rqi"];
            resp_rqi = String(rqi);
            control_flag = 1;
        }
    }
}

void buildResource() {
    // temperally build resource structure into Mobius as oneM2M IoT Platform

    // AE resource
    uint8_t index = 0;
    nCube.resource[index].ty = "2";
    nCube.resource[index].to = "/Mobius";
    nCube.resource[index].rn = AE_NAME;
    nCube.resource[index++].status = 0;

    // Container resource
    nCube.resource[index].ty = "3";
    nCube.resource[index].to = "/Mobius/" + nCube.resource[0].rn;
    nCube.resource[index].rn = "update";                 // guide: do no modify, fix container name for OTA - nCube.resource[1].rn
    nCube.resource[index++].status = 0;

    nCube.resource[index].ty = "3";
    nCube.resource[index].to = "/Mobius/" + nCube.resource[0].rn;
    nCube.resource[index].rn = "co2";
    nCube.resource[index++].status = 0;

    nCube.resource[index].ty = "3";
    nCube.resource[index].to = "/Mobius/" + nCube.resource[0].rn;
    nCube.resource[index].rn = "led";
    nCube.resource[index++].status = 0;

    // Subscription resource
    nCube.resource[index].ty = "23";
    nCube.resource[index].to = "/Mobius/" + nCube.resource[0].rn + '/' + nCube.resource[1].rn;
    nCube.resource[index].rn = "sub";                   // guide: do not modify, fix subscripton name for OTA - nCube.resource[4].rn
    nCube.resource[index++].status = 0;

    nCube.resource[index].ty = "23";
    nCube.resource[index].to = "/Mobius/" + nCube.resource[0].rn + '/' + nCube.resource[3].rn;
    nCube.resource[index].rn = "sub";
    nCube.resource[index++].status = 0;

    nCube.resource_count = index;
}

uint32_t sequence = 0;
void publisher() {
    int i = 0;

    if (state == "create_ae") {
        Serial.print(state);
        Serial.print(" - ");
        if (action_flag == 1) {
            for (i = 0; i < nCube.resource_count; i++) {
                if (nCube.resource[i].ty == "2" && nCube.resource[i].status == 0) {
                    action_flag = 0;
                    sequence = 0;

                    g_idx = i;

                    rand_str(req_id, 8);
                    Serial.print(String(sequence));
                    Serial.print(" - ");
                    Serial.println(String(req_id));
                    nCube.createAE(req_id, 0, "3.14");
                    break;
                }
            }

            if(action_flag == 1) {
                state = "create_cnt";
                Serial.println("");
            }
        }
        else {
            sequence++;
            if(sequence > 2) {
                action_flag = 1;
            }
            Serial.println(String(sequence));
        }
    }

    if (state == "create_cnt") {
        Serial.print(state);
        Serial.print(" - ");
        if (action_flag == 1) {
            for (i = 0; i < nCube.resource_count; i++) {
                if (nCube.resource[i].ty == "3" && nCube.resource[i].status == 0) {
                    action_flag = 0;
                    sequence = 0;

                    g_idx = i;

                    rand_str(req_id, 8);
                    Serial.print(String(sequence));
                    Serial.print(" - ");
                    Serial.println(String(req_id));
                    nCube.createCnt(req_id, i);
                    break;
                }
            }

            if(action_flag == 1) {
                state = "delete_sub";
                Serial.println("");
            }
        }
        else {
            sequence++;
            if(sequence > 2) {
                action_flag = 1;
            }
            Serial.println(String(sequence));
        }
    }

    if (state == "delete_sub") {
        Serial.print(state);
        Serial.print(" - ");
        if (action_flag == 1) {
            for (i = 0; i < nCube.resource_count; i++) {
                if (nCube.resource[i].ty == "23" && nCube.resource[i].status == 0) {
                    action_flag = 0;
                    sequence = 0;

                    g_idx = i;

                    rand_str(req_id, 8);
                    Serial.print(String(sequence));
                    Serial.print(" - ");
                    Serial.println(String(req_id));
                    nCube.deleteSub(req_id, i);
                    break;
                }
            }

            if(action_flag == 1) {
                state = "create_sub";
                Serial.println("");
            }
        }
        else {
            sequence++;
            if(sequence > 2) {
                action_flag = 1;
            }
            Serial.println(String(sequence));
        }
    }

    if (state == "create_sub") {
        Serial.print(state);
        Serial.print(" - ");
        if (action_flag == 1) {
            for (i = 0; i < nCube.resource_count; i++) {
                if (nCube.resource[i].ty == "23" && nCube.resource[i].status == 1) {
                    action_flag = 0;
                    sequence = 0;

                    g_idx = i;

                    rand_str(req_id, 8);
                    Serial.print(String(sequence));
                    Serial.print(" - ");
                    Serial.println(String(req_id));
                    nCube.createSub(req_id, i);
                    break;
                }
            }

            if(action_flag == 1) {
                state = "create_cin";
                Serial.println("");
            }
        }
        else {
            sequence++;
            if(sequence > 2) {
                action_flag = 1;
            }
            Serial.println(String(sequence));
        }
    }

    else if (state == "create_cin") {
    }
}

void setup() {
    pinMode(LED_BLUE_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GND_PIN, OUTPUT);

    digitalWrite(LED_BLUE_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, HIGH);
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GND_PIN, LOW);

    //while (!Serial);
    Serial.begin(115200);

    delay(10);

    nCube.begin();

    nCube.setCallback(resp_callback, noti_callback);

    buildResource();

    //init OTA client
    OTAClient.begin(AE_NAME, FIRMWARE_VERSION);

    state = "create_ae";
    action_flag = 1;

    TasCO2Sensor.init();
    TasCO2Sensor.setCallback(TasCO2Sensor_upload_callback);

    digitalWrite(LED_BLUE_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GND_PIN, LOW);

    digitalWrite(LEDPIN, LOW);
}

void loop() {
    if(nCube.chkConnect()) {
        if (OTAClient.finished()) {
            unsigned long currentMillis = millis();

            if (currentMillis - req_previousMillis >= req_interval) {
                req_previousMillis = currentMillis;
                publisher();
            }
            else if (currentMillis - co2_sensing_previousMillis >= co2_sensing_interval) {
                co2_sensing_previousMillis = currentMillis;

                if (state == "create_cin") {
                    // guide: in here generate sensing data
                    // if get sensing data directly, assign curValue sensing data and set sensing_flag to 1
                    // if request sensing data to sensor, set sensing_flag to 0, in other code of receiving sensing data, assign curValue sensing data and set sensing_flag to 1

                    TasCO2Sensor.requestData();
                    sensing_flag = 0;
                }
            }
            else {
                if (state == "create_cin") {
                    if (sensing_flag == 1) {
                        rand_str(req_id, 8);
                        nCube.createCin(req_id, (nCube.resource[1].to + "/" + nCube.resource[1].rn), curValue);
                        sensing_flag = 0;
                    }

                    if (control_flag == 1) {
                        control_flag = 0;
                        // guide: in here control action code along to noti_con

                        if (noti_con == "active") {
                            OTAClient.start();   // active OTAClient upgrad process
                        }

                        String resp_body = "";
                        resp_body += "{\"rsc\":\"2000\",\"to\":\"\",\"fr\":\"" + nCube.getAeid() + "\",\"pc\":\"\",\"rqi\":\"" + resp_rqi + "\"}";
                        resp_body.toCharArray(body_buff, resp_body.length() + 1);
                        nCube.response(body_buff);
                    }
                    else if (control_flag == 2) {
                        control_flag = 0;
                        // guide: in here control action code along to noti_con

                        if (noti_con == "0") {
                            digitalWrite(LED_BLUE_PIN, LOW);
                            digitalWrite(LED_GREEN_PIN, LOW);
                            digitalWrite(LED_RED_PIN, LOW);
                        }
                        else if (noti_con == "1") {
                            digitalWrite(LED_BLUE_PIN, LOW);
                            digitalWrite(LED_GREEN_PIN, LOW);
                            digitalWrite(LED_RED_PIN, HIGH);
                        }
                        else if (noti_con == "2") {
                            digitalWrite(LED_BLUE_PIN, LOW);
                            digitalWrite(LED_GREEN_PIN, HIGH);
                            digitalWrite(LED_RED_PIN, LOW);
                        }
                        else if (noti_con == "3") {
                            digitalWrite(LED_BLUE_PIN, HIGH);
                            digitalWrite(LED_GREEN_PIN, LOW);
                            digitalWrite(LED_RED_PIN, LOW);
                        }
                        else if (noti_con == "4") {
                            digitalWrite(LED_BLUE_PIN, HIGH);
                            digitalWrite(LED_GREEN_PIN, HIGH);
                            digitalWrite(LED_RED_PIN, LOW);
                        }
                        else if (noti_con == "5") {
                            digitalWrite(LED_BLUE_PIN, HIGH);
                            digitalWrite(LED_GREEN_PIN, LOW);
                            digitalWrite(LED_RED_PIN, HIGH);
                        }
                        else if (noti_con == "6") {
                            digitalWrite(LED_BLUE_PIN, LOW);
                            digitalWrite(LED_GREEN_PIN, HIGH);
                            digitalWrite(LED_RED_PIN, HIGH);
                        }
                        else if (noti_con == "7") {
                            digitalWrite(LED_BLUE_PIN, HIGH);
                            digitalWrite(LED_GREEN_PIN, HIGH);
                            digitalWrite(LED_RED_PIN, HIGH);
                        }

                        String resp_body = "";
                        resp_body += "{\"rsc\":\"2000\",\"to\":\"\",\"fr\":\"" + nCube.getAeid() + "\",\"pc\":\"\",\"rqi\":\"" + resp_rqi + "\"}";
                        resp_body.toCharArray(body_buff, resp_body.length() + 1);
                        nCube.response(body_buff);
                    }
                    TasCO2Sensor.chkCO2Data();
                }
            }
        }
        else {
            OTAClient.poll();
        }
    }
    else {
        digitalWrite(LEDPIN, HIGH);
        delay(100);
        digitalWrite(LEDPIN, LOW);
        delay(100);
    }
}
