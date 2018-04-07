/**
* Copyright (c) 2015, OCEAN
* All rights reserved.
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* 3. The name of the author may not be used to endorse or promote products derived from this software without specific prior written permission.
* THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
* Created by Chen Nan in KETI on 2016-07-28.
*/
#include "m0_ota.h"

KETIOTAClient::KETIOTAClient():_storage(&InternalStorage), nexted(true), request_flag(6), seq(1), resp_seq(-1), temp_data(""), temp_total_size(0), cur_total_size(0) {

};

static long get_version_number(String ver){
    int ver_param[4];

    for(int i = 0; i < 4; i++){
        if(i != 3){
            int pos = ver.indexOf('.');
            String temp = ver.substring(0, pos);
            ver_param[i] = temp.toInt();
            ver.remove(0, pos + 1);
        }
        else {
            ver_param[i] = ver.toInt();
        }
    }
    return ver_param[0] * 1000 + ver_param[1] * 100 +  ver_param[2] * 10 + ver_param[3];
}

void KETIOTAClient::start(){
    request_flag = 0;
}

void KETIOTAClient::begin(String aeid, String fw_ver){
    my_aeid = aeid;
    current_firmware_version = fw_ver;

    Serial.println("AE ID is " + my_aeid);
    Serial.println("Current fw version is " + current_firmware_version);
}

void KETIOTAClient::poll(){
    if(!OTAClient.finished()){
        poll_client();
    }
}

bool KETIOTAClient::finished(){
    if(request_flag == 6){
        return true;
    }
    else {
        return false;
    }
}

void KETIOTAClient::poll_client() {
    if(request_flag == 0) {
        Serial.println("\nStarting connection to server...");
        if (client.connect(FIRMWARE_SERVER_HOST, FIRMWARE_SERVER_PORT)) {
            Serial.println("connected to server");
            // Make a HTTP request:
            client.println("GET /fw/" + my_aeid + "/version HTTP/1.1");
            client.println("Host: " + String(FIRMWARE_SERVER_HOST));
            client.println("Connection: close");
            client.println();

            request_flag = 1;
        }
        else {
            request_flag = 6;
            //previousMillis = millis();
        }
    }
    else if(request_flag == 2 && server_firware_version != NULL){
        Serial.println("\nStarting connection to server...");
        if (client.connect(FIRMWARE_SERVER_HOST, FIRMWARE_SERVER_PORT)) {
            Serial.println("connected to server");
            // Make a HTTP request:
            client.println("GET /fw/" + my_aeid + "/" + server_firware_version + "/size HTTP/1.1");
            client.println("Host: " + String(FIRMWARE_SERVER_HOST));
            client.println("Connection: close");
            client.println();

            temp_total_size = 0;
            seq = 1;

            request_flag = 3;
        }
        else {
            request_flag = 6;
            //previousMillis = millis();
        }
    }
    else if(request_flag == 4 && server_firware_version != NULL){
        Serial.println("\nStarting connection to server...");
        if (client.connect(FIRMWARE_SERVER_HOST, FIRMWARE_SERVER_PORT)) {
            Serial.println("connected to server");
            // Make a HTTP request:
            client.println("GET /fw/" + my_aeid + "/" + server_firware_version + "/data/block?seq=" + String(seq) + " HTTP/1.1");
            client.println("Host: " + String(FIRMWARE_SERVER_HOST));
            client.println("Connection: close");
            client.println();

            request_flag = 5;
        }
        else {
            request_flag = 6;
            //previousMillis = millis();
        }
    }

    if(request_flag == 1){
        if (client) {
            String request = client.readStringUntil('\n');
            request.trim();

            if(!request.endsWith("200 OK")){
                request_flag = 6;
                //previousMillis = millis();
                return;
            }

            String header;
            long contentLength = -1;

            do {
                header = client.readStringUntil('\n');
                header.trim();

                if (header.startsWith("Content-Length: ")) {
                    header.remove(0, 16);

                    Serial.println("Content-Length:" + header);

                    contentLength = header.toInt();
                }
            } while (header != "");

            long read = 0;

            while (client.connected() && read < contentLength) {
                if (client.available()) {
                    read++;
                    char value = (char)client.read();
                    temp_data += value;
                }
            }

            if (read == contentLength) {
                server_firware_version = temp_data;
                long cur_ver = get_version_number(current_firmware_version);
                long ser_ver = get_version_number(server_firware_version);

                Serial.println("Current Firmware version is:" + current_firmware_version);
                Serial.println("Cloud Server Firmware version is:" + server_firware_version);

                if(cur_ver < ser_ver) {
                    request_flag = 2;

                    Serial.println("Need update...!");

                    if (_storage == NULL || !_storage->open()) {
                        request_flag = 6;
                        //previousMillis = millis();
                        Serial.println("Update failed...!");
                        return;
                    }
                }
                else {
                    request_flag = 6;
                    //previousMillis = millis();
                    Serial.println("Do not need update...!");
                }

                temp_data = "";
            }
        }
        else {
            client.stop();
        }
    }
    else if(request_flag == 3) {
        if (client) {
            String request = client.readStringUntil('\n');
            request.trim();

            if(!request.endsWith("200 OK")){
                request_flag = 6;
                //previousMillis = millis();
                return;
            }

            String header;
            long contentLength = -1;

            do {
                header = client.readStringUntil('\n');
                header.trim();

                if (header.startsWith("Content-Length: ")) {
                    header.remove(0, 16);

                    Serial.println("Content-Length:" + header);

                    contentLength = header.toInt();
                }
            } while (header != "");

            long read = 0;

            while (client.connected() && read < contentLength) {
                if (client.available()) {
                    read++;

                    char value = (char)client.read();
                    temp_data += value;
                }
            }

            if (read == contentLength) {
                Serial.println("Cloud Server Firmware size is:" + temp_data);
                cur_total_size = temp_data.toInt();
                request_flag = 4;
                temp_data = "";
            }
        }
        else {
            client.stop();
        }
    }
    else if(request_flag == 5) {
        if (client) {
            String request = client.readStringUntil('\n');
            request.trim();

            if(!request.endsWith("200 OK")){
                request_flag = 6;
                //previousMillis = millis();
                return;
            }

            String header;
            long contentLength = -1;

            do {
                header = client.readStringUntil('\n');
                header.trim();

                if (header.startsWith("Content-Length: ")) {
                    header.remove(0, 16);

                    Serial.println("Content-Length:" + header);

                    contentLength = header.toInt();
                }
                else if(header.startsWith("Is-Next: ")) {
                    header.remove(0, 8);

                    Serial.println("Is-Next:" + header);

                    if(header.startsWith(" yes")) {
                        nexted = true;
                    } else if(header.startsWith(" no")) {
                        nexted = false;
                    }
                }
                else if(header.startsWith("Seq: ")) {
                    header.remove(0, 4);

                    Serial.println("Seq:" + header);
                    Serial.println(" ");
                    resp_seq = header.toInt();
                }
            } while (header != "");

            long read = 0;

            while (client.connected() && read < contentLength) {
                if (client.available()) {
                    read++;

                    char value = (char)client.read();
                    temp_data += value;
                    Serial.print(" ");
                    Serial.print(value, HEX);
                    _storage->write(value);
                }
            }

            if (read == contentLength) {
                temp_total_size += contentLength;
                Serial.println(" ");
                Serial.println("Current received data size: " + String(temp_total_size));

                if(resp_seq == seq){
                    seq ++;
                }

                delay(250);

                if(nexted) {
                    request_flag = 4;
                    temp_data = "";
                    Serial.println(" ");
                }
                else {
                    if(temp_total_size == cur_total_size){
                        request_flag = 6;
                        //previousMillis = millis();
                        temp_data = "";
                        Serial.println(" ");
                        Serial.println("Download firmware finish!");

                        Serial.println("Close flash write!");
                        _storage->close();
                        Serial.println("Apply new firmware!");
                        _storage->apply();
                    }
                    else {
                        init();
                        return;
                    }
                }
            }
            else {
                _storage->clear();

                client.stop();
            }
        }
    }
}

KETIOTAClient OTAClient;
