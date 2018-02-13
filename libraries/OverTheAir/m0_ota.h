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
#include <WiFi101.h>

#include "OTAStorage.h"
#include "InternalStorage.h"

#define FIRMWARE_SERVER_HOST "203.253.128.161"
#define FIRMWARE_SERVER_PORT 8730

//#define VERSION_CHECK_TIME 1000 * 60 * 2

class KETIOTAClient{
  public:
  KETIOTAClient();

  void begin(String aeid, String fw_ver);
  void poll();
  bool finished();
  void start();

  private:
  void poll_client();

  private:
  WiFiClient client;
  OTAStorage* _storage;
  bool nexted;
  int request_flag;
  int seq;
  int resp_seq;
  String temp_data;
  int temp_total_size;
  int cur_total_size;
  //unsigned long previousMillis;
  String current_firmware_version;
  String server_firware_version;
  String my_aeid;
};

extern KETIOTAClient OTAClient;
