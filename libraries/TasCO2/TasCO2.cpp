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
    TasCO2.cpp - Library for CO2 sensor using Serial1
    Created by Ilyeup Ahn in KETI Korea, March 19, 2017.
    Released into the public domain.
*/

#include "TasCO2.h"

const char write_buff[4] = { 0x11, 0x01, 0x01, 0xED };


TasCO2::TasCO2()
{
  _measure_callback = NULL;
  _sData = "";
}

void TasCO2::init() {
  //while (!Serial1);
  Serial1.begin(9600);
}

void TasCO2::setCallback(void (*callback1)(String ppm)) {
  _measure_callback = callback1;
}

void TasCO2::chkCO2Data() {
  if (Serial1.available() > 0) {
    // read the incoming byte:
    char c = Serial1.read();
    _sData += c;
  }

  if (_sData.length() >= 8) {
    Serial.print("Read: ");
    Serial.println(_sData);

    char buff[9];

    _sData.toCharArray(buff, 8);

    int p4 = buff[3] & 0xff;
    int p5 = buff[4] & 0xff;

    _co2Value = (p4 << 8) | p5;

    _sData = "";
    
	curValue = String(_co2Value);
    if(_measure_callback != NULL) {
      _measure_callback(curValue);
    }
  }
}

void TasCO2::requestData() {
  Serial1.write(write_buff, 4);
}

