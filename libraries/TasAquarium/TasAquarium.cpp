/**
Copyright (c) 2017, OCEAN
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. The name of the author may not be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * Created by Hana Jo in KETI on 2018-03-22.
 */


#include "TasAquarium.h"



#define PHSensor        A0   //ph sensor
#define FeederStatus    9    //FeederStatus
#define FEEDER          10   //Feeder
#define WATER_LEVEL     11   //WATER_LEVEL
#define ONE_WIRE_BUS    12   //temp
#define PIN             13   //LED

#define NUMPIXELS       16
#define Offset          0.00
#define samplingInterval 20
#define printInterval   800
#define ArrayLenth      40

//loop
int count = 0;
int waterlevelVal;
//ph
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;

//Temp
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//lED
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800);


//initialize Feeder

void TasAquarium::initFeeder()
{
    pinMode(FEEDER, OUTPUT);
    pinMode(FeederStatus, INPUT_PULLUP); //feeder status
    digitalWrite(FEEDER, LOW);
    _feeder = 0;
}

//initialize WaterLevel
void TasAquarium::initWaterLevel()
{
    pinMode(WATER_LEVEL, INPUT_PULLUP);
}

//initialize LED
void TasAquarium::initLED()
{
    #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
    #endif
    pixels.begin();
}

//initialize Temp
void TasAquarium::initTemp()
{
    sensors.begin();
}

//to calculate pH
double TasAquarium::avergearray(int* arr, int number)
{
    int i;
    int max,min;
    double avg;
    long amount=0;
    if(number<=0){
        Serial.println("Error number for the array to avraging!/n");
        return 0;
    }
    if(number<5){   //less than 5, calculated directly statistics
        for(i=0;i<number;i++){
            amount+=arr[i];
        }
        avg = amount/number;
        return avg;
    }
    else{
        if(arr[0]<arr[1]){
            min = arr[0];max=arr[1];
        }
        else{
            min=arr[1];max=arr[0];
        }
        for(i=2;i<number;i++){
            if(arr[i]<min){
                amount+=min;        //arr<min
                min=arr[i];
            }else {
                if(arr[i]>max){
                    amount+=max;    //arr>max
                    max=arr[i];
                }else{
                    amount+=arr[i]; //min<=arr<=max
                }
            }//if
        }//for
        avg = (double)amount/(number-2);
    }//if
    return avg;
}

//to show LED
void TasAquarium::showLED()
{
    for(int i=0;i<NUMPIXELS;i++){
        pixels.setPixelColor(i, pixels.Color(0,150,0));
        pixels.show();
    }
}

//to stop LED
void TasAquarium::stopLED()
{
    for(int i=0;i<NUMPIXELS;i++){
        pixels.setPixelColor(i, pixels.Color(0,0,0));
        pixels.show();
    }
}

//to read Temp
float TasAquarium::readTemperature()
{
    sensors.requestTemperatures(); // Send the command to get temperatures
    return sensors.getTempCByIndex(0);
}

//to read WaterLevel
int TasAquarium::readWaterLevel()
{
    return digitalRead(WATER_LEVEL);
}

//to read PH
float TasAquarium::readPHSensor()
{
    static unsigned long samplingTime = millis();
    static unsigned long printTime = millis();
    static float pHValue,voltage;
    if(millis()-samplingTime > samplingInterval)
    {
        pHArray[pHArrayIndex++]=analogRead(PHSensor);
        if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
        voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
        pHValue = 3.5*voltage+Offset;
        samplingTime=millis();
    }
    if(millis() - printTime > printInterval)
    {
        return pHValue;
        printTime=millis();
    }
}

//to read Feeder
int TasAquarium::readFeeder()
{
    return _feeder;
}

//to operate Feeder
void TasAquarium::feedFish(int onoff)
{
    if(onoff == 1) {
        _feeder = 1;
        digitalWrite(FEEDER, HIGH); //Turn On Fish Feeder
    }
    else {
        _feeder = 0;
        digitalWrite(FEEDER, LOW); //Turn Off Fish Feeder
    }
}

//to check FeederStatus
void TasAquarium::loop()
{
    if(_feeder == 1 && digitalRead(FeederStatus) == 1) {
    }
    else if(_feeder == 1 && digitalRead(FeederStatus) == 0){
        count ++;
        if(count > 2048){
            count = 0;
            _feeder = 2;
        }
    }
    else if(_feeder == 2 && digitalRead(FeederStatus) == 1){
        _feeder = 3;
    }
    else if(_feeder == 3 && digitalRead(FeederStatus) == 0){
        digitalWrite(FEEDER,LOW);
        _feeder = 0;
    }
}
