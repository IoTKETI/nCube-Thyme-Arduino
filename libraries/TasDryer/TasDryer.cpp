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
    TasDryer.cpp - Library for monitoring and control for Dryer
    Created by Ilyeup Ahn in KETI Korea, Sep. 19, 2017.
*/

#include <Arduino.h>
#include <SPI.h>

#include "TasDryer.h"

#include "HX711.h"
#include <DFRobotHighTemperatureSensor.h>
#include <Adafruit_LiquidCrystal.h>

#include <Wire.h>

#include <MuxShield.h>

#include <FlashStorage.h>

//Initialize the Mux Shield
MuxShield muxShield;

typedef struct _DryerInfo_t {
    float_t calibration_factor;
    float_t calibration_value;
} DryerInfo_t;

FlashStorage(my_flash_store, DryerInfo_t);

#define DEBUG_PRINT         0

#define LOADCELL_DOUT_PIN   6
#define LOADCELL_CLK_PIN    5

#define MICRO_LATCH_PIN     10
#define MICRO_CLOCK_PIN     11
#define MICRO_DATA_PIN      12

#define STIRRER_TXD_PIN     1
#define STIRRER_RXD_PIN     0

//#define DRY_RATE0_PIN       22

#define BUZZER_PIN          13
#define WIFI_SELECT_PIN     12
#define AC_CURRENT4_PIN     11
#define AC_CURRENT3_PIN     10
#define AC_CURRENT2_PIN     9
#define AC_CURRENT1_PIN     8
#define PT100_PIN           7
#define OUTPUT_DOOR_PIN     6
#define INPUT_DOOR_PIN      5
#define LOADCELL_DOWN_BTN_PIN  	4
#define LOADCELL_UP_BTN_PIN  	3
#define STATUS_CTRL_BTN_PIN    	2
#define DEBUG_SELECT_BTN_PIN    1
#define START_BTN_PIN       0


#define OPEN        true
#define CLOSE       false

#define STOP        0
#define COUNTERTURN 1
#define HIGH_TURN   2
#define TURN        3

#define UP          false
#define DOWN        true
#define RELEASE     2
#define DOWNING     3

#define NORMAL      0
#define OVERLOAD    1
#define CLOSE_OUTPUT_DOOR   2
#define OPEN_OUTPUT_DOOR    3
#define LOW_LOAD    4
#define DISCHARGING         5
#define STIRRER		6
#define MICRO		7
#define EMERGENCY		8
#define INPUT_DOOR_OPEN		9
#define ERROR       10
#define END       11
#define INPUT_DOOR_CLOSE       12
#define DISCHARGE       13
#define INIT            14
#define OPEN_INPUT_DOOR    15
#define DEBUG    16
#define MICRO_MODE  17
#define CLOSE_INPUT_DOOR 18
#define END2INPUT 19

#define DOOR_OPEN       1
#define DOOR_CLOSE      2
#define DOOR_STOP       0


#define MICRO_MODE1     1
#define MICRO_MODE2     2
#define MICRO_MODE3     3

#define OFF             false
#define ON              true

#define POOR            false
#define GOOD            true


#define TIME_STIRRER    60 // sec
#define TIME_MICRO      60 // sec
#define TIME_MICRO_FIRST    40 // times = TIME_MICRO * TIME_MICRO_FIRST sec
#define TIME_BUZZER     60 // sec


//float_t calibration_factor = 3274.0; //This value is obtained using the SparkFun_HX711_Calibration sketch // get_loadcell
HX711 scale(LOADCELL_DOUT_PIN, LOADCELL_CLK_PIN);
uint8_t scale_count = 8;
float_t scale_avg = 0;
float_t temperature_avg = 0;

const float voltageRef = 3.300;       //Set reference voltage,you need test your IOREF voltage.
//const float voltageRef = 3.300;
DFRobotHighTemperature PT100 = DFRobotHighTemperature(voltageRef); //Define an PT100 object

Adafruit_LiquidCrystal lcd(0);

TasDryer::TasDryer() {
    _dryerState = STATE_INIT;
}

#define LED_RGB_R 0
#define LED_RGB_G 1
#define LED_RGB_B 2
#define LED_INPUT_STATE 3
#define LED_STIRRER_STATE 4
#define LED_MICRO_STATE 5
#define LED_DISCHARGE_STATE 6
#define LED_END_STATE 7
#define MICRO1_PIN 8
#define MICRO2_PIN 9
#define MICRO3_PIN 10
#define MICRO4_PIN 11
#define COOLER_PIN 12
#define DEODORIZE_PIN 13

DryerInfo_t dryerInfo;

void TasDryer::begin() {
    Serial1.begin(9600);

    muxShield.setMode(1,ANALOG_IN);  //set I/O 1 as analog input
    muxShield.setMode(2,DIGITAL_OUT);        //set I/O 2 as digital output
    muxShield.setMode(3,DIGITAL_OUT);          //set I/O 3 as digital output

    //Set a few outputs on IO2
    muxShield.digitalWriteMS(2,LED_RGB_R,LOW);  //IO2, pin 2
    muxShield.digitalWriteMS(2,LED_RGB_G,LOW);  //IO2, pin 3
    muxShield.digitalWriteMS(2,LED_RGB_B,LOW);  //IO2, pin 2
    muxShield.digitalWriteMS(2,LED_INPUT_STATE,HIGH);  //IO2, pin 3
    muxShield.digitalWriteMS(2,LED_STIRRER_STATE,HIGH);  //IO2, pin 2
    muxShield.digitalWriteMS(2,LED_MICRO_STATE,HIGH);  //IO2, pin 3
    muxShield.digitalWriteMS(2,LED_DISCHARGE_STATE,HIGH);  //IO2, pin 2
    muxShield.digitalWriteMS(2,LED_END_STATE,HIGH);  //IO2, pin 3
    muxShield.digitalWriteMS(2,MICRO1_PIN,LOW);  //IO2, pin 2
    muxShield.digitalWriteMS(2,MICRO2_PIN,LOW);  //IO2, pin 3
    muxShield.digitalWriteMS(2,MICRO3_PIN,LOW);  //IO2, pin 2
    muxShield.digitalWriteMS(2,MICRO4_PIN,LOW);  //IO2, pin 3
    muxShield.digitalWriteMS(2,COOLER_PIN,LOW);  //IO2, pin 2
    muxShield.digitalWriteMS(2,DEODORIZE_PIN,LOW);  //IO2, pin 3
    muxShield.digitalWriteMS(2,14,LOW);  //IO2, pin 2
    muxShield.digitalWriteMS(2,15,LOW);  //IO2, pin 3

    muxShield.digitalWriteMS(3,0,HIGH);  //IO2, pin 2
    muxShield.digitalWriteMS(3,1,HIGH);  //IO2, pin 3
    muxShield.digitalWriteMS(3,2,HIGH);  //IO2, pin 2
    muxShield.digitalWriteMS(3,3,HIGH);  //IO2, pin 3
    muxShield.digitalWriteMS(3,4,HIGH);  //IO2, pin 2
    muxShield.digitalWriteMS(3,5,HIGH);  //IO2, pin 3
    muxShield.digitalWriteMS(3,6,HIGH);  //IO2, pin 2
    muxShield.digitalWriteMS(3,7,HIGH);  //IO2, pin 3
    muxShield.digitalWriteMS(3,8,HIGH);  //IO2, pin 2
    muxShield.digitalWriteMS(3,9,HIGH);  //IO2, pin 3
    muxShield.digitalWriteMS(3,10,HIGH);  //IO2, pin 2
    muxShield.digitalWriteMS(3,11,HIGH);  //IO2, pin 3
    muxShield.digitalWriteMS(3,12,HIGH);  //IO2, pin 2
    muxShield.digitalWriteMS(3,13,HIGH);  //IO2, pin 3
    muxShield.digitalWriteMS(3,14,HIGH);  //IO2, pin 2
    muxShield.digitalWriteMS(3,15,HIGH);  //IO2, pin 3

    lcd.begin(20, 4);

    lcd.clear();

    _w0 = 0.0;           // 투입중량
    _w1 = 0.5;           // 현재중량
    _w2 = 0.0;           // 추가투입중량 Kg
    _maxW = 100.0;       // 최대중량 Kg
    _targetW = 0.0;      // 목표중량

    _curW2 = 0.0;
    _preW2 = 0.0;
    _thresholdW  = 100.0;

    _dryRate = 0.17;     // 건조도

    _buzzerTick = 0;
    _buzzerFlag = 0;
    _buzzerInterval = TIME_BUZZER;

    _coolerTick = 0;
    _disTick = 0;
    _coolerFlag = 0;

    _preTick = 0;
    _interval = TIME_MICRO;

    _turnError = 0;

    _microIdx = 0;
    _microMode = 0;
    _microTick = 0;

    _startBtnDownCount = 0;
    _startBtnDownFlag = 0;

    _inDoorOpenCount = 0;
    _inDoorFlag = 0;

    _outDoorOpenCount = 0;
    _outDoorFlag = 0;

    _loadUpCount = 0;
    _loadUpFlag = 0;

    _statusCount = 0;
    _statusFlag = 0;

    _loadDownCount = 0;
    _loadDownFlag = 0;

    _loadcell_calibration_lcd_previousMillis = 0;
    _loadcell_calibration_lcd_interval = 1 * 1000;

    _preStirrerCurrent = 1;
    _stirrerCurrent = 0;

    _microStatus = GOOD;

    _dryerEvent = 0;

    _debugSelCount = 0;
    _debugSelFlag = 0;
    _debugSelStatus = 0;

    // DEODORIZE PIN 설정
/*    pinMode(DEODORIZE_PIN, OUTPUT);
    digitalWrite(DEODORIZE_PIN, LOW);

    // MICRO 제어 PIN 설정
    pinMode(MICRO_LATCH_PIN, OUTPUT);
    pinMode(MICRO_CLOCK_PIN, OUTPUT);
    pinMode(MICRO_DATA_PIN, OUTPUT);

    digitalWrite(MICRO_LATCH_PIN, LOW);
    shiftOut(MICRO_DATA_PIN, MICRO_CLOCK_PIN, LSBFIRST, _microCtrl);
    digitalWrite(MICRO_LATCH_PIN, HIGH);

    updateShiftRegister();*/

    // dryRate 설정 PIN 설정
  //    pinMode(DRY_RATE0_PIN, INPUT);

  //   Serial.println("HX711 calibration sketch");
  // Serial.println("Remove all weight from scale");
  // Serial.println("After readings begin, place known weight on scale");
  // Serial.println("Press + or a to increase calibration factor");
  // Serial.println("Press - or z to decrease calibration factor");
  //
  // scale.set_scale();
  // scale.tare(); //Reset the scale to 0
  //
  // long zero_factor = scale.read_average(); //Get a baseline reading
  // Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  // Serial.println(zero_factor);

    dryerInfo = my_flash_store.read();

    if(dryerInfo.calibration_factor == 0) {
        dryerInfo.calibration_factor = -2004.0;
        dryerInfo.calibration_value = -1738.1;
        my_flash_store.write(dryerInfo);
    }

    // loadcell 초기화
    scale.set_scale(dryerInfo.calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
    // scale.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0

    #if DEBUG_PRINT
    Serial.print("SCALE_INIT");
    #endif

    _dryerState = STATE_INIT;

    _debugStatus = STATE_INIT;
}

uint8_t TasDryer::get_wifi_select() {
    int analog1 = muxShield.analogReadMS(1, WIFI_SELECT_PIN);  //IO3, pin 12

    if(analog1 > 512) {
        return 1;
    }
    else {
        return 0;
    }
}

uint8_t TasDryer::get_debug_select() {
    uint8_t curStatus = 0;
    int analog1 = muxShield.analogReadMS(1, DEBUG_SELECT_BTN_PIN);  //IO3, pin 12
    if(analog1 > 512) {
        curStatus = 1;
    }
    else {
        curStatus = 0;
    }

    if (curStatus == LOW && _debugSelFlag == UP) {
        _debugSelCount = 0;
        _debugSelFlag = DOWN;
    }
    else if (curStatus == LOW && _debugSelFlag == DOWN) {
        _debugSelCount++;
        if (_debugSelCount >= 16) {
            _debugSelCount = 16;
            _debugSelStatus = 0;
        }
    }
    else if (curStatus == HIGH && _debugSelFlag == DOWN) {
        _debugSelCount = 0;
        _debugSelFlag = UP;
    }
    else if (curStatus == HIGH && _debugSelFlag == UP) {
        _debugSelCount++;
        if (_debugSelCount >= 16) {
            _debugSelCount = 16;
            _debugSelStatus = 1;
        }
    }

    return _debugSelStatus;
}

/**
* @brief Dryer 현재 상태 LCD에 표시
* @param 없음
* @return 없음
*/
void TasDryer::print_info_lcd() {
    if(_preDryerState != _dryerState) {
        lcd.setCursor(0,1);
        lcd.print("   ");
        lcd.setCursor(0,1);
        lcd.print(_dryerState);

        _preDryerState = _dryerState;
    }

    if(_preW1 != _w1) {
        lcd.setCursor(4,1);
        lcd.print("     ");
        lcd.setCursor(4,1);
        lcd.print(_w1, 1);

        _preW1 = _w1;
    }

    if(_preW0 != _w0) {
        lcd.setCursor(9,1);
        lcd.print("     ");
        lcd.setCursor(9,1);
        lcd.print(_w0, 1);

        _preW0 = _w0;
    }

    if(_preTargetW != _targetW) {
        lcd.setCursor(15,1);
        lcd.print("     ");
        lcd.setCursor(15,1);
        lcd.print(_targetW, 1);

        _preTargetW = _targetW;
    }

 //   if(_preMicroMode != _microMode) {
 //       lcd.setCursor(0,2);
 //       lcd.print("     ");
 //       lcd.setCursor(0,2);
 //       lcd.print(_microMode);

 //       _preMicroMode = _microMode;
 //   }

    // if(_preMicroIdx != _microIdx) {
    //     lcd.setCursor(15,2);
    //     lcd.print("     ");
    //     lcd.setCursor(15,2);
    //     lcd.print(_microIdx);

    //     _preMicroIdx = _microIdx;
    // }

    if(_preCurTick != _curTick) {
        lcd.setCursor(0,3);
        lcd.print("     ");
        lcd.setCursor(0,3);
        lcd.print(_curTick);

        _preCurTick = _curTick;
    }

    if(_preTemperature != _temperature) {
        lcd.setCursor(10,3);
        lcd.print("     ");
        lcd.setCursor(10,3);
        lcd.print(_temperature);

        _preTemperature = _temperature;
    }

    if(_preStirrerCurrent != _stirrerCurrent) {
        lcd.setCursor(15,3);
        lcd.print("     ");
        lcd.setCursor(15,3);
        lcd.print(_stirrerCurrent);

        _preStirrerCurrent = _stirrerCurrent;
    }
}

uint8_t inverterFrame[32];
uint8_t responseIdx = 0;
uint8_t responseFrame[32];
uint8_t flagFrame = 0;

uint8_t _stirrerFlag = 0;

String TasDryer::getDryerStatus() {
    String status = "";

    if(_dryerState == STATE_INPUT) {
        status += "IN";
    }
    else if(_dryerState == STATE_DOOR) {
        status += "DO";
    }
    else if(_dryerState == STATE_STIRRER) {
        status += "ST";
    }
    else if(_dryerState == STATE_MICRO) {
        status += "MI";
    }
    else if(_dryerState == STATE_DISCHARGE) {
        status += "DI";
    }
    else if(_dryerState == STATE_END) {
        status += "EN";
    }
    status += "; ";

    status += String(_w1);
    status += "; ";

    status += String(_targetW);
    status += "; ";

    status += String(_temperature);
    status += "; ";

    status += String(_stirrerCurrent);
    status += "; ";

    return status;
}

/**
* @brief Dryer 장치 구동 루프 함수
* @details 장치의 각 8개 상태에 따른 동작 구현
* @param 없음
* @return 없음
*/
uint8_t test_count = 0;
uint32_t dryer_previousMillis = 0;
uint32_t dryer_interval = 1000;
uint32_t dryer_tick = 0;

uint8_t input_door_status = CLOSE;
uint8_t output_door_status = CLOSE;

void TasDryer::loop() {
    get_debug_select();

    if(_dryerState == STATE_DEBUG) {
        get_loadcell_button();
        get_status_button();
        debug();

        scale.set_scale(dryerInfo.calibration_factor); //Adjust to this calibration factor

        Serial.print("Reading: ");
        Serial.print(scale.get_units()*0.453592, 1);
        Serial.print(" kg"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
        Serial.print(" calibration_factor: ");
        Serial.print(dryerInfo.calibration_factor);
        Serial.println();

        if(Serial.available())
        {
            char temp = Serial.read();
            if(temp == '+' || temp == 'a')
                dryerInfo.calibration_factor += 10;
            else if(temp == '-' || temp == 'z')
                dryerInfo.calibration_factor -= 10;
        }
    }
    else {
        unsigned long currentMillis = millis();
        if (currentMillis - dryer_previousMillis >= dryer_interval) {
            dryer_previousMillis = currentMillis;

            dryer_tick++;

            if(dryer_tick%2 == 0) {
                chk_pt100();
            }

            if(dryer_tick%10 == 0) {
                _dryerEvent |= EVENT_DRYER_STIRRER_TICK;
            }

            chk_loadcell();
            get_current_micro();
            print_info_lcd();

            _curTick = dryer_tick;
        }
        else {
            //chk_discharge_door();
            chk_buzzer();
            get_start_button();
            input_door_status = get_input_door();
            output_door_status = get_output_door();
            get_stirrer_current();

            if(_dryerState == STATE_INIT) {
                init();
            }
            else if(_dryerState == STATE_ERROR) {
                error();
            }
            else if(_dryerState == STATE_INPUT) {
                input();
            }
            else if(_dryerState == STATE_DOOR) {
                door();
            }
            else if(_dryerState == STATE_STIRRER) {
                stirrer();
            }
            else if(_dryerState == STATE_MICRO) {
                micro();
            }
            else if(_dryerState == STATE_DISCHARGE) {
                discharge();
            }
            else if(_dryerState == STATE_END) {
                end();
            }
            else if(_dryerState == STATE_EMERGENCY) {
                emergency();
            }
            else {
                _dryerState = _preDryerState;
            }
        }
    }
}

/**
* @brief 장치 초기화
* @param 없음
* @return 없음
*/
void TasDryer::init() {
    #if DEBUG_PRINT
    Serial.println("DRYER_STATE : INIT");
    #endif

    // @todo 센서 체킹
    uint8_t check_count = 0;

    _w0 = _w1;
    if(_w0 >= 0.0) {
        check_count++;
    }

    if(get_dry_rate() >= 0.0) {
        check_count++;
    }

    if(check_count == 0) {
        lcd.setCursor(0,0);
        lcd.print("                    ");
        lcd.setCursor(0,0);
        lcd.print("ERROR <-- INIT");

        before_error(STATE_INIT);
    }
    else {
        if(_debugSelStatus == 1) {
            lcd.clear();
            lcd_init2debug_log();
            lcd_status = DEBUG;

            before_debug();
            return;
        }
        else {
            if(input_door_status == CLOSE && output_door_status == CLOSE) {
                lcd_init2input_log();
                lcd_status = INIT;

                before_input();
                return;
            }
            else {
                if(input_door_status == OPEN) {
                    if(lcd_status != OPEN_INPUT_DOOR) {
                        lcd_input_door_log();
                        lcd_status = OPEN_INPUT_DOOR;
                    }
                }
                else if(output_door_status == OPEN) {
                    if(lcd_status != OPEN_OUTPUT_DOOR) {
                        lcd_output_door_log();
                        lcd_status = OPEN_OUTPUT_DOOR;
                    }
                }
            }
        }
    }
}

void TasDryer::before_debug() {
    _dryerEvent = 0;
    stirrer_status = TURN;
    set_stirrer(STOP);
    off_all_cooler();
    off_all_power_supply();

    _preW0 = 0.0;
    _w0 += _w2;
    _targetW = _w0 * _dryRate;

    _dryerState = STATE_DEBUG;

    _debugStatus = STATE_INIT;

    muxShield.digitalWriteMS(2,LED_INPUT_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_STIRRER_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_MICRO_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_DISCHARGE_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_END_STATE,HIGH);

    lcd.setCursor(10,1);
    lcd.print(dryerInfo.calibration_factor);
    lcd.setCursor(0,1);
    lcd.print(get_loadcell());
    lcd.setCursor(0,2);
    lcd.print("INIT");
}

float_t cal_gap = 10.0;
void TasDryer::debug() {
    if(_debugSelStatus == 0) {
        lcd.clear();
        lcd_debug2input_log();
        lcd_status = INPUT;

        off_buzzer();
        before_input();
        return;
    }
    else {
        unsigned long currentMillis = millis();
        if (currentMillis - _loadcell_calibration_lcd_previousMillis >= _loadcell_calibration_lcd_interval) {
            _loadcell_calibration_lcd_previousMillis = currentMillis;

            lcd.setCursor(0,1);
            lcd.print("                    ");
            scale.set_scale(dryerInfo.calibration_factor);
            lcd.setCursor(0,1);
            lcd.print(get_loadcell());
            lcd.setCursor(10,1);
            lcd.print(dryerInfo.calibration_factor);
        }
        else {
            if(_dryerEvent & EVENT_LOADCELL_BTN_UP_CLICK) {
                _dryerEvent &= ~EVENT_LOADCELL_BTN_UP_CLICK;
                cal_gap = 10.0;
                dryerInfo.calibration_factor += 1.0;

                lcd.setCursor(0,1);
                lcd.print("                    ");
                scale.set_scale(dryerInfo.calibration_factor);
                lcd.setCursor(0,1);
                lcd.print(get_loadcell());
                lcd.setCursor(10,1);
                lcd.print(dryerInfo.calibration_factor);
            }
            else if(_dryerEvent & EVENT_LOADCELL_BTN_UP_PRESSED) {
                _dryerEvent &= ~EVENT_LOADCELL_BTN_UP_PRESSED;
                dryerInfo.calibration_factor += cal_gap;
                cal_gap += 5;

                lcd.setCursor(0,1);
                lcd.print("                    ");
                scale.set_scale(dryerInfo.calibration_factor);
                lcd.setCursor(0,1);
                lcd.print(get_loadcell());
                lcd.setCursor(10,1);
                lcd.print(dryerInfo.calibration_factor);
            }
            else if(_dryerEvent & EVENT_LOADCELL_BTN_DOWN_CLICK) {
                _dryerEvent &= ~EVENT_LOADCELL_BTN_DOWN_CLICK;
                cal_gap = 10.0;
                dryerInfo.calibration_factor -= 1.0;

                lcd.setCursor(0,1);
                lcd.print("                    ");
                scale.set_scale(dryerInfo.calibration_factor);
                lcd.setCursor(0,1);
                lcd.print(get_loadcell());
                lcd.setCursor(10,1);
                lcd.print(dryerInfo.calibration_factor);
            }
            else if(_dryerEvent & EVENT_LOADCELL_BTN_DOWN_PRESSED) {
                _dryerEvent &= ~EVENT_LOADCELL_BTN_DOWN_PRESSED;
                dryerInfo.calibration_factor -= cal_gap;
                cal_gap += 5;

                lcd.setCursor(0,1);
                lcd.print("                    ");
                scale.set_scale(dryerInfo.calibration_factor);
                lcd.setCursor(0,1);
                lcd.print(get_loadcell());
                lcd.setCursor(10,1);
                lcd.print(dryerInfo.calibration_factor);
            }
            else if(_dryerEvent & EVENT_LOADCELL_BTN_UP_DOWN_PRESSED) {
                _dryerEvent &= ~EVENT_LOADCELL_BTN_UP_DOWN_PRESSED;

                scale.set_scale(dryerInfo.calibration_factor);
                dryerInfo.calibration_value += get_loadcell();
                // scale.tare();

                my_flash_store.write(dryerInfo);
                lcd.setCursor(0,1);
                lcd.print("                    ");
            }
            else if(_dryerEvent & EVENT_STATUS_BTN_CLICK) {
                _dryerEvent &= ~EVENT_STATUS_BTN_CLICK;

                if(_debugStatus == STATE_INPUT) {
                    _debugStatus = STATE_STIRRER;
                    muxShield.digitalWriteMS(2, LED_INPUT_STATE, HIGH);
                    muxShield.digitalWriteMS(2, LED_STIRRER_STATE, LOW);
                    lcd.setCursor(0,2);
                    lcd.print("                    ");
                    lcd.setCursor(0,2);
                    lcd.print("STIRRER");
                }
                else if(_debugStatus == STATE_STIRRER) {
                    _debugStatus = STATE_MICRO;
                    muxShield.digitalWriteMS(2, LED_STIRRER_STATE, HIGH);
                    muxShield.digitalWriteMS(2, LED_MICRO_STATE, LOW);
                    lcd.setCursor(0,2);
                    lcd.print("                    ");
                    lcd.setCursor(0,2);
                    lcd.print("MICRO");
                }
                else if(_debugStatus == STATE_MICRO) {
                    _debugStatus = STATE_DISCHARGE;
                    muxShield.digitalWriteMS(2, LED_MICRO_STATE, HIGH);
                    muxShield.digitalWriteMS(2, LED_DISCHARGE_STATE, LOW);
                    lcd.setCursor(0,2);
                    lcd.print("                    ");
                    lcd.setCursor(0,2);
                    lcd.print("DISCHARGE");
                }
                else if(_debugStatus == STATE_DISCHARGE) {
                    _debugStatus = STATE_END;
                    muxShield.digitalWriteMS(2, LED_DISCHARGE_STATE, HIGH);
                    muxShield.digitalWriteMS(2, LED_END_STATE, LOW);
                    lcd.setCursor(0,2);
                    lcd.print("                    ");
                    lcd.setCursor(0,2);
                    lcd.print("END");
                }
                else if(_debugStatus == STATE_END) {
                    _debugStatus = STATE_INIT;
                    muxShield.digitalWriteMS(2, LED_END_STATE, HIGH);
                    lcd.setCursor(0,2);
                    lcd.print("                    ");
                    lcd.setCursor(0,2);
                    lcd.print("INIT");
                }
                else if(_debugStatus == STATE_INIT) {
                    muxShield.digitalWriteMS(2, LED_INPUT_STATE, LOW);
                    _debugStatus = STATE_INPUT;
                    lcd.setCursor(0,2);
                    lcd.print("                    ");
                    lcd.setCursor(0,2);
                    lcd.print("INPUT");
                }
            }
            else {
                if(_debugStatus == STATE_INIT) {
                }
                else if(_debugStatus == STATE_ERROR) {
                }
                else if(_debugStatus == STATE_INPUT) {
                }
                else if(_debugStatus == STATE_DOOR) {
                }
                else if(_debugStatus == STATE_STIRRER) {
                }
                else if(_debugStatus == STATE_MICRO) {
                }
                else if(_debugStatus == STATE_DISCHARGE) {
                }
                else if(_debugStatus == STATE_END) {
                }
                else if(_debugStatus == STATE_EMERGENCY) {
                }
                else {
                }
            }
        }
    }
}

/**
* @brief INPUT 단계로 진입하기 전 작업 구현
* @param 없음
* @return 없음
*/
void TasDryer::before_input() {
    _dryerEvent = 0;
    stirrer_status = TURN;
    set_stirrer(STOP);
    off_all_cooler();
    off_all_power_supply();

    _preW0 = 0.0;
    _w0 += _w2;
    _targetW = _w0 * _dryRate;

    _dryerState = STATE_INPUT;

    muxShield.digitalWriteMS(2,LED_INPUT_STATE,LOW);
    muxShield.digitalWriteMS(2,LED_STIRRER_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_MICRO_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_DISCHARGE_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_END_STATE,HIGH);
}

/**
* @brief Dryer 제품투입 단계 구현
* @param 없음
* @return 없음
*/
void TasDryer::input() {
    if(_debugSelStatus == 1) {
        lcd.clear();
        lcd_init2debug_log();
        lcd_status = DEBUG;

        before_debug();
        return;
    }
    else {
        if(input_door_status == CLOSE && output_door_status == CLOSE) {
            if(lcd_status == OPEN_OUTPUT_DOOR) {
                lcd_input2input_log();
                lcd_status = CLOSE_OUTPUT_DOOR;
            }

            else if(_dryerEvent & EVENT_DRYER_START_BTN_DOWN) {
                _dryerEvent &= ~EVENT_DRYER_START_BTN_DOWN;

                if(_w0 >= 10.0) {
                    if(lcd_status != STIRRER) {
        				lcd_input2stirrer_log();
        				lcd_status = STIRRER;

                        stirrer_status = STOP;
                        set_stirrer(TURN);
        			}
                    before_stirrer();

                    muxShield.digitalWriteMS(2,LED_RGB_R,LOW);
                    muxShield.digitalWriteMS(2,LED_RGB_G,HIGH);
                    muxShield.digitalWriteMS(2,LED_RGB_B,LOW);
                    return;
                }
                else {
                    if(lcd_status != LOW_LOAD) {
                        lcd_low_load_log();
                        lcd_status = LOW_LOAD;
                    }
                }
            }
        }
        else {
            if(input_door_status == OPEN) {
                if(lcd_status != INPUT_DOOR_OPEN) {
                    lcd_input2door_log();
                    lcd_status = INPUT_DOOR_OPEN;
                }
                before_door();
                return;
            }
            else if(output_door_status == OPEN) {
                if(lcd_status != OPEN_OUTPUT_DOOR) {
                    lcd_output_door_log();
                    lcd_status = OPEN_OUTPUT_DOOR;
                }
            }
        }
    }
}


/**
* @brief DOOR 단계로 진입하기 전 작업 구현
* @param 없음
* @return 없음
*/
void TasDryer::before_door() {
    stirrer_status = TURN;
    set_stirrer(STOP);
    off_all_power_supply();

    _w2 = 0.0;
    _preW2 = _w0;

    _preTargetW = 0.0;
    _targetW = _w0 * _dryRate;
    _targetW = round(_targetW*10)/10;

    _dryerState = STATE_DOOR;
}

/**
* @brief Dryer 문열림 (DOOR) 단계 구현
* @param 없음
* @return 없음
*/
void TasDryer::door() {
    if(input_door_status == OPEN && output_door_status == CLOSE) {
        if(lcd_status == OPEN_OUTPUT_DOOR) {
            lcd_door2door_log();
            lcd_status = CLOSE_OUTPUT_DOOR;
        }

        _curW2 = _w1;
        if(_curW2 > 0.1) {
            if(_curW2 > _thresholdW) {
                on_buzzer(TIME_BUZZER);
                if(lcd_status != OVERLOAD) {
                    lcd_overload_log();
                    lcd_status = OVERLOAD;
                }
            }

            _w2 = _curW2 - _preW2;
            if(_w2 < 0.0) {
                _w2 = 0.0;
            }

            if((_w0 + _w2) >= _thresholdW) {
                on_buzzer(TIME_BUZZER);
                if(lcd_status != OVERLOAD) {
                    lcd_overload_log();
                    lcd_status = OVERLOAD;
                }
            }
            else {
                if(lcd_status != INPUT_DOOR_OPEN) {
                    lcd_door2door_log();
                    lcd_status = INPUT_DOOR_OPEN;
                }
            }
        }
    }
    else {
        if(input_door_status == CLOSE) {
            if((_w0 + _w2) >= _thresholdW) {
                on_buzzer(TIME_BUZZER);
                if(lcd_status != OVERLOAD) {
                    lcd_overload_log();
                    lcd_status = OVERLOAD;
                }
            }
            else {
                if(lcd_status != INPUT_DOOR_CLOSE) {
                    lcd_door2input_log();
                    lcd_status = INPUT_DOOR_CLOSE;
                }
                off_buzzer();
                before_input();
                return;
            }
        }
        else if(output_door_status == OPEN) {
            if(lcd_status != OPEN_OUTPUT_DOOR) {
                lcd_output_door_log();
                lcd_status = OPEN_OUTPUT_DOOR;
            }
        }
    }
}


/**
* @brief STIRRER 단계로 진입하기 전 작업 구현
* @param 없음
* @return 없음
*/
void TasDryer::before_stirrer() {
    on_deodorize();
    stirrer_status = STOP;
    set_stirrer(TURN);
    off_all_power_supply();
    set_timeout(TIME_STIRRER); // sec

    muxShield.digitalWriteMS(2,LED_INPUT_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_STIRRER_STATE,LOW);
    muxShield.digitalWriteMS(2,LED_MICRO_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_DISCHARGE_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_END_STATE,HIGH);

    _dryerState = STATE_STIRRER;
}

/**
* @brief Dryer 교반기 (STIRRER) 단계 구현
* @param 없음
* @return 없음
*/
void TasDryer::stirrer() {
    if(input_door_status == CLOSE && output_door_status == CLOSE) {
        if (_curTick - _preTick >= _interval) {
            _preTick = _curTick;

            // timeout
            if(lcd_status != MICRO) {
    			lcd_stirrer2micro_log();
    			lcd_status = MICRO;
    		}
            before_micro();
            muxShield.digitalWriteMS(2,LED_RGB_R,LOW);
            muxShield.digitalWriteMS(2,LED_RGB_G,LOW);
            muxShield.digitalWriteMS(2,LED_RGB_B,HIGH);
            return;
        }
        else {
            if(_dryerEvent & EVENT_DRYER_STIRRER_TICK) {
                _dryerEvent &= ~EVENT_DRYER_STIRRER_TICK;

                request_stirrer_current();
            }
            else if(_dryerEvent & EVENT_DRYER_STIRRER_CURRENT) {
                _dryerEvent &= ~EVENT_DRYER_STIRRER_CURRENT;

                if(get_stirrer() == OVERLOAD) {
                    if(lcd_status != OVERLOAD) {
                        lcd_stirrer_overload_log();
                        lcd_status = OVERLOAD;
                    }

                    _turnError++;
                    if(stirrer_status == COUNTERTURN) {
                        stirrer_status = STOP;
                        set_stirrer(TURN);
                    }
                    else {
                        stirrer_status = STOP;
                        set_stirrer(COUNTERTURN);
                    }

                    if(_turnError >= 6) {
                        if(lcd_status != ERROR) {
                            lcd_stirrer2error_log();
                            lcd_status = ERROR;
                        }

                        before_error(STATE_STIRRER);
                        return;
                    }
                }
                else {
            		_turnError = 0;
                    stirrer_status = STOP;
                    set_stirrer(TURN);
                }
            }
        }
    }
    else {
        if(input_door_status == OPEN) {
            if(lcd_status != INPUT_DOOR_OPEN) {
                lcd_stirrer2door_log();
                lcd_status = INPUT_DOOR_OPEN;
            }
            before_door();
            return;
        }
        else if(output_door_status == OPEN) {
            if(lcd_status != INPUT_DOOR_OPEN) {
                lcd_stirrer2door_log();
                lcd_status = INPUT_DOOR_OPEN;
            }
            before_door();
            return;
        }
    }
}


/**
* @brief MICRO 단계로 진입하기 전 작업 구현
* @param 없음
* @return 없음
*/
void TasDryer::before_micro() {
    on_deodorize();

    stirrer_status = STOP;
    set_stirrer(TURN);

//    _thresholdW = (_w0 + _targetW)/2.0;

    _microMode = MICRO_MODE1;
    _microTick = 0;

    set_micro();
    set_timeout(TIME_MICRO); // 5min

    muxShield.digitalWriteMS(2,LED_INPUT_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_STIRRER_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_MICRO_STATE,LOW);
    muxShield.digitalWriteMS(2,LED_DISCHARGE_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_END_STATE,HIGH);

    _dryerState = STATE_MICRO;
}

/**
* @brief Dryer 건조(MICRO) 단계 구현
* @param 없음
* @return 없음
*/
void TasDryer::micro() {
    if(input_door_status == CLOSE && output_door_status == CLOSE) {
        if (_curTick - _preTick >= _interval) {
            _preTick = _curTick;

            // timeout
            _microTick++;

            set_micro();

            //if(lcd_status != MICRO_MODE) {
    			lcd_micro_mode_log();
    			lcd_status = MICRO_MODE;
    		//}

            if(_w1 < _targetW) {
                if(lcd_status != DISCHARGE) {
                    lcd_micro2dis_log();
                    lcd_status = DISCHARGE;
                }
                before_discharge();
                muxShield.digitalWriteMS(2,LED_RGB_R,HIGH);
                muxShield.digitalWriteMS(2,LED_RGB_G,LOW);
                muxShield.digitalWriteMS(2,LED_RGB_B,LOW);
            }
        }
        else {
            if(_dryerEvent & EVENT_DRYER_STIRRER_TICK) {
                _dryerEvent &= ~EVENT_DRYER_STIRRER_TICK;

                request_stirrer_current();
            }
            else if(_dryerEvent & EVENT_DRYER_STIRRER_CURRENT) {
                _dryerEvent &= ~EVENT_DRYER_STIRRER_CURRENT;

                if(get_stirrer() == OVERLOAD) {
                    if(lcd_status != OVERLOAD) {
                        lcd_micro_overload_log();
                        lcd_status = OVERLOAD;
                    }

                    _turnError++;
                    if(stirrer_status == COUNTERTURN) {
                        stirrer_status = STOP;
                        set_stirrer(TURN);
                    }
                    else {
                        stirrer_status = STOP;
                        set_stirrer(COUNTERTURN);
                    }

                    if(_turnError >= 6) {
                        if(lcd_status != ERROR) {
                            lcd_micro2error_log();
                            lcd_status = ERROR;
                        }

                        before_error(STATE_MICRO);
                        return;
                    }
                }
                else {
            		_turnError = 0;
                    stirrer_status = STOP;
                    set_stirrer(TURN);
                }
            }
        }
    }
    else {
        if(input_door_status == OPEN) {
            if(lcd_status != INPUT_DOOR_OPEN) {
                lcd_micro2door_log();
                lcd_status = INPUT_DOOR_OPEN;
            }
            before_door();
        }
        else if(output_door_status == OPEN) {
            if(lcd_status != INPUT_DOOR_OPEN) {
                lcd_micro2door_log();
                lcd_status = INPUT_DOOR_OPEN;
            }
            before_door();
        }
    }
}


/**
* @brief DISCHARGE 단계로 진입하기 전 작업 구현
* @param 없음
* @return 없음
*/
uint8_t dis_flag = 0;
void TasDryer::before_discharge() {
    stirrer_status = TURN;
    set_stirrer(STOP);
    off_all_power_supply();

    on_buzzer(TIME_BUZZER);

    muxShield.digitalWriteMS(2,LED_INPUT_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_STIRRER_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_MICRO_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_DISCHARGE_STATE,LOW);
    muxShield.digitalWriteMS(2,LED_END_STATE,HIGH);

    dis_flag = 0;
    _dryerState = STATE_DISCHARGE;
}

/**
* @brief Dryer 문열림 (DOOR) 단계 구현
* @param 없음
* @return 없음
*/
void TasDryer::discharge() {
    if(_w1 < 1.0 && dis_flag == DISCHARGING) {
        if(output_door_status == OPEN) {
            if(lcd_status != CLOSE_OUTPUT_DOOR) {
                lcd_dis_close_door_log();
                lcd_status = CLOSE_OUTPUT_DOOR;

                on_buzzer(TIME_BUZZER);
            }
        }
        else {
            on_buzzer(TIME_BUZZER);

            if(lcd_status != END) {
                lcd_dis2end_log();
                lcd_status = END;
            }

            before_end();
            muxShield.digitalWriteMS(2,LED_RGB_R,HIGH);
            muxShield.digitalWriteMS(2,LED_RGB_G,HIGH);
            muxShield.digitalWriteMS(2,LED_RGB_B,HIGH);
        }
    }
    else {
        if(output_door_status == OPEN) {
            off_buzzer();

            if(lcd_status != DISCHARGING) {
                stirrer_status = STOP;
                set_stirrer(TURN);
                lcd_discharge_log();
                lcd_status = DISCHARGING;
                dis_flag = DISCHARGING;

                on_buzzer(TIME_BUZZER);
            }
        }
        else {
            if(lcd_status != OPEN_OUTPUT_DOOR) {
                stirrer_status = TURN;
                set_stirrer(STOP);
                lcd_dis_open_door_log();
                lcd_status = OPEN_OUTPUT_DOOR;

                on_buzzer(TIME_BUZZER);
            }
        }
    }
}

/**
* @brief END 단계로 진입하기 전 작업 구현
* @param 없음
* @return 없음
*/
void TasDryer::before_end() {
    stirrer_status = HIGH_TURN;
    set_stirrer(STOP);
    off_all_power_supply();

    set_timeout(TIME_STIRRER*2); // 10min

    muxShield.digitalWriteMS(2,LED_INPUT_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_STIRRER_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_MICRO_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_DISCHARGE_STATE,HIGH);
    muxShield.digitalWriteMS(2,LED_END_STATE,LOW);

    _dryerState = STATE_END;
    endTimeout = 0;
}

/**
* @brief Dryer 종료 (END) 단계 구현
* @param 없음
* @return 없음
*/
void TasDryer::end() {
    if (_curTick - _preTick >= _interval) {
        _preTick = _curTick;

        // timeout
        set_stirrer(STOP);
        off_deodorize();
        on_buzzer(TIME_BUZZER);
        endTimeout = 1;
    }
    else {
        if(input_door_status == CLOSE && output_door_status == CLOSE) {
            if(endTimeout == 1) {
                if(lcd_status != END2INPUT) {
                    lcd_end2input_log();
                    lcd_status = END2INPUT;
                }

                _thresholdW  = 100.0;
                _w0 = _w1;
                _w2 = 0.0;
                before_input();
                muxShield.digitalWriteMS(2,LED_RGB_R,LOW);
                muxShield.digitalWriteMS(2,LED_RGB_G,LOW);
                muxShield.digitalWriteMS(2,LED_RGB_B,LOW);
                return;
            }
            else {
            }
        }
        else if(input_door_status == OPEN) {
            if(lcd_status != CLOSE_INPUT_DOOR) {
                lcd_end_close_in_door_log();
                lcd_status = CLOSE_INPUT_DOOR;
            }
        }
        else if(output_door_status == OPEN) {
            if(lcd_status != CLOSE_OUTPUT_DOOR) {
                lcd_end_close_out_door_log();
                lcd_status = CLOSE_OUTPUT_DOOR;
            }
        }
    }
}

/**
* @brief ERROR 단계로 진입하기 전 작업 구현
* @param 없음
* @return 없음
*/
void TasDryer::before_error(uint8_t code) {
    set_stirrer(STOP);
    off_all_power_supply();
    off_deodorize();

    _errorCode = code;
    //lcd_error_log();

    _dryerState = STATE_ERROR;
}

/**
* @brief Dryer ERROR 단계 구현
* @param 없음
* @return 없음
*/
void TasDryer::error() {
}

/**
* @brief EMERGENCY 단계로 진입하기 전 작업 구현
* @param 없음
* @return 없음
*/
void TasDryer::before_emergency() {
    set_stirrer(STOP);
    off_all_power_supply();

    on_buzzer(TIME_BUZZER);

    _dryerState = STATE_EMERGENCY;
}

/**
* @brief Dryer emergency 단계 구현
* @param 없음
* @return 없음
*/
void TasDryer::emergency() {
}


/**
* @brief 교반기 인버터 모델 읽기 요청
* @details 장치의 각 8개 상태에 따른 동작 구현
* @param 없음
* @return 없음
*/
void readInverter() {
    uint8_t idx = 0;
    inverterFrame[idx++] = 0x05;

    //0x05 0x30 0x31 0x52 0x30 0x30 0x30 0x30 0x31 0x41 0x34 0x04

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '1';

    inverterFrame[idx++] = 'R';

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';

    inverterFrame[idx++] = '1';

    //inverterFrame[idx++] = 0x41;
    //inverterFrame[idx++] = 0x34;

    uint16_t sum = 0;
    for(int i = 1; i < idx; i++) {
        sum += inverterFrame[i];
    }

    String stringOne =  String(sum&0x00ff, HEX);
    stringOne.toUpperCase();

    inverterFrame[idx++] = stringOne.charAt(0);
    inverterFrame[idx++] = stringOne.charAt(1);

    inverterFrame[idx++] = 0x04;
    flagFrame = 1;

    //Serial.println((char*) inverterFrame);

    for(int i = 0; i < idx; i++) {
        Serial1.write(inverterFrame[i]);
    }

    //    while(Serial1.available() != 0)
    while  (Serial1.available() > 0) {
        // read the incoming byte:
        char c = Serial1.read();
        if(flagFrame == 1) {
            if(c == 0x06 || c == 0x15) {
                responseIdx = 0;
                flagFrame = 2;
                responseFrame[0] = c;
                responseIdx = 1;
            }
        }
        else if(flagFrame == 2) {
            if(c == 0x04) {
                flagFrame = 0;
                responseFrame[responseIdx++] = c;

                //Serial.println((char*)responseFrame);
            }
            else {
                responseFrame[responseIdx++] = c;
                if(responseIdx >= 32) {
                    responseIdx = 0;
                    flagFrame = 0;
                }
            }
        }
        else {
            responseIdx = 0;
        }
    }
}

/**
* @brief 교반기 인버터 출력 전류 읽기 요청
* @details 장치의 각 8개 상태에 따른 동작 구현
* @param 없음
* @return 없음
*/
void TasDryer::request_stirrer_current() {
    uint8_t idx = 0;
    inverterFrame[idx++] = 0x05;

    //0x05 0x30 0x31 0x52 0x30 0x30 0x30 0x30 0x31 0x41 0x34 0x04

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '1';

    inverterFrame[idx++] = 'R';

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '9';

    inverterFrame[idx++] = '1';

    //inverterFrame[idx++] = 0x41;
    //inverterFrame[idx++] = 0x34;

    uint16_t sum = 0;
    for(int i = 1; i < idx; i++) {
        sum += inverterFrame[i];
    }

    String stringOne =  String(sum&0x00ff, HEX);
    stringOne.toUpperCase();

    inverterFrame[idx++] = stringOne.charAt(0);
    inverterFrame[idx++] = stringOne.charAt(1);

    inverterFrame[idx++] = 0x04;
    flagFrame = 1;

    //Serial.println((char*) inverterFrame);

    for(int i = 0; i < idx; i++) {
        Serial1.write(inverterFrame[i]);
    }

    _stirrerFlag = 1;
}

void TasDryer::get_stirrer_current() {
    if(Serial1.available() > 0) {
        // read the incoming byte:
        char c = Serial1.read();
        //Serial.print(c);
        if(flagFrame == 1) {
            if(c == 0x06 || c == 0x15) {
                responseIdx = 0;
                flagFrame = 2;
                responseFrame[0] = c;
                responseIdx = 1;
            }
        }
        else if(flagFrame == 2) {
            if(c == 0x04) {
                flagFrame = 0;
                responseFrame[responseIdx++] = c;

                if(_stirrerFlag == 1) { // read current of inverter
                    uint32_t ampare = 0;
                    if(0x30 <= responseFrame[4] && responseFrame[4] <= 0x39) {
                        ampare = responseFrame[4]-0x30;
                    }
                    else if(0x41 <= responseFrame[4] && responseFrame[4] <= 0x46) {
                        ampare = responseFrame[4]-0x37;
                    }
                    // else {
                    //     ampare = 0;
                    // }
                    ampare <<= 8;
                    if(0x30 <= responseFrame[5] && responseFrame[5] <= 0x39) {
                        ampare |= responseFrame[5]-0x30;
                    }
                    else if(0x41 <= responseFrame[5] && responseFrame[5] <= 0x46) {
                        ampare |= responseFrame[5]-0x37;
                    }
                    // else {
                    //     ampare = 0;
                    // }
                    ampare <<= 8;
                    if(0x30 <= responseFrame[6] && responseFrame[6] <= 0x39) {
                        ampare |= responseFrame[6]-0x30;
                    }
                    else if(0x41 <= responseFrame[6] && responseFrame[6] <= 0x46) {
                        ampare |= responseFrame[6]-0x37;
                    }
                    // else {
                    //     ampare = 0;
                    // }
                    ampare <<= 8;
                    if(0x30 <= responseFrame[7] && responseFrame[7] <= 0x39) {
                        ampare |= responseFrame[7]-0x30;
                    }
                    else if(0x41 <= responseFrame[7] && responseFrame[7] <= 0x46) {
                        ampare |= responseFrame[7]-0x37;
                    }
                    // else {
                    //     ampare = 0;
                    // }

                    // if(ampare >= 50) {
                    //     _stirrerCurrent = _preStirrerCurrent;
                    // }
                    // else {
                        _stirrerCurrent = ampare;
                    // }

                    _dryerEvent |= EVENT_DRYER_STIRRER_CURRENT;
                }
                else if (_stirrerFlag == 2) { // read response of inverter for command of forward
                }
                else if (_stirrerFlag == 3) { // read response of inverter for command of backward
                }
                else if (_stirrerFlag == 4) { // read response of inverter for command of backward
                }
                else if (_stirrerFlag == 5) { // read response of inverter for command of backward
                }
                else { // read response of inverter for command of high forward
                }
            }
            else {
                responseFrame[responseIdx++] = c;
                if(responseIdx >= 32) {
                    responseIdx = 0;
                    flagFrame = 0;
                }
            }
        }
        else {
            responseIdx = 0;
        }
    }
}

/**
* @brief 교반기 인버터 정방향 명령
* @details 장치의 각 8개 상태에 따른 동작 구현
* @param 없음
* @return 없음
*/
void TasDryer::set_stirrer_forward() {
    uint8_t idx = 0;
    inverterFrame[idx++] = 0x05;

    //0x05 0x30 0x31 0x52 0x30 0x30 0x30 0x30 0x31 0x41 0x34 0x04

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '1';

    inverterFrame[idx++] = 'W';

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '5';

    inverterFrame[idx++] = '2';

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = 'F';
    inverterFrame[idx++] = 'A';
    inverterFrame[idx++] = '0';

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '2';

    //inverterFrame[idx++] = 0x41;
    //inverterFrame[idx++] = 0x34;

    uint16_t sum = 0;
    for(int i = 1; i < idx; i++) {
        sum += inverterFrame[i];
    }

    String stringOne =  String(sum&0x00ff, HEX);
    stringOne.toUpperCase();

    inverterFrame[idx++] = stringOne.charAt(0);
    inverterFrame[idx++] = stringOne.charAt(1);

    inverterFrame[idx++] = 0x04;
    flagFrame = 1;

    //Serial.println((char*) inverterFrame);

    for(int i = 0; i < idx; i++) {
        Serial1.write(inverterFrame[i]);
    }

    _stirrerFlag = 2;
}

/**
* @brief 교반기 인버터 정방향 명령
* @details 장치의 각 8개 상태에 따른 동작 구현
* @param 없음
* @return 없음
*/
void TasDryer::set_stirrer_high_forward() {
    uint8_t idx = 0;
    inverterFrame[idx++] = 0x05;

    //0x05 0x30 0x31 0x52 0x30 0x30 0x30 0x30 0x31 0x41 0x34 0x04

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '1';

    inverterFrame[idx++] = 'W';

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '5';

    inverterFrame[idx++] = '2';

    // inverterFrame[idx++] = '1';
    // inverterFrame[idx++] = '7';
    // inverterFrame[idx++] = '7';
    // inverterFrame[idx++] = '0';

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = 'F';
    inverterFrame[idx++] = 'A';
    inverterFrame[idx++] = '0';

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '2';

    //inverterFrame[idx++] = 0x41;
    //inverterFrame[idx++] = 0x34;

    uint16_t sum = 0;
    for(int i = 1; i < idx; i++) {
        sum += inverterFrame[i];
    }

    String stringOne =  String(sum&0x00ff, HEX);
    stringOne.toUpperCase();

    inverterFrame[idx++] = stringOne.charAt(0);
    inverterFrame[idx++] = stringOne.charAt(1);

    inverterFrame[idx++] = 0x04;
    flagFrame = 1;

    //Serial.println((char*) inverterFrame);

    for(int i = 0; i < idx; i++) {
        Serial1.write(inverterFrame[i]);
    }

    _stirrerFlag = 4;
}

/**
* @brief 교반기 인버터 역방향 명령
* @details 장치의 각 8개 상태에 따른 동작 구현
* @param 없음
* @return 없음
*/
void TasDryer::set_stirrer_backward() {
    uint8_t idx = 0;
    inverterFrame[idx++] = 0x05;

    //0x05 0x30 0x31 0x52 0x30 0x30 0x30 0x30 0x31 0x41 0x34 0x04

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '1';

    inverterFrame[idx++] = 'W';

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '5';

    inverterFrame[idx++] = '2';

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = 'F';
    inverterFrame[idx++] = 'A';
    inverterFrame[idx++] = '0';

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '4';

    //inverterFrame[idx++] = 0x41;
    //inverterFrame[idx++] = 0x34;

    uint16_t sum = 0;
    for(int i = 1; i < idx; i++) {
        sum += inverterFrame[i];
    }

    String stringOne =  String(sum&0x00ff, HEX);
    stringOne.toUpperCase();

    inverterFrame[idx++] = stringOne.charAt(0);
    inverterFrame[idx++] = stringOne.charAt(1);

    inverterFrame[idx++] = 0x04;
    flagFrame = 1;

    //Serial.println((char*) inverterFrame);

    for(int i = 0; i < idx; i++) {
        Serial1.write(inverterFrame[i]);
    }

    _stirrerFlag = 3;
}

/**
* @brief 교반기 인버터 정지 명령
* @details 장치의 각 8개 상태에 따른 동작 구현
* @param 없음
* @return 없음
*/
void TasDryer::set_stirrer_stop() {
    uint8_t idx = 0;
    inverterFrame[idx++] = 0x05;

    //0x05 0x30 0x31 0x52 0x30 0x30 0x30 0x30 0x31 0x41 0x34 0x04

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '1';

    inverterFrame[idx++] = 'W';

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '6';

    inverterFrame[idx++] = '1';

    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '0';
    inverterFrame[idx++] = '1';

    //inverterFrame[idx++] = 0x41;
    //inverterFrame[idx++] = 0x34;

    uint16_t sum = 0;
    for(int i = 1; i < idx; i++) {
        sum += inverterFrame[i];
    }

    String stringOne =  String(sum&0x00ff, HEX);
    stringOne.toUpperCase();

    inverterFrame[idx++] = stringOne.charAt(0);
    inverterFrame[idx++] = stringOne.charAt(1);

    inverterFrame[idx++] = 0x04;
    flagFrame = 1;

    //Serial.println((char*) inverterFrame);

    for(int i = 0; i < idx; i++) {
        Serial1.write(inverterFrame[i]);
    }

    _stirrerFlag = 5;
}


/**
* @brief 교반기 내부 온도 체킹
* @param 없음
* @return 없음
*/
void TasDryer::chk_pt100() {
    _temperature = get_pt100();
    #if DEBUG_PRINT
    //Serial.print("temp : ");
    //Serial.println(_temperature);
    #endif
    if(_temperature >= 110) {
        off_all_power_supply();
    }
}

/**
* @brief 로드셀 체킹
* @param 없음
* @return 없음
*/
void TasDryer::chk_loadcell() {
    float_t weight = 3.1;

    weight = get_loadcell();

    //if(weight < 0.0) {
    //    weight = 0.0;
    //}

    //Serial.print("load : ");
    //Serial.println(weight);

    // if(weight <= 0.0) {
    //    weight = 0.0;
    // }

    //scale_avg = ( scale_avg * (scale_count - 1) + weight ) / scale_count; // 평균 계산
    //_w1 = round(scale_avg*10)/10.0;

    _w1 = weight;
}

/**
* @brief 로드셀의 데이터를 읽는 함수
* @param 없음
* @return 로드셀 데이터:float_t
*/
float_t TasDryer::get_loadcell() {
    float_t weight = 3.14;

    weight = (scale.get_units()) * 0.453592; //scale.get_units() returns a float

    weight -= dryerInfo.calibration_value;

    scale_avg = ( scale_avg * (scale_count - 1) + weight ) / scale_count; // 평균 계산

    return round(scale_avg*10)/10.0;

    //return weight;
}

/**
* @brief 건조도 설정값 읽기
* @param 없음
* @return 건조도 값:float_t
*/
float_t TasDryer::get_dry_rate() {
    float_t rate = 0.17;

    // uint8_t val = (2 * digitalRead(DRY_RATE1_PIN) + digitalRead(DRY_RATE0_PIN));
    //
    // if(val == 0) {
    //     rate = 0.17;
    // }
    // else if(val == 1) {
    //     rate = 0.20;
    // }
    // else if(val == 2) {
    //     rate = 0.25;
    // }
    // else if(val == 3) {
    //     rate = 0.30;
    // }

    #if DEBUG_PRINT
    Serial.print("Dry Raet: ");
    Serial.println(rate, 2);
    #endif

    return rate;
}

/**
* @brief 입력문 상태 읽기
* @param 없음
* @return 열림|닫힘:boolean
*/
bool TasDryer::get_input_door() {
    uint8_t curStatus = 1;

    int analog1 = muxShield.analogReadMS(1, INPUT_DOOR_PIN);  //IO3, pin 12

    if(analog1 > 512) {
        curStatus = 1;
    }
    else {
        curStatus = 0;
    }

    if (curStatus == LOW && _inDoorFlag == UP) {
        _inDoorOpenCount = 0;
        _inDoorFlag = DOWN;
    }
    else if (curStatus == LOW && _inDoorFlag == DOWN) {
        _inDoorOpenCount++;
        if (_inDoorOpenCount >= 128) {
            _inDoorOpenCount = 128;
            input_door_status = OPEN;
        }
    }
    else if (curStatus == HIGH && _inDoorFlag == DOWN) {
        _inDoorOpenCount = 0;
        _inDoorFlag = UP;
    }
    else if (curStatus == HIGH && _inDoorFlag == UP) {
        _inDoorOpenCount++;
        if (_inDoorOpenCount >= 128) {
            _inDoorOpenCount = 128;
            input_door_status = CLOSE;
        }
    }

    return input_door_status;
}

/**
* @brief 배출문 상태 읽기
* @param 없음
* @return 열림|닫힘:boolean
*/
bool TasDryer::get_output_door() {
    uint8_t curStatus = 1;

    int analog1 = muxShield.analogReadMS(1, OUTPUT_DOOR_PIN);  //IO3, pin 12

    if(analog1 > 512) {
        curStatus = 1;
    }
    else {
        curStatus = 0;
    }

    if (curStatus == LOW && _outDoorFlag == UP) {
        _outDoorOpenCount = 0;
        _outDoorFlag = DOWN;
    }
    else if (curStatus == LOW && _outDoorFlag == DOWN) {
        _outDoorOpenCount++;
        if (_outDoorOpenCount >= 128) {
            _outDoorOpenCount = 128;
            output_door_status = OPEN;
        }
    }
    else if (curStatus == HIGH && _outDoorFlag == DOWN) {
        _outDoorOpenCount = 0;
        _outDoorFlag = UP;
    }
    else if (curStatus == HIGH && _outDoorFlag == UP) {
        _outDoorOpenCount++;
        if (_outDoorOpenCount >= 128) {
            _outDoorOpenCount = 128;
            output_door_status = CLOSE;
        }
    }

    return output_door_status;
}


/**
* @brief 장치 스타트 버튼 이벤트 처리 함수
* @param 없음
* @return 눌림|안눌림:boolean
*/
bool TasDryer::get_start_button() {
    bool status = UP;
    uint8_t curStatus = 1;

    int analog1 = muxShield.analogReadMS(1, START_BTN_PIN);  //IO3, pin 12

    if(analog1 > 512) {
        curStatus = 1;
    }
    else {
        curStatus = 0;
    }

    if (curStatus == LOW && _startBtnDownFlag == UP) {
        _startBtnDownCount = 0;
        _startBtnDownFlag = DOWN;
    }
    else if (curStatus == LOW && _startBtnDownFlag == DOWN) {
        _startBtnDownCount++;
        if (_startBtnDownCount >= 512) {
            _startBtnDownCount = 512;
            _dryerEvent &= ~EVENT_DRYER_START_BTN_UP;
            _dryerEvent |= EVENT_DRYER_START_BTN_DOWN;
        }
    }
    else if (curStatus == HIGH && _startBtnDownFlag == DOWN) {
        _startBtnDownCount = 0;
        _startBtnDownFlag = UP;
    }

    return status;
}

void TasDryer::get_loadcell_button() {
    uint8_t curStatus1 = 1;
    uint8_t curStatus2 = 1;
    int analog1 = muxShield.analogReadMS(1, LOADCELL_UP_BTN_PIN);  //IO3, pin 12
    int analog2 = muxShield.analogReadMS(1, LOADCELL_DOWN_BTN_PIN);  //IO3, pin 12

    if(analog1 > 512) {
        curStatus1 = 1;
    }
    else {
        curStatus1 = 0;
    }

    if(analog2 > 512) {
        curStatus2 = 1;
    }
    else {
        curStatus2 = 0;
    }

    if (curStatus1 == LOW && _loadUpFlag == DOWN && curStatus2 == LOW && _loadDownFlag == DOWN) {
        _loadUpCount++;
        _loadDownCount++;
        if ((_loadUpCount+_loadDownCount) >= 48) {
            _loadUpCount = 0;
            _loadDownCount = 0;
            _dryerEvent |= EVENT_LOADCELL_BTN_UP_DOWN_PRESSED;
        }
    }
    else {
        if (curStatus1 == LOW && _loadUpFlag == UP) {
            _loadUpCount = 0;
            _loadUpFlag = DOWN;
        }
        else if (curStatus1 == LOW && _loadUpFlag == DOWN && _loadDownFlag == UP) {
            _loadUpCount++;
            if (_loadUpCount >= 24) {
                _loadUpCount = 24;
                _dryerEvent |= EVENT_LOADCELL_BTN_UP_PRESSED;
            }
        }
        else if (curStatus1 == HIGH && _loadUpFlag == DOWN && _loadDownFlag == UP) {
            _loadUpCount = 0;
            _loadUpFlag = UP;
            _dryerEvent |= EVENT_LOADCELL_BTN_UP_CLICK;
        }
        else if (curStatus1 == HIGH) {
            _loadUpCount = 0;
            _loadUpFlag = UP;
        }

        if (curStatus2 == LOW && _loadDownFlag == UP) {
            _loadDownCount = 0;
            _loadDownFlag = DOWN;
        }
        else if (curStatus2 == LOW && _loadDownFlag == DOWN && _loadUpFlag == UP) {
            _loadDownCount++;
            if (_loadDownCount >= 24) {
                _loadDownCount = 24;
                _dryerEvent |= EVENT_LOADCELL_BTN_DOWN_PRESSED;
            }
        }
        else if (curStatus2 == HIGH && _loadDownFlag == DOWN && _loadUpFlag == UP) {
            _loadDownCount = 0;
            _loadDownFlag = UP;
            _dryerEvent |= EVENT_LOADCELL_BTN_DOWN_CLICK;
        }
        else if (curStatus2 == HIGH) {
            _loadDownCount = 0;
            _loadDownFlag = UP;

        }
    }
}

void TasDryer::get_status_button() {
    uint8_t curStatus1 = 1;
    int analog1 = muxShield.analogReadMS(1, STATUS_CTRL_BTN_PIN);  //IO3, pin 12

    if(analog1 > 512) {
        curStatus1 = 1;
    }
    else {
        curStatus1 = 0;
    }

    if (curStatus1 == LOW && _statusFlag == UP) {
        _statusCount = 0;
        _statusFlag = DOWN;
    }
    else if (curStatus1 == LOW && _statusFlag == DOWN) {
        _statusCount++;
        if (_statusCount >= 512) {
            _statusCount = 512;
        }
    }
    else if (curStatus1 == HIGH && _statusFlag == DOWN) {
        _statusCount = 0;
        _statusFlag = UP;
        _dryerEvent |= EVENT_STATUS_BTN_CLICK;
    }
    else if (curStatus1 == HIGH) {
        _statusCount = 0;
        _statusFlag = UP;
    }
}

/**
* @brief 교반기 상태 확인하는 함수
* @param 없음
* @return 과부하|정상:boolean
*/
bool TasDryer::get_stirrer() {
    bool status = NORMAL;

    // todo: overload 상태의 전류 확인
    if(_stirrerCurrent > 100) {
       status = OVERLOAD;
    }

    return status;
}

/**
* @brief 부저 울림 - 타이머 설정
* @param 없음
* @return 없음
*/
void TasDryer::set_cooler_timeout(uint32_t interval) {
    _coolerInterval = interval;
    _coolerTick = _curTick;
}

/**
* @brief 부저 울림 - 타이머 설정
* @param 없음
* @return 없음
*/
void TasDryer::on_buzzer(uint32_t interval) {
    _buzzerInterval = interval;
    _buzzerTick = _curTick;

    tone(BUZZER_PIN, 330);

    _buzzerFlag = 1;
}

/**
* @brief 부저 정지
* @param 없음
* @return 없음
*/
void TasDryer::off_buzzer() {
    if(_buzzerFlag == 1) {
        noTone(BUZZER_PIN);
        _buzzerFlag = 0;
    }
}

/**
* @brief 타이머 확인하고 부저 중지
* @param 없음
* @return 없음
*/
void TasDryer::chk_buzzer() {
    if(_buzzerFlag == 1) {
        if (_curTick - _buzzerTick >= _buzzerInterval) {
            _buzzerTick = _curTick;

            // buzzer timeout
            noTone(BUZZER_PIN);
            _buzzerFlag = 0;
        }
    }
}

/**
* @brief 마그네트론 상태에 따라 쿨러 정지
* @param 없음
* @return 없음
*/
// bool TasDryer::chk_micro_cooler() {
//     if(_coolerFlag == 1) {
//         if (_curTick - _coolerTick >= _coolerInterval) {
//             _coolerTick = _curTick;
//
//             // cooler timeout
//             if(off_micro_cooler() == true) {
//                 _coolerFlag = 0;
//                 return true;
//             }
//             else {
//                 return false;
//             }
//         }
//         else {
//             return false;
//         }
//     }
//     else {
//         return true;
//     }
// }

/**
* @brief 타이머 설정
* @param 없음
* @return 없음
*/
void TasDryer::set_timeout(uint32_t interval) {
    _interval = interval;
    _preTick = _curTick;
}

/**
* @brief 탈취장치 전원 온
* @param 없음
* @return 없음
*/
void TasDryer::on_deodorize() {
    muxShield.digitalWriteMS(2,DEODORIZE_PIN,HIGH);  //IO2, pin 3
}


/**
* @brief 탈취장치 전원 오프
* @param 없음
* @return 없음
*/
void TasDryer::off_deodorize() {
    muxShield.digitalWriteMS(2,DEODORIZE_PIN,LOW);  //IO2, pin 3
    off_all_cooler();
}

/**
* @brief 교반기 회전 제어
* @param 정지, 정회전, 역회전, 최대정회전 : uint8_t
* @return 없음
*/
void TasDryer::set_stirrer(uint8_t ctrl) {
    if(ctrl == STOP) { // 정지
        if(stirrer_status != STOP) {
            set_stirrer_stop();
            stirrer_status = STOP;
        }
    }
    else if(ctrl == COUNTERTURN) { // 역회전
        if(stirrer_status != COUNTERTURN) {
            set_stirrer_backward();
            stirrer_status = COUNTERTURN;
        }
    }
    else if(ctrl == HIGH_TURN) { // 최대정회전
        if(stirrer_status != HIGH_TURN) {
            set_stirrer_high_forward();
            stirrer_status = HIGH_TURN;
        }
    }
    else { // 정회전
        if(stirrer_status != TURN) {
            set_stirrer_forward();
            stirrer_status = TURN;
        }
    }
}

/**
* @brief 모든 마그네트론의 파워서플라이 오프
* @param 없음
* @return 없음
*/
void TasDryer::off_all_power_supply() {
    muxShield.digitalWriteMS(2,MICRO1_PIN,LOW);
    muxShield.digitalWriteMS(2,MICRO2_PIN,LOW);
    muxShield.digitalWriteMS(2,MICRO3_PIN,LOW);
    muxShield.digitalWriteMS(2,MICRO4_PIN,LOW);
}

/**
* @brief 모든 마그네트론의 쿨러 오프
* @param 없음
* @return 없음
*/
void TasDryer::off_all_cooler() {
    muxShield.digitalWriteMS(2,COOLER_PIN,LOW);
}

void TasDryer::on_all_cooler() {
    muxShield.digitalWriteMS(2,COOLER_PIN,HIGH);
}

/**
* @brief 해당 마그네트론의 쿨러 오프
* @param 마그네트론 번호 : uint8_t
* @return 없음
*/
// bool TasDryer::off_micro_cooler() {
//     uint8_t num_off_cooler = 0;
//     for(uint8_t i = 0; i < 4; i++) {
//         if(get_power_supply(i) == OFF) {
//             _microCtrl &= ~(0x01 << i);
//             num_off_cooler++;
//         }
//     }
//     updateShiftRegister();
//
//     if(num_off_cooler >= 4) {
//         return true;
//     }
//     else {
//         return false;
//     }
// }

/**
* @brief 모드에 따른 마그네트론 구동
* @param 없음
* @return 없음
*/
void TasDryer::set_micro() {
    on_all_cooler();

    if(_microMode == MICRO_MODE1) {
        if(_microTick == 0) {
            _microIdx = 0;

            muxShield.digitalWriteMS(2,MICRO1_PIN,HIGH);
            muxShield.digitalWriteMS(2,MICRO2_PIN,HIGH);
            muxShield.digitalWriteMS(2,MICRO3_PIN,HIGH);
            muxShield.digitalWriteMS(2,MICRO4_PIN,HIGH);
        }
        else if( _microTick >= TIME_MICRO_FIRST) {
            _microTick = 0;
            _microIdx++;
            if(_microIdx >= 5) {
                _microIdx = 1;
            }

            if(_microIdx == 1) {
                muxShield.digitalWriteMS(2,MICRO1_PIN,LOW);
                muxShield.digitalWriteMS(2,MICRO2_PIN,HIGH);
                muxShield.digitalWriteMS(2,MICRO3_PIN,HIGH);
                muxShield.digitalWriteMS(2,MICRO4_PIN,HIGH);
            }
            else if(_microIdx == 2) {
                muxShield.digitalWriteMS(2,MICRO1_PIN,HIGH);
                muxShield.digitalWriteMS(2,MICRO2_PIN,LOW);
                muxShield.digitalWriteMS(2,MICRO3_PIN,HIGH);
                muxShield.digitalWriteMS(2,MICRO4_PIN,HIGH);
            }
            else if(_microIdx == 3) {
                muxShield.digitalWriteMS(2,MICRO1_PIN,HIGH);
                muxShield.digitalWriteMS(2,MICRO2_PIN,HIGH);
                muxShield.digitalWriteMS(2,MICRO3_PIN,LOW);
                muxShield.digitalWriteMS(2,MICRO4_PIN,HIGH);
            }
            else if(_microIdx == 4) {
                muxShield.digitalWriteMS(2,MICRO1_PIN,HIGH);
                muxShield.digitalWriteMS(2,MICRO2_PIN,HIGH);
                muxShield.digitalWriteMS(2,MICRO3_PIN,HIGH);
                muxShield.digitalWriteMS(2,MICRO4_PIN,LOW);
            }

            _microMode = MICRO_MODE2;
        }
    }
    else if(_microMode == MICRO_MODE2) {
        _microIdx++;
        if(_microIdx >= 5) {
            _microIdx = 1;
        }

        if(_microIdx == 1) {
            muxShield.digitalWriteMS(2,MICRO1_PIN,LOW);
            muxShield.digitalWriteMS(2,MICRO2_PIN,HIGH);
            muxShield.digitalWriteMS(2,MICRO3_PIN,HIGH);
            muxShield.digitalWriteMS(2,MICRO4_PIN,HIGH);
        }
        else if(_microIdx == 2) {
            muxShield.digitalWriteMS(2,MICRO1_PIN,HIGH);
            muxShield.digitalWriteMS(2,MICRO2_PIN,LOW);
            muxShield.digitalWriteMS(2,MICRO3_PIN,HIGH);
            muxShield.digitalWriteMS(2,MICRO4_PIN,HIGH);
        }
        else if(_microIdx == 3) {
            muxShield.digitalWriteMS(2,MICRO1_PIN,HIGH);
            muxShield.digitalWriteMS(2,MICRO2_PIN,HIGH);
            muxShield.digitalWriteMS(2,MICRO3_PIN,LOW);
            muxShield.digitalWriteMS(2,MICRO4_PIN,HIGH);
        }
        else if(_microIdx == 4) {
            muxShield.digitalWriteMS(2,MICRO1_PIN,HIGH);
            muxShield.digitalWriteMS(2,MICRO2_PIN,HIGH);
            muxShield.digitalWriteMS(2,MICRO3_PIN,HIGH);
            muxShield.digitalWriteMS(2,MICRO4_PIN,LOW);
        }
    }
}

/**
* @brief 교반기 내의 온도값(pt100 온도 센서) 읽기
* @param 없음
* @return 온도:float_t
*/
int TasDryer::get_pt100() {
    int analog1 = muxShield.analogReadMS(1, PT100_PIN);  //IO3, pin 12
    int temperature = PT100.readTemperature2(analog1);  //Get temperature

    temperature_avg = ( temperature_avg * (8 - 1) + temperature ) / 8; // 평균 계산

    return temperature_avg;
}

/**
* @brief 마그네트론의 상태를 체크하기 위해 전체 마그네트론(파워서플라이) 전류의 합계
* @param 없음
* @return 전류값:float_t
*/
#define ACTectionRange 20
void TasDryer::get_current_micro() {
    float ACCurrtntValue = 0;
    unsigned int peakVoltage = 0;
    unsigned int voltageVirtualValue = 0;  //Vrms
    unsigned int Vref = 3300;
    for (int i = 0; i < 5; i++)
    {
        peakVoltage += muxShield.analogReadMS(1, AC_CURRENT1_PIN);   //read peak voltage
        delay(1);
    }
    peakVoltage = peakVoltage / 5;
    voltageVirtualValue = peakVoltage * 0.707;  	//change the peak voltage to the Virtual Value of voltage
    voltageVirtualValue = (voltageVirtualValue * Vref / 1024) / 2;
    ACCurrtntValue = voltageVirtualValue * ACTectionRange;
    _microACCurrtntValue1 = ACCurrtntValue/1000;

    lcd.setCursor(0,2);
    lcd.print("     ");
    lcd.setCursor(0,2);
    lcd.print(_microACCurrtntValue1);

    ACCurrtntValue = 0;
    peakVoltage = 0;
    voltageVirtualValue = 0;
    for (int i = 0; i < 5; i++)
    {
        peakVoltage += muxShield.analogReadMS(1, AC_CURRENT2_PIN);   //read peak voltage
        delay(1);
    }
    peakVoltage = peakVoltage / 5;
    voltageVirtualValue = peakVoltage * 0.707;  	//change the peak voltage to the Virtual Value of voltage
    voltageVirtualValue = (voltageVirtualValue * Vref / 1024) / 2;
    ACCurrtntValue = voltageVirtualValue * ACTectionRange;
    _microACCurrtntValue2 = ACCurrtntValue/1000;

    lcd.setCursor(5,2);
    lcd.print("     ");
    lcd.setCursor(5,2);
    lcd.print(_microACCurrtntValue2);

    ACCurrtntValue = 0;
    peakVoltage = 0;
    voltageVirtualValue = 0;
    for (int i = 0; i < 5; i++)
    {
        peakVoltage += muxShield.analogReadMS(1, AC_CURRENT3_PIN);   //read peak voltage
        delay(1);
    }
    peakVoltage = peakVoltage / 5;
    voltageVirtualValue = peakVoltage * 0.707;  	//change the peak voltage to the Virtual Value of voltage
    voltageVirtualValue = (voltageVirtualValue * Vref / 1024) / 2;
    ACCurrtntValue = voltageVirtualValue * ACTectionRange;
    _microACCurrtntValue3 = ACCurrtntValue/1000;

    lcd.setCursor(10,2);
    lcd.print("     ");
    lcd.setCursor(10,2);
    lcd.print(_microACCurrtntValue3);

    ACCurrtntValue = 0;
    peakVoltage = 0;
    voltageVirtualValue = 0;
    for (int i = 0; i < 5; i++)
    {
        peakVoltage += muxShield.analogReadMS(1, AC_CURRENT4_PIN);   //read peak voltage
        delay(1);
    }
    peakVoltage = peakVoltage / 5;
    voltageVirtualValue = peakVoltage * 0.707;  	//change the peak voltage to the Virtual Value of voltage
    voltageVirtualValue = (voltageVirtualValue * Vref / 1024) / 2;
    ACCurrtntValue = voltageVirtualValue * ACTectionRange;
    _microACCurrtntValue4 = ACCurrtntValue/1000;

    lcd.setCursor(15,2);
    lcd.print("     ");
    lcd.setCursor(15,2);
    lcd.print(_microACCurrtntValue4);
}


/**
* @brief INIT 상태에서 출력 데이터 초기값을 LCD에 출력
* @param 없음
* @return 없음
*/
void TasDryer::lcd_init_log() {
    // @todo: LCD 출력 :  1~4열 초기값 출력
    // 1열: NORMAL
    // 2열: 0 Kg
    // 3열: dryerW0 Kg
    // 4열: 0 A
    lcd.clear();
    lcd.print("NORMAL");
    lcd.setCursor(0,1);
    lcd.print("O Kg");
    lcd.setCursor(0,2);
    String str =  String(_dryRate, 2);
    lcd.print(str);
    lcd.setCursor(0,3);
    lcd.print("0 A");
}

/**
* @brief INPUT 상태에서 LCD 출력
* @param 없음
* @return 없음
*/
void TasDryer::lcd_input2input_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("INPUT <-- INPUT");
}

void TasDryer::lcd_input2door_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("DOOR <-- INPUT");
}

void TasDryer::lcd_input2stirrer_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("STIRRER <-- INPUT");
}

void TasDryer::lcd_init2input_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("INPUT <-- INIT");
}

void TasDryer::lcd_init2debug_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("DEBUG <-- INIT");
}

void TasDryer::lcd_door2input_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("INPUT <-- DOOR");
}

void TasDryer::lcd_debug2input_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("INPUT <-- DEBUG");
}

void TasDryer::lcd_door2door_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("DOOR <-- DOOR");
}

void TasDryer::lcd_stirrer2door_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("DOOR <-- STIRRER");
}

void TasDryer::lcd_stirrer2error_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("ERROR <-- STIRRER");
}

void TasDryer::lcd_stirrer2micro_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("MICRO <-- STIRRER");
}

void TasDryer::lcd_output_door_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("CLOSE OUTUPUT DOOR");
}

void TasDryer::lcd_input_door_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("CLOSE INPUT DOOR");
}

void TasDryer::lcd_low_load_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("INPUT: LOW LOAD");
}

void TasDryer::lcd_overload_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("DOOR: OVERLOAD");
}

void TasDryer::lcd_stirrer_overload_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("STIRRER: OVERLOAD");
}

void TasDryer::lcd_micro_mode_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    if(_microMode == 1) {
        lcd.print("MICRO: M1");
    }
    else {
        lcd.print("MICRO: M2");
    }
}

void TasDryer::lcd_micro_overload_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("MICRO: OVERLOAD");
}

void TasDryer::lcd_micro2error_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("ERROR <-- MICRO");
}

void TasDryer::lcd_micro2door_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("DOOR <-- MICRO");
}

void TasDryer::lcd_micro2dis_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("DISCHARGE <-- MICRO");
}

void TasDryer::lcd_dis_open_door_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("DIS: OPEN OUT_DOOR");
}

void TasDryer::lcd_dis_close_door_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("DIS: CLOSE OUT_DOOR");
}

void TasDryer::lcd_discharge_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("DISCHARGING");
}


void TasDryer::lcd_dis2end_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("END <-- DISCHARGE");
}

void TasDryer::lcd_end_close_out_door_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("END: CLOSE OUT_DOOR");
}

void TasDryer::lcd_end_close_in_door_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("END: CLOSE IN_DOOR");
}

void TasDryer::lcd_end2input_log() {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("INPUT <-- END");
}

/**
* @brief 교반기(STIRRER) 상태에서 LCD 출력
* @param 없음
* @return 없음
*/
void TasDryer::lcd_stirrer_log() {
    // @todo: LCD 출력 :  1열, OVERLOAD, DO NOT ADD LOAD

}

/**
* @brief DOOR 상태에서 LCD 출력 - DOOR OPEN 출력
* @param 출력 케이스:uint8_t
* @return 없음
*/
void TasDryer::lcd_door_log(uint8_t cases) {
    if(cases == 0) {
        // @todo: LCD 출력 :  1열, DOOR OPEN
    }
    else if(cases == 1) {
        // @todo: LCD 출력 :  1열, DO NOT ADD LOAD
    }
    else if(cases == 2) {
        // @todo: LCD 출력 :  1열, OVERLOAD
    }
}

/**
* @brief ERROR 상태에서 LCD 출력
* @param 없음
* @return 없음
*/
void TasDryer::lcd_error_log() {
    // @todo: LCD 출력 :  ERROR, ERROR CODE 출력
//    lcd.clear();
//    lcd.print("ERROR");
//    lcd.setCursor(0,1);
//    lcd.print("CODE : ");
//    lcd.print(_errorCode);
}

/**
* @brief MICRO 상태에서 LCD 출력
* @param 없음
* @return 없음
*/
void TasDryer::lcd_micro_log() {
    // @todo: LCD 출력 :  마그네트론 상태 출력 (CHECK MAGNETRON)
//    lcd.clear();
//    lcd.print("Magnetron state : ");
//    lcd.setCursor(0,1);
//    lcd.print(_microStatus);
}

/**
* @brief DISCHARGE 상태에서 LCD 출력
* @param 없음
* @return 없음
*/
void TasDryer::lcd_discharge_log(uint8_t cases) {
    // @todo: LCD 출력 :  DRYING END or DISCHARGE END

    /*lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);

    if(cases == 0) {
        lcd.print("DRYING END");
    }
    else if(cases == 1) {
        lcd.print("DISCHARGE END");
    }
    else if(cases == 2) {
        lcd.print("DISCHARGING");
    }*/
}

/**
* @brief END 상태에서 LCD 출력
* @param 없음
* @return 없음
*/
void TasDryer::lcd_end_log() {
    // @todo: LCD 출력 :  PROCESS END
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("PROCESS END");
}

/**
* @brief END 상태에서 LCD 출력
* @param 없음
* @return 없음
*/
void TasDryer::lcd_emergency_log() {
    // @todo: LCD 출력 :  PROCESS END
    // lcd.setCursor(0,0);
    // lcd.print("                    ");
    // lcd.setCursor(0,0);
    // lcd.print("EMERGENCY");
}
