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

#define DEBUG_PRINT         0

#define LOADCELL_DOUT_PIN   6
#define LOADCELL_CLK_PIN    5

#define DEODORIZE_PIN       A1
#define MICRO_LATCH_PIN     10
#define MICRO_CLOCK_PIN     11
#define MICRO_DATA_PIN      12

#define STIRRER_TXD_PIN     1
#define STIRRER_RXD_PIN     0

//#define DRY_RATE0_PIN       22

#define BUZZER_PIN          13

#define START_BTN_PIN       A2
#define EMERGENCY_BTN_PIN   9


#define OUTPUT_DOOR_PIN     A5
#define INPUT_DOOR_PIN      A0
#define DISCHARGE_BTN_PIN  	A3

#define PT100_PIN           A4


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
#define TIME_MICRO      300 // sec
#define TIME_MICRO_FIRST    8 // times = TIME_MICRO * TIME_MICRO_FIRST sec
#define TIME_BUZZER     60 // sec


float_t calibration_factor = 3274.0; //This value is obtained using the SparkFun_HX711_Calibration sketch // get_loadcell
HX711 scale(LOADCELL_DOUT_PIN, LOADCELL_CLK_PIN);
uint8_t scale_count = 4;
float_t scale_avg = 0;
float_t temperature_avg = 0;

//const float voltageRef = 5.000;       //Set reference voltage,you need test your IOREF voltage.
const float voltageRef = 3.300;
DFRobotHighTemperature PT100 = DFRobotHighTemperature(voltageRef); //Define an PT100 object

Adafruit_LiquidCrystal lcd(0);

TasDryer::TasDryer() {
    _dryerState = STATE_INIT;
}

void TasDryer::begin() {
    Serial1.begin(9600);

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

    _microCtrl = 0;
    _microCtrl |= (0x01 << 0);
    _microCtrl |= (0x01 << 1);

    _microPower = 0;

    _startBtnDownCount = 0;
    _startBtnDownFlag = 0;

    _emerBtnDownCount = 0;
    _emerBtnDownFlag = 0;

    _disBtnDownCount = 0;
    _disBtnDownFlag = 0;

    _inDoorOpenCount = 0;
    _inDoorFlag = 0;

    _outDoorOpenCount = 0;
    _outDoorFlag = 0;


    _preStirrerCurrent = 1;
    _stirrerCurrent = 0;

    _microStatus = GOOD;

    _dryerEvent = 0;

    // DEODORIZE PIN 설정
    pinMode(DEODORIZE_PIN, OUTPUT);
    digitalWrite(DEODORIZE_PIN, LOW);

    // MICRO 제어 PIN 설정
    pinMode(MICRO_LATCH_PIN, OUTPUT);
    pinMode(MICRO_CLOCK_PIN, OUTPUT);
    pinMode(MICRO_DATA_PIN, OUTPUT);

    digitalWrite(MICRO_LATCH_PIN, LOW);
    shiftOut(MICRO_DATA_PIN, MICRO_CLOCK_PIN, LSBFIRST, _microCtrl);
    digitalWrite(MICRO_LATCH_PIN, HIGH);

    updateShiftRegister();

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

    // loadcell 초기화
   scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
   //scale.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0
    #if DEBUG_PRINT
    Serial.print("SCALE_INIT");
    #endif

    // BUTTON 초기화
    pinMode(START_BTN_PIN, INPUT_PULLUP);
    pinMode(EMERGENCY_BTN_PIN, INPUT_PULLUP);

    // DOOR 센싱 초기화
    pinMode(INPUT_DOOR_PIN, INPUT_PULLUP);
    pinMode(DISCHARGE_BTN_PIN, INPUT_PULLUP);
    pinMode(OUTPUT_DOOR_PIN, INPUT_PULLUP);

    lcd.begin(20, 4);

    lcd.clear();

    _dryerState = STATE_INIT;
}

/**
* @brief 마그네트론 파워서플라이와 쿨러를 제어는 시프트레지스터 74HC595의 출력을 내보내는 함수
* @details _microCtrl의 각 비트에 해당하는 값으로 온오프 제어
* @param 없음
* @return 없음
*/
void TasDryer::updateShiftRegister() {
    digitalWrite(MICRO_LATCH_PIN, LOW);
    shiftOut(MICRO_DATA_PIN, MICRO_CLOCK_PIN, LSBFIRST, _microCtrl);
    digitalWrite(MICRO_LATCH_PIN, HIGH);
}


/**
* @brief Dryer 현재 상태 LCD에 표시
* @param 없음
* @return 없음
*/
void TasDryer::print_debug_lcd() {
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

    if(_preMicroMode != _microMode) {
        lcd.setCursor(0,2);
        lcd.print("     ");
        lcd.setCursor(0,2);
        lcd.print(_microMode);

        _preMicroMode = _microMode;
    }

    if(_preMicroCtrl != _microCtrl) {
        lcd.setCursor(5,2);
        lcd.print("     ");
        lcd.setCursor(5,2);
        lcd.print(_microCtrl);

        _preMicroCtrl = _microCtrl;
    }

    if(_preMicroPower != _microPower) {
        lcd.setCursor(10,2);
        lcd.print("     ");
        lcd.setCursor(10,2);
        lcd.print(_microPower);

        _preMicroPower = _microPower;
    }

    if(_preMicroIdx != _microIdx) {
        lcd.setCursor(15,2);
        lcd.print("     ");
        lcd.setCursor(15,2);
        lcd.print(_microIdx);

        _preMicroIdx = _microIdx;
    }

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

    status += String(_dryerState);
    status += ";";

    status += String(_w1);
    status += ";";

    status += String(_w0);
    status += ";";

    status += String(_targetW);
    status += ";";

    status += String(_microMode);
    status += ";";

    status += String(_temperature);
    status += ";";

    status += String(_stirrerCurrent);
    status += ";";

    return status;
}

/**
* @brief Dryer 장치 구동 루프 함수
* @details 장치의 각 8개 상태에 따른 동작 구현
* @param 없음
* @return 없음
*/
uint8_t test_count = 0;
void TasDryer::loop(uint32_t tick) {
    if(_curTick != tick) {
        if(tick%2 == 0) {
            chk_pt100();
        }

        chk_loadcell();

        print_debug_lcd();

        if(tick%5 == 0) {
            _dryerEvent |= EVENT_DRYER_STIRRER_TICK;
       }
    }

    _curTick = tick;

    //chk_discharge_door();
    chk_buzzer();
    get_emergency_button();
    get_start_button();
    get_input_door();
    get_discharge_button();
    get_output_door();

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
                    else {
                        ampare = 0;
                    }
                    ampare <<= 8;
                    if(0x30 <= responseFrame[5] && responseFrame[5] <= 0x39) {
                        ampare |= responseFrame[5]-0x30;
                    }
                    else if(0x41 <= responseFrame[5] && responseFrame[5] <= 0x46) {
                        ampare |= responseFrame[5]-0x37;
                    }
                    else {
                        ampare = 0;
                    }
                    ampare <<= 8;
                    if(0x30 <= responseFrame[6] && responseFrame[6] <= 0x39) {
                        ampare |= responseFrame[6]-0x30;
                    }
                    else if(0x41 <= responseFrame[6] && responseFrame[6] <= 0x46) {
                        ampare |= responseFrame[6]-0x37;
                    }
                    else {
                        ampare = 0;
                    }
                    ampare <<= 8;
                    if(0x30 <= responseFrame[7] && responseFrame[7] <= 0x39) {
                        ampare |= responseFrame[7]-0x30;
                    }
                    else if(0x41 <= responseFrame[7] && responseFrame[7] <= 0x46) {
                        ampare |= responseFrame[7]-0x37;
                    }
                    else {
                        ampare = 0;
                    }

                    if(ampare >= 50) {
                        _stirrerCurrent = _preStirrerCurrent;
                    }
                    else {
                        _stirrerCurrent = ampare;
                    }

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
        lcd.setCursor(0,0);
        lcd.print("                    ");
        lcd.setCursor(0,0);
        lcd.print("INPUT <-- INIT");
        lcd_status = INIT;

        before_input();
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
    set_output_door(DOOR_CLOSE);

    _preW0 = 0.0;
    _w0 += _w2;
    _targetW = _w0 * _dryRate;

    _dryerState = STATE_INPUT;
}

/**
* @brief Dryer 제품투입 단계 구현
* @param 없음
* @return 없음
*/
void TasDryer::input() {
    if(_dryerEvent & EVENT_DRYER_EMERGENCY_BTN_DOWN) {
        _dryerEvent &= ~EVENT_DRYER_EMERGENCY_BTN_DOWN;

        lcd.setCursor(0,0);
        lcd.print("                    ");
        lcd.setCursor(0,0);
        lcd.print("EMERGENCY <-- INPUT");
        lcd_status = EMERGENCY;
        before_emergency();
        return;
    }

    if(_dryerEvent & EVENT_DRYER_DIS_BTN_DOWN) {
        _dryerEvent &= ~EVENT_DRYER_DIS_BTN_DOWN;
        set_output_door(DOOR_CLOSE);
    }

    else if(_dryerEvent & EVENT_DRYER_DIS_BTN_UP) {
        _dryerEvent &= ~EVENT_DRYER_DIS_BTN_UP;
        set_output_door(DOOR_OPEN);
    }

    else if(_dryerEvent & EVENT_DRYER_INPUT_DOOR_OPEN) {
        _dryerEvent &= ~EVENT_DRYER_INPUT_DOOR_OPEN;

        if(lcd_status != INPUT_DOOR_OPEN) {
            lcd.setCursor(0,0);
            lcd.print("                    ");
            lcd.setCursor(0,0);
            lcd.print("DOOR <-- INPUT");
            lcd_status = INPUT_DOOR_OPEN;
        }
        before_door();
        return;
    }

    else if(_dryerEvent & EVENT_DRYER_OUTPUT_DOOR_OPEN) {
        _dryerEvent &= ~EVENT_DRYER_OUTPUT_DOOR_OPEN;

        if(lcd_status != OPEN_OUTPUT_DOOR) {
            lcd.setCursor(0,0);
            lcd.print("                    ");
            lcd.setCursor(0,0);
            lcd.print("CLOSE OUTPUT DOOR");
            lcd_status = OPEN_OUTPUT_DOOR;
        }
        return;
    }

    else if(_dryerEvent & EVENT_DRYER_OUTPUT_DOOR_CLOSE) {
        _dryerEvent &= ~EVENT_DRYER_OUTPUT_DOOR_CLOSE;

        if(lcd_status != CLOSE_OUTPUT_DOOR) {
            lcd.setCursor(0,0);
            lcd.print("                    ");
            lcd.setCursor(0,0);
            lcd.print("INPUT <-- INPUT");
            lcd_status = CLOSE_OUTPUT_DOOR;
        }
        return;
    }

    else if(_dryerEvent & EVENT_DRYER_START_BTN_DOWN) {
        _dryerEvent &= ~EVENT_DRYER_START_BTN_DOWN;

        if(_w0 >= 10.0) {
            if(lcd_status != STIRRER) {
				lcd.setCursor(0,0);
				lcd.print("                    ");
				lcd.setCursor(0,0);
				lcd.print("STIRRER <-- INPUT");
				lcd_status = STIRRER;

                stirrer_status = STOP;
                set_stirrer(TURN);
			}
            before_stirrer();
            return;
        }
        else {
            if(lcd_status != LOW_LOAD) {
                lcd.setCursor(0,0);
                lcd.print("                    ");
                lcd.setCursor(0,0);
                lcd.print("LOW LOAD");
                lcd_status = LOW_LOAD;
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
    set_output_door(DOOR_CLOSE);
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
    if(_dryerEvent & EVENT_DRYER_EMERGENCY_BTN_DOWN) {
        _dryerEvent &= ~EVENT_DRYER_EMERGENCY_BTN_DOWN;

        lcd.setCursor(0,0);
        lcd.print("                    ");
        lcd.setCursor(0,0);
        lcd.print("EMERGENCY <-- DOOR");
        lcd_status = EMERGENCY;
        before_emergency();
        return;
    }

    if(_dryerEvent & EVENT_DRYER_INPUT_DOOR_CLOSE) {
        _dryerEvent &= ~EVENT_DRYER_INPUT_DOOR_CLOSE;

        if(lcd_status != INPUT_DOOR_CLOSE) {
            lcd.setCursor(0,0);
            lcd.print("                    ");
            lcd.setCursor(0,0);
            lcd.print("INPUT <-- DOOR");
            lcd_status = INPUT_DOOR_CLOSE;
        }

        off_buzzer();
        before_input();
        return;
    }

    _curW2 = _w1;
    if(_curW2 > 0.1) {
        if(_curW2 > _thresholdW) {
            on_buzzer(TIME_BUZZER);
            //lcd_door_log(2);
        }

        _w2 = _curW2 - _preW2;
        if(_w2 < 0.0) {
            _w2 = 0.0;
        }

        if((_w0 + _w2) >= 100.0) {
            on_buzzer(TIME_BUZZER);
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
    set_output_door(DOOR_CLOSE);
    off_all_power_supply();
    set_timeout(TIME_STIRRER); // sec

    _dryerState = STATE_STIRRER;
}

/**
* @brief Dryer 교반기 (STIRRER) 단계 구현
* @param 없음
* @return 없음
*/
void TasDryer::stirrer() {
    if(_dryerEvent & EVENT_DRYER_EMERGENCY_BTN_DOWN) {
        _dryerEvent &= ~EVENT_DRYER_EMERGENCY_BTN_DOWN;

        lcd.setCursor(0,0);
        lcd.print("                    ");
        lcd.setCursor(0,0);
        lcd.print("EMERGENCY <-- DOOR");
        lcd_status = EMERGENCY;
        before_emergency();
        return;
    }

    if (_curTick - _preTick >= _interval) {
        _preTick = _curTick;

        // timeout
        if(lcd_status != MICRO) {
			lcd.setCursor(0,0);
			lcd.print("                    ");
			lcd.setCursor(0,0);
			lcd.print("MICRO <-- STIRRER");
			lcd_status = MICRO;
			set_stirrer(TURN);
		}
        before_micro();
        return;
    }

    if(_dryerEvent & EVENT_DRYER_INPUT_DOOR_OPEN) {
        _dryerEvent &= ~EVENT_DRYER_INPUT_DOOR_OPEN;

        if(lcd_status != INPUT_DOOR_OPEN) {
            lcd.setCursor(0,0);
            lcd.print("                    ");
            lcd.setCursor(0,0);
            lcd.print("DOOR <-- STIRRER");
            lcd_status = INPUT_DOOR_OPEN;
        }
        before_door();
        return;
    }

    else if(_dryerEvent & EVENT_DRYER_STIRRER_TICK) {
        _dryerEvent &= ~EVENT_DRYER_STIRRER_TICK;

        get_stirrer_current();
        return;
    }

    else if(_dryerEvent & EVENT_DRYER_STIRRER_CURRENT) {
        _dryerEvent &= ~EVENT_DRYER_STIRRER_CURRENT;

        if(get_stirrer() == OVERLOAD) {
            if(lcd_status != OVERLOAD) {
                lcd.setCursor(0,0);
                lcd.print("                    ");
                lcd.setCursor(0,0);
                lcd.print("STIRRER OVERLOAD");
                lcd_status = OVERLOAD;

                _preOverloadTick = _curTick;
    			set_stirrer(COUNTERTURN);
            }

            if(_turnError >= 6) {
                if(lcd_status != ERROR) {
                    lcd.setCursor(0,0);
                    lcd.print("                    ");
                    lcd.setCursor(0,0);
                    lcd.print("ERROR <-- STIRRER");
                    lcd_status = ERROR;
                }

                before_error(STATE_STIRRER);
            }
    		else {
    			if((_curTick - _preOverloadTick) >= 15) {
    				_preOverloadTick = _curTick;

    				if(stirrer_status == COUNTERTURN) {
    					set_stirrer(TURN);
    					_turnError++;
    				}
    				else {
    					set_stirrer(COUNTERTURN);
    					_turnError++;
    				}
    			}
    		}
        }
        else {
    		_turnError = 0;
            stirrer_status = STOP;
            set_stirrer(TURN);
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
    set_stirrer(TURN);
    set_output_door(DOOR_CLOSE);

    _thresholdW = (_w0 + _targetW)/2.0;

    _microMode = MICRO_MODE1;
    _microTick = 0;
    _microIdx = 0;

    set_micro();
    set_timeout(TIME_MICRO); // 5min

    _dryerState = STATE_MICRO;
}

/**
* @brief Dryer 건조(MICRO) 단계 구현
* @param 없음
* @return 없음
*/
void TasDryer::micro() {
    if(_dryerEvent & EVENT_DRYER_EMERGENCY_BTN_DOWN) {
        _dryerEvent &= ~EVENT_DRYER_EMERGENCY_BTN_DOWN;

        lcd.setCursor(0,0);
        lcd.print("                    ");
        lcd.setCursor(0,0);
        lcd.print("EMERGENCY <-- MICRO");
        lcd_status = EMERGENCY;
        before_emergency();
        return;
    }

    if (_curTick - _preTick >= _interval) {
        _preTick = _curTick;

        // timeout
        _microTick++;
        _microIdx++;
        set_micro();
        return;
    }

    if(_dryerEvent & EVENT_DRYER_INPUT_DOOR_OPEN) {
        _dryerEvent &= ~EVENT_DRYER_INPUT_DOOR_OPEN;

        if(lcd_status != INPUT_DOOR_OPEN) {
            lcd.setCursor(0,0);
            lcd.print("                    ");
            lcd.setCursor(0,0);
            lcd.print("DOOR <-- MICRO");
            lcd_status = INPUT_DOOR_OPEN;
        }
        before_door();
        return;
    }

    else if(_dryerEvent & EVENT_DRYER_STIRRER_TICK) {
        _dryerEvent &= ~EVENT_DRYER_STIRRER_TICK;

        get_stirrer_current();
        return;
    }

    else if(_dryerEvent & EVENT_DRYER_STIRRER_CURRENT) {
        _dryerEvent &= ~EVENT_DRYER_STIRRER_CURRENT;

        if(get_stirrer() == OVERLOAD) {
            if(lcd_status != OVERLOAD) {
                lcd.setCursor(0,0);
                lcd.print("                    ");
                lcd.setCursor(0,0);
                lcd.print("STIRRER OVERLOAD");
                lcd_status = OVERLOAD;

                _preOverloadTick = _curTick;
    			set_stirrer(COUNTERTURN);
            }

            if(_turnError >= 6) {
                if(lcd_status != ERROR) {
                    lcd.setCursor(0,0);
                    lcd.print("                    ");
                    lcd.setCursor(0,0);
                    lcd.print("ERROR <-- MICRO");
                    lcd_status = ERROR;
                }

                before_error(STATE_STIRRER);
            }
    		else {
    			if((_curTick - _preOverloadTick) >= 15) {
    				_preOverloadTick = _curTick;

    				if(stirrer_status == COUNTERTURN) {
    					set_stirrer(TURN);
    					_turnError++;
    				}
    				else {
    					set_stirrer(COUNTERTURN);
    					_turnError++;
    				}
    			}
    		}
        }
        else {
    		_turnError = 0;
            set_stirrer(TURN);

            if(_w1 <= _targetW) {
                if(lcd_status != DISCHARGE) {
                    lcd.setCursor(0,0);
                    lcd.print("                    ");
                    lcd.setCursor(0,0);
                    lcd.print("DISCHARGE <-- MICRO");
                    lcd_status = DISCHARGE;
                }
                before_discharge();
                return;
            }
        }
    }
}


/**
* @brief DISCHARGE 단계로 진입하기 전 작업 구현
* @param 없음
* @return 없음
*/
void TasDryer::before_discharge() {
    stirrer_status = HIGH_TURN;
    set_stirrer(STOP);
    set_output_door(DOOR_CLOSE);
    off_all_power_supply();

    on_buzzer(TIME_BUZZER);

    _dryerState = STATE_DISCHARGE;
}

/**
* @brief Dryer 문열림 (DOOR) 단계 구현
* @param 없음
* @return 없음
*/
void TasDryer::discharge() {
    if(_dryerEvent & EVENT_DRYER_EMERGENCY_BTN_DOWN) {
        _dryerEvent &= ~EVENT_DRYER_EMERGENCY_BTN_DOWN;

        lcd.setCursor(0,0);
        lcd.print("                    ");
        lcd.setCursor(0,0);
        lcd.print("EMERGENCY <-- MICRO");
        lcd_status = EMERGENCY;
        before_emergency();
        return;
    }

    else if(_dryerEvent & EVENT_DRYER_DIS_BTN_DOWN) {
        _dryerEvent &= ~EVENT_DRYER_DIS_BTN_DOWN;
        set_output_door(DOOR_CLOSE);
    }

    else if(_dryerEvent & EVENT_DRYER_DIS_BTN_UP) {
        _dryerEvent &= ~EVENT_DRYER_DIS_BTN_UP;
        set_output_door(DOOR_OPEN);
    }

    else if(_dryerEvent & EVENT_DRYER_OUTPUT_DOOR_CLOSE) {
        _dryerEvent &= ~EVENT_DRYER_OUTPUT_DOOR_CLOSE;

        if(lcd_status != CLOSE_OUTPUT_DOOR) {
            lcd.setCursor(0,0);
            lcd.print("                    ");
            lcd.setCursor(0,0);
            lcd.print("OPEN OUTPUT DOOR");
            lcd_status = CLOSE_OUTPUT_DOOR;
        }

        on_buzzer(TIME_BUZZER);
    }

    else if(_dryerEvent & EVENT_DRYER_OUTPUT_DOOR_OPEN) {
        _dryerEvent &= ~EVENT_DRYER_OUTPUT_DOOR_OPEN;

        if(lcd_status != DISCHARGING) {
            lcd.setCursor(0,0);
            lcd.print("                    ");
            lcd.setCursor(0,0);
            lcd.print("DISCHARGING");
            lcd_status = DISCHARGING;
        }

        set_stirrer(HIGH_TURN);
    }

    if(_w1 < 0.5) {
        on_buzzer(TIME_BUZZER);

        if(lcd_status != END) {
            lcd.setCursor(0,0);
            lcd.print("                    ");
            lcd.setCursor(0,0);
            lcd.print("END <-- DISCHARGE");
            lcd_status = END;
        }

        before_end();
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

    _dryerState = STATE_END;
    endTimeout = 0;
}

/**
* @brief Dryer 종료 (END) 단계 구현
* @param 없음
* @return 없음
*/
void TasDryer::end() {
    if(_dryerEvent & EVENT_DRYER_EMERGENCY_BTN_DOWN) {
        _dryerEvent &= ~EVENT_DRYER_EMERGENCY_BTN_DOWN;

        lcd.setCursor(0,0);
        lcd.print("                    ");
        lcd.setCursor(0,0);
        lcd.print("EMERGENCY <-- END");
        lcd_status = EMERGENCY;
        before_emergency();
        return;
    }

    if(_dryerEvent & EVENT_DRYER_DIS_BTN_DOWN) {
        _dryerEvent &= ~EVENT_DRYER_DIS_BTN_DOWN;
        set_output_door(DOOR_CLOSE);
    }

    else if(_dryerEvent & EVENT_DRYER_DIS_BTN_UP) {
        _dryerEvent &= ~EVENT_DRYER_DIS_BTN_UP;
        set_output_door(DOOR_OPEN);
    }

    if (_curTick - _preTick >= _interval) {
        _preTick = _curTick;

        // timeout
        set_stirrer(STOP);
        off_deodorize();
        on_buzzer(TIME_BUZZER);
        endTimeout = 1;
    }

    if(endTimeout == 1) {
        if(_dryerEvent & EVENT_DRYER_OUTPUT_DOOR_CLOSE) {
            _dryerEvent &= ~EVENT_DRYER_OUTPUT_DOOR_CLOSE;

            if(lcd_status != CLOSE_OUTPUT_DOOR) {
                lcd.setCursor(0,0);
                lcd.print("                    ");
                lcd.setCursor(0,0);
                lcd.print("INPUT <-- END");
                lcd_status = CLOSE_OUTPUT_DOOR;
            }

            _thresholdW  = 100.0;
            _w0 = _w1;
            _w2 = 0.0;
            before_input();
            return;
        }

        if(_dryerEvent & EVENT_DRYER_OUTPUT_DOOR_OPEN) {
            _dryerEvent &= ~EVENT_DRYER_OUTPUT_DOOR_OPEN;

            if(lcd_status != OPEN_OUTPUT_DOOR) {
                lcd.setCursor(0,0);
                lcd.print("                    ");
                lcd.setCursor(0,0);
                lcd.print("CLOSE OUTPUT DOOR");
                lcd_status = OPEN_OUTPUT_DOOR;
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
    set_output_door(DOOR_CLOSE);
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
    if(_dryerEvent & EVENT_DRYER_EMERGENCY_BTN_DOWN) {
        _dryerEvent &= ~EVENT_DRYER_EMERGENCY_BTN_DOWN;

        lcd.setCursor(0,0);
        lcd.print("                    ");
        lcd.setCursor(0,0);
        lcd.print("EMERGENCY <-- ERROR");
        lcd_status = EMERGENCY;
        before_emergency();
        return;
    }
}

/**
* @brief EMERGENCY 단계로 진입하기 전 작업 구현
* @param 없음
* @return 없음
*/
void TasDryer::before_emergency() {
    set_stirrer(STOP);
    set_output_door(DOOR_CLOSE);
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
    if(_dryerEvent & EVENT_DRYER_EMERGENCY_BTN_UP) {
        _dryerEvent &= ~EVENT_DRYER_EMERGENCY_BTN_UP;

        lcd.setCursor(0,0);
        lcd.print("                    ");
        lcd.setCursor(0,0);
        lcd.print("INPUT <-- EMERGENCY");
        lcd_status = EMERGENCY;

        _thresholdW  = 100.0;
        _w0 = _w1;
        _w2 = 0.0;

        off_deodorize();
        before_input();
        return;
    }
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
void TasDryer::get_stirrer_current() {
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
        set_output_door(DOOR_CLOSE);
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
    _w1 = weight;
}

/**
* @brief 로드셀의 데이터를 읽는 함수
* @param 없음
* @return 로드셀 데이터:float_t
*/
float_t TasDryer::get_loadcell() {
    float_t weight = 3.14;

    weight = (scale.get_units(4)) * 0.453592; //scale.get_units() returns a float

//    weight -= 84.5; // calibration
//    weight *= 1.8; // calibration
//    weight -= 18.4; // calibration
    weight -= 1319.9;

    //if(weight <= 0.0) {
    //   weight = 0.0;
    //}

    scale_avg = ( scale_avg * (scale_count - 1) + weight ) / scale_count; // 평균 계산

    return round(scale_avg*10)/10.0;
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
    bool status = CLOSE;

    if (digitalRead(INPUT_DOOR_PIN) == LOW && _inDoorFlag == UP) {
        _inDoorOpenCount = 0;
        _inDoorFlag = DOWN;
    }
    else if (digitalRead(INPUT_DOOR_PIN) == LOW && _inDoorFlag == DOWN) {
        _inDoorOpenCount++;
        if (_inDoorOpenCount >= 4096) {
            _inDoorOpenCount = 4096;
            _dryerEvent &= ~EVENT_DRYER_INPUT_DOOR_CLOSE;
            _dryerEvent |= EVENT_DRYER_INPUT_DOOR_OPEN;
            _inDoorOpenCount = 0;
        }
    }
    else if (digitalRead(INPUT_DOOR_PIN) == HIGH && _inDoorFlag == DOWN) {
        _inDoorOpenCount = 0;
        _inDoorFlag = UP;
    }
    else if (digitalRead(INPUT_DOOR_PIN) == HIGH && _inDoorFlag == UP) {
        _inDoorOpenCount++;
        if (_inDoorOpenCount >= 4096) {
            _inDoorOpenCount = 4096;
            _dryerEvent &= ~EVENT_DRYER_INPUT_DOOR_OPEN;
            _dryerEvent |= EVENT_DRYER_INPUT_DOOR_CLOSE;
            _inDoorOpenCount = 0;
        }
    }

    return status;
}

/**
* @brief 배출문 상태 읽기
* @param 없음
* @return 열림|닫힘:boolean
*/
bool TasDryer::get_output_door() {
    bool status = CLOSE;

    if (digitalRead(OUTPUT_DOOR_PIN) == LOW && _outDoorFlag == UP) {
        _outDoorOpenCount = 0;
        _outDoorFlag = DOWN;
    }
    else if (digitalRead(OUTPUT_DOOR_PIN) == LOW && _outDoorFlag == DOWN) {
        _outDoorOpenCount++;
        if (_outDoorOpenCount >= 4096) {
            _outDoorOpenCount = 4096;
            _dryerEvent &= ~EVENT_DRYER_OUTPUT_DOOR_CLOSE;
            _dryerEvent |= EVENT_DRYER_OUTPUT_DOOR_OPEN;
            _outDoorOpenCount = 0;
        }
    }
    else if (digitalRead(OUTPUT_DOOR_PIN) == HIGH && _outDoorFlag == DOWN) {
        _outDoorOpenCount = 0;
        _outDoorFlag = UP;
    }
    else if (digitalRead(OUTPUT_DOOR_PIN) == HIGH && _outDoorFlag == UP) {
        _outDoorOpenCount++;
        if (_outDoorOpenCount >= 4096) {
            _outDoorOpenCount = 4096;
            _dryerEvent &= ~EVENT_DRYER_OUTPUT_DOOR_OPEN;
            _dryerEvent |= EVENT_DRYER_OUTPUT_DOOR_CLOSE;
            _outDoorOpenCount = 0;
        }
    }

    return status;
}


/**
* @brief 장치 스타트 버튼 이벤트 처리 함수
* @param 없음
* @return 눌림|안눌림:boolean
*/
bool TasDryer::get_start_button() {
    bool status = UP;

    if (digitalRead(START_BTN_PIN) == LOW && _startBtnDownFlag == UP) {
        _startBtnDownCount = 0;
        _startBtnDownFlag = DOWN;
    }
    else if (digitalRead(START_BTN_PIN) == LOW && _startBtnDownFlag == DOWN) {
        _startBtnDownCount++;
        if (_startBtnDownCount >= 4096) {
            _startBtnDownCount = 4096;
            _dryerEvent &= ~EVENT_DRYER_START_BTN_UP;
            _dryerEvent |= EVENT_DRYER_START_BTN_DOWN;
            _startBtnDownCount = 0;
        }
    }
    else if (digitalRead(START_BTN_PIN) == HIGH && _startBtnDownFlag == DOWN) {
        _startBtnDownCount = 0;
        _startBtnDownFlag = UP;
    }
    else if (digitalRead(START_BTN_PIN) == HIGH && _startBtnDownFlag == UP) {
        _startBtnDownCount++;
        if (_startBtnDownCount >= 4096) {
            _startBtnDownCount = 4096;
            _dryerEvent &= ~EVENT_DRYER_START_BTN_DOWN;
            _dryerEvent |= EVENT_DRYER_START_BTN_UP;
            _startBtnDownCount = 0;
        }
    }

    return status;
}

/**
* @brief 장치 배출 버튼 이벤트 처리 함수
* @param 없음
* @return 눌림|안눌림:boolean
*/
bool TasDryer::get_discharge_button() {
   bool status = UP;

   if (digitalRead(DISCHARGE_BTN_PIN) == LOW && _disBtnDownFlag == UP) {
       _disBtnDownCount = 0;
       _disBtnDownFlag = DOWN;
   }
   else if (digitalRead(DISCHARGE_BTN_PIN) == LOW && _disBtnDownFlag == DOWN) {
       _disBtnDownCount++;
       if (_disBtnDownCount >= 4096) {
           _disBtnDownCount = 4096;
           _dryerEvent &= ~EVENT_DRYER_DIS_BTN_UP;
           _dryerEvent |= EVENT_DRYER_DIS_BTN_DOWN;
           _disBtnDownCount = 0;
       }
   }
   else if (digitalRead(DISCHARGE_BTN_PIN) == HIGH && _disBtnDownFlag == DOWN) {
       _disBtnDownCount = 0;
       _disBtnDownFlag = UP;
   }
   else if (digitalRead(DISCHARGE_BTN_PIN) == HIGH && _disBtnDownFlag == UP) {
       _disBtnDownCount++;
       if (_disBtnDownCount >= 4096) {
           _disBtnDownCount = 4096;
           _dryerEvent &= ~EVENT_DRYER_DIS_BTN_DOWN;
           _dryerEvent |= EVENT_DRYER_DIS_BTN_UP;
           _disBtnDownCount = 0;
       }
   }

   return status;
}

/**
* @brief EMERGENCY 버튼 이벤트 처리 함수
* @param 없음
* @return 눌림|안눌림:boolean
*/
uint8_t TasDryer::get_emergency_button() {
    uint8_t status = UP;

    if (digitalRead(EMERGENCY_BTN_PIN) == LOW && _emerBtnDownFlag == UP) {
        _emerBtnDownCount = 0;
        _emerBtnDownFlag = DOWN;
    }
    else if (digitalRead(EMERGENCY_BTN_PIN) == LOW && _emerBtnDownFlag == DOWN) {
        _emerBtnDownCount++;
        if (_emerBtnDownCount >= 4096) {
            _emerBtnDownCount = 4096;
            _dryerEvent &= ~EVENT_DRYER_EMERGENCY_BTN_UP;
            _dryerEvent |= EVENT_DRYER_EMERGENCY_BTN_DOWN;
            _emerBtnDownCount = 0;
        }
    }
    else if (digitalRead(EMERGENCY_BTN_PIN) == HIGH && _emerBtnDownFlag == DOWN) {
        _emerBtnDownCount = 0;
        _emerBtnDownFlag = UP;
    }
    else if (digitalRead(EMERGENCY_BTN_PIN) == HIGH && _emerBtnDownFlag == UP) {
        _emerBtnDownCount++;
        if (_emerBtnDownCount >= 4096) {
            _emerBtnDownCount = 4096;
            _dryerEvent &= ~EVENT_DRYER_EMERGENCY_BTN_DOWN;
            _dryerEvent |= EVENT_DRYER_EMERGENCY_BTN_UP;
            _emerBtnDownCount = 0;
        }
    }

    return status;
}

/**
* @brief 교반기 상태 확인하는 함수
* @param 없음
* @return 과부하|정상:boolean
*/
bool TasDryer::get_stirrer() {
    bool status = NORMAL;

    // todo: overload 상태의 전류 확인
    if(_stirrerCurrent > 15) {
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
void TasDryer::set_output_door(uint8_t command) {
    if(command == DOOR_CLOSE) {
        if(output_door_status != DOOR_CLOSE) {
            output_door_status = DOOR_CLOSE;
            // _microCtrl &= ~(0x01 << 0);
            // _microCtrl &= ~(0x01 << 1);

            _microCtrl |= (0x01 << 0);
            _microCtrl |= (0x01 << 1);

            //_microCtrl &= ~(0x01 << 0);
            //_microCtrl &= ~(0x01 << 1);

            updateShiftRegister();
            _disFlag = 0;
        }
    }
    else if(command == DOOR_OPEN) {
        if(output_door_status != DOOR_OPEN) {
            output_door_status = DOOR_OPEN;
            // _microCtrl |= (0x01 << 0);
            // _microCtrl |= (0x01 << 1);

            _microCtrl &= ~(0x01 << 0);
            _microCtrl &= ~(0x01 << 1);

            //_microCtrl |= (0x01 << 0);
            //_microCtrl |= (0x01 << 1);

            updateShiftRegister();
            _disFlag = 0;
        }
    }
    // else {
    //     if(output_door_status != DOOR_STOP) {
    //         output_door_status = DOOR_OPEN;
    //         _microCtrl &= ~(0x01 << 0);
    //         _microCtrl |= (0x01 << 1);
    //
    //         updateShiftRegister();
    //     }
    // }
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
* @brief (임시) 배출 도어 4초후 멈춤
* @param 없음
* @return 없음
*/
bool TasDryer::chk_discharge_door() {
    if(_disFlag == 1) {
        if (_curTick - _disTick >= _disInterval) {
            _disTick = _curTick;

            // discharge door timeout
            set_output_door(DOOR_STOP);
            _disFlag = 0;
            return true;
        }
    }
    return false;
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
    digitalWrite(DEODORIZE_PIN, HIGH);
}


/**
* @brief 탈취장치 전원 오프
* @param 없음
* @return 없음
*/
void TasDryer::off_deodorize() {
    digitalWrite(DEODORIZE_PIN, LOW);
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
* @brief 해당 마그네트론의 파워서플라이 온, 오프 상태 읽기
* @param 마그네트론 번호 : uint8_t
* @return ON|OFF:bool
*/
bool TasDryer::get_power_supply(uint8_t num) {
    uint8_t status = OFF;

    status = _microPower & (0x01 << num);

    return (status >> num);
}

/**
* @brief 모든 마그네트론의 파워서플라이 오프
* @param 없음
* @return 없음
*/
void TasDryer::off_all_power_supply() {
    _microCtrl &= ~(0x01 << 4);
    _microCtrl &= ~(0x01 << 5);
    _microCtrl &= ~(0x01 << 6);
    _microCtrl &= ~(0x01 << 7);

    updateShiftRegister();
}

/**
* @brief 모든 마그네트론의 쿨러 오프
* @param 없음
* @return 없음
*/
void TasDryer::off_all_cooler() {
    _microCtrl &= ~(0x01 << 2);
    _microCtrl &= ~(0x01 << 3);

    updateShiftRegister();
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
    _microCtrl |= (0x01 << 2);
    _microCtrl |= (0x01 << 3);

    if(_microMode == MICRO_MODE1) {
        if(_microTick == 0) {
            _microCtrl |= (0x01 << (((_microIdx+0)%4)+4));
            _microCtrl |= (0x01 << (((_microIdx+1)%4)+4));
            _microCtrl |= (0x01 << (((_microIdx+2)%4)+4));
            _microCtrl |= (0x01 << (((_microIdx+3)%4)+4));

            updateShiftRegister();
        }
        else if( _microTick >= TIME_MICRO_FIRST) {
            _microTick = 0;
            _microIdx = 0;
            _microMode = MICRO_MODE2;

            _microCtrl &= ~(0x01 << (((_microIdx+0)%4)+4));
            _microCtrl |= (0x01 << (((_microIdx+1)%4)+4));
            _microCtrl |= (0x01 << (((_microIdx+2)%4)+4));
            _microCtrl |= (0x01 << (((_microIdx+3)%4)+4));

            updateShiftRegister();
        }
    }
    else if(_microMode == MICRO_MODE2) {
        _microCtrl &= ~(0x01 << (((_microIdx+0)%4)+4));
        _microCtrl |= (0x01 << (((_microIdx+1)%4)+4));
        _microCtrl |= (0x01 << (((_microIdx+2)%4)+4));
        _microCtrl |= (0x01 << (((_microIdx+3)%4)+4));

        updateShiftRegister();
    }
}

/**
* @brief 교반기 내의 온도값(pt100 온도 센서) 읽기
* @param 없음
* @return 온도:float_t
*/
int TasDryer::get_pt100() {
    int temperature = PT100.readTemperature(PT100_PIN);  //Get temperature

    temperature_avg = ( temperature_avg * (8 - 1) + temperature ) / 8; // 평균 계산

    return temperature_avg;
}

/**
* @brief 마그네트론의 상태를 체크하기 위해 전체 마그네트론(파워서플라이) 전류의 합계
* @param 없음
* @return 전류값:float_t
*/
float_t TasDryer::get_current_micro() {
    float_t current = 10;

    return current;
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
void TasDryer::lcd_input_log() {
    // @todo: LCD 출력 :  3열, w0 값 출력
    lcd.setCursor(0,0);
    lcd.print("                    ");
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
