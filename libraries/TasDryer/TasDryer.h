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
    TasDryer.h - monitoring and control for Dryer
    Created by Ilyeup Ahn in KETI Korea, Sep. 19, 2017.
*/

#ifndef TASDRYER_H
#define TASDRYER_H

#include "Arduino.h"

// define Dryer state when running
#define STATE_INIT      0x01
#define STATE_ERROR     0x02
#define STATE_EMERGENCY 0x03
#define STATE_INPUT     0x04
#define STATE_DOOR      0x08
#define STATE_STIRRER   0x10
#define STATE_MICRO     0x20
#define STATE_DISCHARGE 0x40
#define STATE_END       0x80
#define STATE_DEBUG     0x11

#define EVENT_DRYER_INIT                    0x0001
#define EVENT_DRYER_ERROR                   0x0002
#define EVENT_DRYER_OUTPUT_DOOR_OPEN        0x0004
#define EVENT_DRYER_OUTPUT_DOOR_CLOSE       0x0008
#define EVENT_DRYER_INPUT_DOOR_OPEN         0x0010
#define EVENT_DRYER_INPUT_DOOR_CLOSE        0x0020
#define EVENT_DRYER_STIRRER_CURRENT         0x0040
#define EVENT_STATUS_BTN_CLICK              0x0080
#define EVENT_LOADCELL_BTN_UP_DOWN_PRESSED  0x0100
#define EVENT_DRYER_STIRRER_TICK            0x0200
#define EVENT_LOADCELL_BTN_UP_PRESSED       0x0400
#define EVENT_LOADCELL_BTN_UP_CLICK         0x0800
#define EVENT_LOADCELL_BTN_DOWN_PRESSED     0x1000
#define EVENT_LOADCELL_BTN_DOWN_CLICK       0x2000
#define EVENT_DRYER_START_BTN_UP            0x4000
#define EVENT_DRYER_START_BTN_DOWN          0x8000


#define STATE_INPUT     0x04
#define STATE_DOOR      0x08
#define STATE_STIRRER   0x10
#define STATE_MICRO     0x20
#define STATE_DISCHARGE 0x40
#define STATE_END       0x80

/**
* @brief Dryer(음식물건조기)를 구동하기 위한 클래스이다.
* @author Ilyeup Ahn
* @date 2017-09-06
* @version 0.0.1
*/
class TasDryer
{
  public:
    TasDryer();
    void begin();

    String getDryerStatus();

    void debug();
    void before_debug();
    void init();
    void before_error(uint8_t code);
    void error();
    void before_input();
    void input();
    void before_stirrer();
    void stirrer();
    void before_door();
    void door();
    void before_micro();
    void micro();
    void before_discharge();
    void discharge();
    void before_end();
    void end();
    void before_emergency();
    void emergency();

    uint8_t get_wifi_select();

    void loop();

    void print_info_lcd();

  private:
    float_t _dryRate;  // 건조도

    int8_t _temperature;

    uint16_t _dryerEvent;
    uint8_t _dryerBtnEvent;

    uint8_t _dryerState;
    float_t _w0;       // 투입중량
    float_t _w1;       // 현재중량
    float_t _w2;       // 추가투입중량
    float_t _maxW;     // 최대중량
    float_t _targetW;  // 목표중량

    float_t _thresholdW;    // 투입가능 기준 중량
    float_t _preW2;         // 문 열 때 중량
    float_t _curW2;         // 문 닫았을 때 중량

    uint32_t _startBtnDownCount;
    uint8_t _startBtnDownFlag;

    uint32_t _loadUpCount;
    uint8_t _loadUpFlag;

    uint32_t _statusCount;
    uint8_t _statusFlag;

    uint32_t _loadDownCount;
    uint8_t _loadDownFlag;

    uint32_t _inDoorOpenCount;
    uint8_t _inDoorFlag;
    uint32_t _outDoorOpenCount;
    uint8_t _outDoorFlag;

    uint32_t _debugSelCount;
    uint8_t _debugSelFlag;
    uint8_t _debugSelStatus;

    uint32_t _loadcell_calibration_lcd_previousMillis;
    uint32_t _loadcell_calibration_lcd_interval;

    uint8_t _turnError;
    uint8_t _errorCode;

    uint8_t _microIdx;
    uint8_t _microMode;
    uint8_t _microTick;
    uint8_t _microStatus;

    uint32_t _curTick;
    uint32_t _preOverloadTick;

    uint32_t _preTick;
    uint32_t _interval;

    uint32_t _buzzerTick;
    uint32_t _buzzerInterval;
    uint8_t _buzzerFlag;

    uint32_t _coolerTick;
    uint32_t _coolerInterval;
    uint8_t _coolerFlag;

    uint32_t _disTick;
    uint32_t _disInterval;
    uint8_t _disFlag;

    uint16_t _debugStatus;

    uint32_t _stirrerCurrent;

    uint8_t _preDryerState;
    float_t _preW1;
    float_t _preW0;
    float_t _preTargetW;
    uint8_t _preMicroMode;
    uint8_t _preMicroCtrl;
    uint8_t _preMicroPower;
    uint8_t _preMicroIdx;
    uint32_t _preCurTick;
    int8_t _preTemperature;
    uint8_t _preCoolerFlag;
    uint32_t _preStirrerCurrent;
    uint8_t endTimeout;
    uint8_t _preStirrerStatus;

    uint8_t stirrer_status;
    uint8_t lcd_status;
    uint8_t output_door_status;

    float_t _microACCurrtntValue1;
    float_t _microACCurrtntValue2;
    float_t _microACCurrtntValue3;
    float_t _microACCurrtntValue4;

    uint32_t _elapsed_micro_tick;
    uint8_t _elapsed_micro_h;
    uint8_t _elapsed_micro_m;
    uint8_t _elapsed_micro_s;
    uint32_t _preElapsedMicroTick;

    uint8_t get_debug_select();
    float_t get_loadcell();
    float_t get_loadcell_4tare();
    float_t get_dry_rate();
    bool get_input_door();
    bool get_output_door();
    bool get_start_button();
    bool get_discharge_button();
    uint8_t get_emergency_button();
    bool get_stirrer();
    int get_pt100();
    void get_current_micro();
    bool get_power_supply(uint8_t num);
    bool chk_micro_cooler();
    void set_cooler_timeout(uint32_t interval);
    void request_stirrer_current();
    void get_stirrer_current();
    void set_stirrer_stop();
    void set_stirrer_forward();
    void set_stirrer_backward();
    void set_stirrer_high_forward();

    void get_loadcell_button();
    void get_status_button();
    void set_timeout(uint32_t interval);
    void on_buzzer(uint32_t interval);
    void chk_buzzer();
    void off_buzzer();
    void on_deodorize();
    void off_deodorize();
    void set_stirrer(uint8_t ctrl);
    void off_all_power_supply();
    void off_all_cooler();
    void on_all_cooler();
    bool off_micro_cooler();
    void set_micro();
    void set_micro_mode1();
    void set_micro_mode2();
    void set_micro_mode3();
    void set_micro_mode_off();
    void set_micro_before();
    void chk_pt100();
    void chk_loadcell();

    void lcd_init_log();
    void lcd_input_log();
    void lcd_stirrer_log();
    void lcd_door_log();
    void lcd_error_log();
    void lcd_dis_log();
    void lcd_discharging_log();
    void lcd_end_log();
    void lcd_emergency_log();
    void lcd_output_door_log();
    void lcd_input_door_log();
    void lcd_low_load_log();
    void lcd_overload_log();
    void lcd_door2door_log();
    void lcd_micro_mode_log();
    void lcd_dis_open_door_log();
    void lcd_dis_close_door_log();
    void lcd_end_close_out_door_log();
    void lcd_end_close_in_door_log();
    void lcd_debug_log();
    void lcd_pause_log();
    void lcd_over_weight_log();

};

#endif // TASDRYER_H
