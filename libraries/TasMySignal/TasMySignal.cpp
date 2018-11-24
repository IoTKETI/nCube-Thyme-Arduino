/*
    TasMySignal.cpp - Library for MySignal HW v2
    Created by Chanhyung Lee in KETI Korea, November 24, 2018.
    Released into the public domain.
*/

#include "TasMySignal.h"

Adafruit_LiquidCrystal lcd(0);
#define LCD_GND 5

void TasMySignal::begin() {
    Serial1.begin(9600);
    delay(300);

    pinMode(temperaturePin, INPUT_PULLUP);
    pinMode(spirometerPin, INPUT_PULLUP);
    pinMode(bloodpressurePin, INPUT_PULLUP);
    pinMode(spo2Pin, INPUT_PULLUP);
    pinMode(glucometerPin, INPUT_PULLUP);

    TasMySignal::LCD_hello();

    _btnDownFlag = UP;
    _btnDownCnt = 0;

    sensorData.status = disabled;
}


void TasMySignal::loop() {
    TasMySignal::sensingRequester();
    TasMySignal::dataReciever();
    TasMySignal::dataLoader();
    TasMySignal::getButton();
}

void TasMySignal::sensorEnabler(uint8_t _sensor) {

    sensorData.type = _sensor;
    sensorData.status = enabled;
    sensorData.data = "";
    sensorData.failCounter = 0;
}


void TasMySignal::sensingRequester() {
    if (sensorData.status == enabled) {
        TasMySignal::LCD_clear();
        TasMySignal::LCD_sensor(sensorData.type);
        TasMySignal::LCD_status(enabled);
        if (sensorData.failCounter < max_failcnt) {
            Serial1.write(0x7E);
            Serial1.write(sensorData.type);
            Serial1.write(0x7F);
            sensorData.status = requested;
            TasMySignal::LCD_status(requested);
        }
        else {
            sensorData.status = failed;
            TasMySignal::LCD_status(failed);
        }
    }
}


void TasMySignal::dataReciever() {
    if (Serial1.available() > 0) {
        char c = Serial1.read();
        if (_dataFlag == 1) {
            _dataFrame[_dataFrame_idx++] = c;
        }
        if (_dataFlag == 0 && c == 0x7E) {
            _dataFlag = 1;
        }
        if (_dataFlag == 1 && c == 0x7F) {
            _dataFlag = 2;
        }
        if (_dataFlag == 1 && c == 0x7C) {
            TasMySignal::LCD_status(connected);
            _dataFrame_idx = 0;
        }
        if (_dataFlag == 1 && c == 0x7D) {
            TasMySignal::LCD_status(sensing);
            _dataFrame_idx = 0;
        }
    }
    else {
    }
}

void TasMySignal::dataLoader() {
    if (_dataFlag == 2 && sensorData.status == requested) {
        for (int j = 0; j < (_dataFrame_idx - 1); j++) {
            sensorData.data += _dataFrame[j];
        }

        if (sensorData.data == "f") {
            sensorData.data = "";
            sensorData.failCounter++;
            sensorData.status = enabled;
            Serial.println(sensorData.failCounter);
        }
        else {
            sensorData.status = recieved;
            TasMySignal::LCD_status(recieved);
            TasMySignal::LCD_data(sensorData.type, sensorData.data);
        }

        _dataFrame_idx = 0;
        _dataFlag = 0;
    }
}

void TasMySignal::getButton() {
    if (sensorData.status == disabled) {
        uint8_t _tempBtn = digitalRead(temperaturePin);
        uint8_t _spiroBtn = digitalRead(spirometerPin);
        uint8_t _bpBtn = digitalRead(bloodpressurePin);
        uint8_t _spo2Btn = digitalRead(spo2Pin);
        uint8_t _glucoBtn = digitalRead(glucometerPin);

        if (_tempBtn == LOW || _spiroBtn == LOW || _bpBtn == LOW || _spo2Btn == LOW || _glucoBtn == LOW) {
            if (_btnDownFlag == UP) {
                _btnDownCnt++;
                if (_btnDownCnt > 1024) {
                    if (_tempBtn == LOW) {
                        _btnType = temperature;
                    }
                    else if (_spiroBtn == LOW) {
                        _btnType = spirometer;
                    }
                    else if (_bpBtn == LOW) {
                        _btnType = bloodpressure;
                    }
                    else if (_spo2Btn == LOW) {
                        _btnType = spo2;
                    }
                    else if (_glucoBtn == LOW) {
                        _btnType = glucometer;
                    }
                    _btnDownCnt = 0;
                    _btnDownFlag = DOWN;
                }
            }
            else if (_btnDownFlag == DOWN) {
                // TasMySignal::sensorEnabler(_btnStatus);
                // TasMySignal::LCD_clear();
                // TasMySignal::LCD_sensor(_btnStatus);
                // TasMySignal::LCD_status(enabled);
                _btnDownFlag = DOWNCON;
                _btnDownCnt = 0;
            }
        }
        else {
            _btnDownFlag = UP;
        }
    }
}


void TasMySignal::LCD_hello() {
    lcd.begin(20, 4);
    lcd.setBacklight(HIGH);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SmartAging-MySignals");
}

void TasMySignal::LCD_sensor(uint8_t sensor) {
    lcd.setCursor(0, 1);
    lcd.print("                    ");
    lcd.setCursor(0, 1);
    lcd.print("Sensor:");
    if (sensor == temperature) {
        lcd.print("temperature");
    }
    else if (sensor == spirometer) {
        lcd.print("spirometer");
    }
    else if (sensor == bloodpressure) {
        lcd.print("bloodpressure");
    }
    else if (sensor == spo2) {
        lcd.print("spo2");
    }
    else if (sensor == glucometer) {
        lcd.print("glucometer");
    }
}

uint8_t _preStatus = disabled;
void TasMySignal::LCD_status(uint8_t status) {
    if (_preStatus != status) {
        lcd.setCursor(0, 2);
        lcd.print("                    ");
        lcd.setCursor(0, 2);
        lcd.print("-> ");
        if (status == enabled) {
            lcd.print("Sensor enabled");
        }
        else if (status == requested) {
            lcd.print("Wait for Sensor");
        }
        else if (status == connected) {
            lcd.print("Sensor Connected");
        }
        else if (status == sensing){
            lcd.print("Sensing...");
        }
        else if (status == recieved) {
            lcd.print("Sensing Success");
        }
        else if (status == failed) {
            lcd.print("Sensing Failed");
        }

        _preStatus = status;
    }
}

void TasMySignal::LCD_data(uint8_t sensor, String data) {
    lcd.setCursor(0, 3);
    lcd.print("                    ");
    lcd.setCursor(0, 3);
    if (sensor == temperature) {
        lcd.print("Temperature:");
        lcd.print(data);
    }
    else if (sensor == spirometer) {
        lcd.print("L/min, L:");
        lcd.print(data);
    }
    else if (sensor == bloodpressure) {
        lcd.print("M,m,pulse:");
        lcd.print(data);
    }
    else if (sensor == spo2) {
        lcd.print("SpO2,pulse:");
        lcd.print(data);
    }
    else if (sensor == glucometer) {
        lcd.print("mg/dl:");
        lcd.print(data);
    }
}

void TasMySignal::LCD_clear() {
    lcd.setCursor(0, 1);
    lcd.print("                    ");
    lcd.setCursor(0, 2);
    lcd.print("                    ");
    lcd.setCursor(0, 3);
    lcd.print("                    ");
}