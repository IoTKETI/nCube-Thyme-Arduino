/*
    TasAgingSignals.h - Library for MySignal HW v2
    Created by Chanhyung Lee in KETI Korea, November 24, 2018.
    Released into the public domain.
*/

#include <Arduino.h>
#include <Adafruit_LiquidCrystal.h>

#define temperature 0x31
#define spirometer 0x32
#define bloodpressure 0x33
#define spo2 0x34
#define glucometer 0x35

#define temperaturePin 6
#define spirometerPin 9
#define bloodpressurePin 10
#define spo2Pin 11
#define glucometerPin 12

#define disabled 0x10
#define enabled 0x11
#define requested 0x12
#define connected 0x13
#define sensing   0x14
#define recieved 0x15
#define failed 0x16

#define UP 0x00
#define DOWN 0x01
#define DOWNCON 0x02

#define max_failcnt 3

typedef struct _sensorData_ {
    uint8_t type;
    String data;
    uint8_t status;
    uint8_t failCounter;
} sensorData_t;

class TasAgingSignals {
    public:
        void begin();
        void loop();
        void sensorEnabler(uint8_t _sensor);

        uint8_t _btnType;
        uint8_t _btnDownFlag;

        sensorData_t sensorData;

      private:
        void dataReciever();
        void dataLoader();
        void sensingRequester();
        void getButton();
        void LCD_hello();
        void LCD_sensor(uint8_t sensor);
        void LCD_status(uint8_t status);
        void LCD_data(uint8_t sensor, String data);
        void LCD_clear();

        uint16_t _btnDownCnt;

        char _dataFrame[128];
        uint8_t _dataFrame_idx = 0;
        int _dataFlag = 0;
};