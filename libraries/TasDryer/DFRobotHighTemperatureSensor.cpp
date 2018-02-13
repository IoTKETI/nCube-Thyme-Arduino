/*
  DFRobotHighTemperatureSensor.cpp - DFRobotHighTemperatureSensor library
  Developed by Bernie - 2016/1/13

  This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.

HighTemperature Sensor info found at http://www.dfrobot.com/

Version 1.0: 13 Jan 2016 by bernie
- Updated to build against Arduino 1.0.6
- Made accessors inline in the header so they can be optimized away
*/

#include "DFRobotHighTemperatureSensor.h"

DFRobotHighTemperature::DFRobotHighTemperature()  //Constructor,Default reference voltage 5.000V
{
	_voltageRef = 5.000;
}

DFRobotHighTemperature::DFRobotHighTemperature(float v)  //Constructor
{
	_voltageRef = v;
}

DFRobotHighTemperature::~DFRobotHighTemperature()		//Destructor
{

}
int DFRobotHighTemperature::readTemperature(int PIN)     //Get temperature
{
    int sensorValue = analogRead(PIN);
    float voltage = 0,res = 0;
    //voltage = sensorValue * _voltageRef / 1024.0;  //If your microcontroller are a 12 bit ADC, you should be change 1024 to 4096
	voltage = sensorValue * _voltageRef / 1024.0;  //If your microcontroller are a 12 bit ADC, you should be change 1024 to 4096
	//If you have a DS18B20 on hand, you can make a more accurate calibration.
	//change 220.9 to 210.9 or 230.9,and you need change 2.209 to 2.109 or 2.309 at the same time to accurate calibration.
	res =  (1800 * voltage + 250.9 * 18) / (2.509 * 18 - voltage);
	//res =  (1800 * voltage + 235.9 * 18) / (2.359 * 18 - voltage);
	//searched by the halving method
    int front = 0, end = 0, mid = 0;
    front = 0;
    end = 399;
    mid = (front + end) / 2;
    while (front < end && pgm_read_float(&PT100Tab[mid]) != res) {
        if (pgm_read_float(&PT100Tab[mid]) < res) {
            if (pgm_read_float(&PT100Tab[mid + 1]) < res) {
                front = mid + 1;
            }
            else {
                mid = comp(res, mid);
                return mid;
            }
        }

        if (pgm_read_float(&PT100Tab[mid]) > res) {
            if (pgm_read_float(&PT100Tab[mid - 1]) > res) {
                end = mid - 1;
            }
            else {
                mid = comp(res, mid - 1);
                return mid;
            }
        }

        mid = front + (end - front) / 2;
    }
    return mid;
}
int DFRobotHighTemperature::comp(float pt, int i) //Which number is closer on the two adjacent numbers.
{
    if ((pt - pgm_read_float(&PT100Tab[i])) > (pgm_read_float(&PT100Tab[i + 1]) - pgm_read_float(&PT100Tab[i])) / 2 ) {
        return i + 1;
    }
    else {
        return i;
    }
}
