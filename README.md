# nCube:Thyme for Arduino
## Version
2.1.0 (20180408)

## Introduction
nCube:Thyme for Arduino is a light version nCube for arduino devices. </br>
The project name: "nCube-Base". nCube-Base is a type of AE(oneM2M Resource Type) in software level.

## Hardware
- Arduino Zero + WiFi Module or [Adafruit Feather M0 WiFi Board](https://www.adafruit.com/product/3061)

<div align="left">
<img src="https://user-images.githubusercontent.com/29790334/28243934-48f4f748-6a16-11e7-9c2c-42c0f320ca76.png" width="400">
</div><br/>

## Connectivity
The sample source introduces how to use Adafruit Feather M0 WiFi Board to connect CO2 sensor, RGB LED light connect to Mobius server platform.

<img src="https://user-images.githubusercontent.com/29790334/36194128-e59c054a-11ab-11e8-9723-8c0a0aed8256.png" width="600">


- [CM1106 CO2 Sensor](http://www.gassensor.com.cn/pro/typeid/12/id/294.html)
- [RGB LED Module](http://alexnld.com/product/rgb-3-color-led-module-for-arduino-red-green-blue/)

## Installation

- Download [Arduino IDE](https://www.arduino.cc/en/Main/Software).
- Install Arduino IDE.
- Download [nCube-Thyme-Arduino](https://github.com/IoTKETI/nCube-Thyme-Arduino/archive/master.zip).
- Copy the libraries file to Arduino IDE libraries folder.</br>
We provide nCube-Mint source as Arduino libraries and you need to copy or overwrite it to the Arduino IDE library home "c:\Users\[user name]\Document\Arduino\libraries".

## Configuration

Adafruit Feather M0 is not an Arduino official device. Therefore, some configurations for the Arduino IDE to work well with Adafruit Feather M0 device were made. Configuration details are available at [Adafruit Feather M0 offical site](https://learn.adafruit.com/adafruit-feather-m0-wifi-atwinc1500/setup) or [nCube-mint guide document](https://github.com/IoTKETI/nCube-Thyme-Arduino/raw/master/doc/nCube-Mint_v1.0.docx).

## Running

- Connect the debug cable to PC<br/>
![image](https://user-images.githubusercontent.com/29790334/28244086-e1b366c2-6a1b-11e7-9188-76a5a860c30e.png)
- Open "File>Examples>oneM2MClient>nCube-Mint"<br/>

<div align="left">
<img src="https://user-images.githubusercontent.com/29790334/28244187-eff4f104-6a1d-11e7-93b6-1997d4054ab9.png" width="600">
</div>

- Open "Tools>Serial Monitor" for view the log.<br/>

<div align="left">
<img src="https://user-images.githubusercontent.com/29790334/28244217-f855df1a-6a1e-11e7-86ae-d09b2d438086.png" width="480">
</div><br/>

- Modifications of the source can be made if needed.
- Click "upload" button at top of the Arduino IDE.
- Connect PC WiFi to SSID "wifi101-3B3E".
- Open the link "http://wifi101.local".<br/>

<div align="left">
<img src="https://user-images.githubusercontent.com/29790334/28244236-aaba4556-6a1f-11e7-92db-ee807222bf45.png" width="400">
</div>

- Input WiFi connection information on the page and click connection button.<br/>

<div align="left">
<img src="https://user-images.githubusercontent.com/29790334/28244247-fb989342-6a1f-11e7-8e89-6e33ad868291.png" width="600">
</div><br/>

<div align="left">
<img src="https://user-images.githubusercontent.com/29790334/28244248-0a759f90-6a20-11e7-8d7c-62e6a0e80fe6.png" width="600">
</div><br/>



# Over The Air(OTA)
OTA (Over The Air) means programmers can code on local and send it to the device to upgrade or replace with new firmware through the cloud. In Arduino, some developer finds a new way to update the Arduino sketch online. It enables developers to install wireless Arduino devices in a place where there is a difficulty of accessibility can be updated without uninstallation.

![image](https://user-images.githubusercontent.com/29790334/28252674-b3accaae-6ad2-11e7-95bc-0a142a8a4633.png)

## nCube-Mint-OTA

&Cube-Mint is a software for the light weight WiFi devices(e.g. Arduino Yun or Arduino UNO) with WiFi shield and direct internet access without gateway. It is a light weight C++ program that make connections between Adafruit Feather M0 WiFi board and Mobius IoT Server Platform (see more in IoT-OCEAN official website). As mentioned above, we also provide a software named &Cube-Mint OTA to support a remote software upgrade with the internet connection in a situation where there is a difficulty of accessibility.

OTA(Over The Air) stands for the remote software upgrade that can code local and send it to the device to upgrade or replace with new firmware through the cloud. In Arduino, some developer finds a new way to update the Arduino sketch online. It enables developers to install wireless Arduino devices in a place where there is a difficulty of accessibility.

![image](https://user-images.githubusercontent.com/29790334/28252678-cb023dba-6ad2-11e7-840a-825906fd25e0.png)

## Example

1. First firmware
- Open "File>Examples>oneM2MClient>nCube-Mint-OTA".
- Check	“FIRMWARE_VERSION”, "AE_ID" and "buildResource()".
- Upload sketch to the board.

2. Update firmware
- Check	“FIRMWARE_VERSION”, "AE_ID"(raise the version number).
- Compile new ".Hex" file.
```
C:\Users\[user]\AppData\Local\Temp\arduino_build_********
```
- Open OTA Server website "http://203.253.128.161:8730"
![image](https://user-images.githubusercontent.com/29790334/28252742-16d533b8-6ad4-11e7-819a-b60ed5240421.png)
- Upload ".Hex" file.
- Send update command with HTTP request
```
POST /Mobius/[your-ae]/update
Host: 203.253.128.161:7579
Accept: application/json
X-M2M-RI: 12345
X-M2M-Origin: [your-ae]
Content-Type: application/vnd.onem2m-res+json; ty=4

{"m2m:cin":{"con":"active"}}
```
## OTA Result

![1](https://user-images.githubusercontent.com/29790334/28252773-fd39e984-6ad4-11e7-9cbb-8fd721186da6.png)

## Dependency Libraries

- ArduinoJson
- FlashStorage
- PubSubClient
- WiFi101
- WiFi101OTA
- TasCO2: CM1106 CO2 Sensor library(Made by KETI)
- OneM2MClient: Mobius client library(Made by KETI)
- OverTheAir: Device Online upgrade module(Made by KETI)

## Document

For more information, please refer to the installation guide document as below.

- nCube-Mint [installation guide](https://github.com/IoTKETI/nCube-Thyme-Arduino/raw/master/doc/(English)%20nCube-Arduino_Developer%20Guide_English.pdf).
- nCube-Mint-OTA [installation guide](https://github.com/IoTKETI/nCube-Thyme-Arduino/raw/master/doc/nCube-Mint-OTA_v1.0.pdf).

# Author
Il Yeup Ahn (iyahn@keti.re.kr; ryeubi@gmail.com)
