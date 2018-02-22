extra_script.py is provided for OTA in environments using PlatformIO. 

In platformio.ini file under your Arduino project directory,
append script below

extra_scripts = extra_script.py

And then, move the extra_script.py file to same directory with platformio.ini


After PlatformIO compile the Arduino source code under src directory,
firmware.hex file will be created under .pioenvs/adafruit_feather_m0 directory.

The firmware.hex file will be used for OTA update through nCube:Thyme for Arduino OTA Web Dashboard(http://203.253.128.161:8730).
