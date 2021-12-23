#!/bin/bash

git pull
./arduino-cli compile --fqbn per1234:sam:arduino_due_x_dbg --libraries /arduino/main/libraries/Arduino_PID_Library,/arduino/main/libraries/Rosserial_Arduino_Library arduino/main
./arduino-cli upload  --fqbn per1234:sam:arduino_due_x_dbg --port /dev/ttyACM0 arduino/main