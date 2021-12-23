#!/bin/bash

git pull
./arduino-cli compile --fqbn arduino:avr:uno --libraries /arduino/main/libraries/Arduino_PID_Library,/arduino/main/libraries/Rosserial_Arduino_Library arduino/main
./arduino-cli upload  --fqbn arduino:avr:uno --port /dev/ttyACM0 arduino/main
