#!/bin/bash

./arduino-cli compile --fqbn per1234:sam:arduino_due_x_dbg arduino/main
./arduino-cli upload  --fqbn per1234:sam:arduino_due_x_dbg --port /dev/ttyACM0 arduino/main
