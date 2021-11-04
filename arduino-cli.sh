BOARD=${2:-"due"}
FILE=${1:-flashbot}

if [ $BOARD == "due" ]
then
   ./arduino-cli compile --fqbn per1234:sam:arduino_due_x_dbg $FILE --libraries /arduino/main/libraries/Arduino_PID_Library,/arduino/main/libraries/Rosserial_Arduino_Library
   ./arduino-cli upload  --fqbn per1234:sam:arduino_due_x_dbg $FILE -p /dev/ttyACM0
elif [ $BOARD == "uno" ]
then
   ./arduino-cli compile --fqbn arduino:avr:uno $FILE --libraries /arduino/main/libraries/Arduino_PID_Library,/arduino/main/libraries/Rosserial_Arduino_Library
   ./arduino-cli upload  --fqbn arduino:avr:uno $FILE -p /dev/ttyACM0
fi