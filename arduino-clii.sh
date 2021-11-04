BOARD=${2:-"uno"}
FILE=${1:-flashbot}

if [ $BOARD == "due" ]
then
   ./arduino-clii compile --fqbn per1234:sam:arduino_due_x_dbg $FILE --libraries /arduino/main/libraries/Arduino_PID_Library,/arduino/main/libraries/Rosserial_Arduino_Library
   ./arduino-clii upload  --fqbn per1234:sam:arduino_due_x_dbg $FILE -p /dev/ttyACM0
elif [ $BOARD == "uno" ]
then
   ./arduino-clii compile --fqbn arduino:avr:uno $FILE --libraries /arduino/main/libraries/Arduino_PID_Library,/arduino/main/libraries/Rosserial_Arduino_Library
   ./arduino-clii upload  --fqbn arduino:avr:uno $FILE -p /dev/ttyACM0
fi




