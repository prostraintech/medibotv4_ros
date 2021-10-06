BOARD=${2:-"due"}
FILE=${1:-flashbot}

if [ $BOARD == "due" ]
then
   ./arduino-cli compile --fqbn per1234:sam:arduino_due_x_dbg $FILE 
   ./arduino-cli upload  --fqbn per1234:sam:arduino_due_x_dbg $FILE -p /dev/ttyACM0
elif [ $BOARD == "uno" ]
then
   ./arduino-cli compile --fqbn arduino:avr:uno $FILE 
   ./arduino-cli upload  --fqbn arduino:avr:uno $FILE -p /dev/ttyACM0
fi