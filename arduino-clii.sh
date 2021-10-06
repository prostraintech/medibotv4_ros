BOARD=${2:-"uno"}
FILE=${1:-flashbot}

if [ $BOARD == "due" ]
then
   ./arduino-clii compile --fqbn per1234:sam:arduino_due_x_dbg $FILE 
   ./arduino-clii upload  --fqbn per1234:sam:arduino_due_x_dbg $FILE -p /dev/ttyACM0
elif [ $BOARD == "uno" ]
then
   ./arduino-clii compile --fqbn arduino:avr:uno $FILE 
   ./arduino-clii upload  --fqbn arduino:avr:uno $FILE -p /dev/ttyACM0
fi




