#!/bin/bash

SETUP="Standard_test"
PROJECT="Movement"
SCP="test"

SOURCE_DIR=$(pwd)
echo $SOURCE_DIR
VIRES_DIR=$SOURCE_DIR/../VIRES/VTD.2.0/bin
VTD_SCP_GENERATOR_EXE=$SOURCE_DIR/../VIRES/VTD2.0/Runtime/Tools/ScpGenerator/
PORT_SCP=48179

cd $VIRES_DIR 
./vtdStart.sh -setup=$SETUP -project=$PROJECT & 
echo "vtdStart.sh pid = $!"

sleep 3
cd ../Runtime/Tools/ScpGenerator/
./scpGenerator -p $PORT_SCP -c $SOURCE_DIR/apply.scp
sleep 5
./scpGenerator -p $PORT_SCP -c $SOURCE_DIR/$SCP/Run.scp
sleep 10

read -p "Darn, that weather!Press enter to let it snow"


./scpGenerator -p $PORT_SCP -i '<Info level="info"> <Message popup="true" text="snow!!!!"/> </Info>'
./scpGenerator -p $PORT_SCP -i '<Symbol name="expl01" > <Text data="Let it snow(Let it snow, let it snow)" colorRGB="0xffffff" size="50.0" /> <PosScreen x="0.01" y="0.05" /></Symbol>'
./scpGenerator -p $PORT_SCP -i '<Environment><Friction value="1.000000" /><TimeOfDay headlights="false" value="39600" /><Sky cloudState="4/8" visibility="100000.000000" /><Precipitation intensity="1.000000" type="snow" /><Road effectScale="0.500000" state="wet" /></Environment>'

read -p "Press enter for summer"

./scpGenerator -p $PORT_SCP -i '<Info level="info"> <Message popup="true" text="Summer!!!!"/> </Info>'
./scpGenerator -p $PORT_SCP -i '<Symbol name="expl01" > <Text data="Time for Summer" colorRGB="0xffff00" size="50.0" /> <PosScreen x="0.01" y="0.05" /></Symbol>'
./scpGenerator -p $PORT_SCP -i '<Environment><Friction value="1.000000" /><TimeOfDay headlights="false" value="39600" /><Sky cloudState="0/8" visibility="100000.000000" /><Precipitation intensity="1.000000" type="none" /><Road effectScale="0.500000" state="dry" /></Environment>'

read -p "Press enter to stop"
./scpGenerator -p $PORT_SCP -i '<SimCtrl> <Stop/> </SimCtrl>'

cd $VIRES_DIR
simserver_pid=`pgrep simServer | head -1`
echo "simserver pid is $simserver_pid"
if [[ simserver_pid -gt 0 ]]; then echo "PID is running"; else echo "PID is not running"; ./vtdStop.sh; exit; fi
./vtdStop.sh


