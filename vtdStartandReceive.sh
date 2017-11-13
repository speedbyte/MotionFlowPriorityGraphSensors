#!/bin/bash

#call: bash vtdStartandReceive Project SCPDirectory image_dir
#example bash vtdStartandReceive Movement movingCar image_02_moving_car

if [ $# == 4 ]; then
SETUP=$1
PROJECT=$2
SCP=$3
RESULT_PATH=$4
elif [ $# == 1 ]; then	#set project
SETUP="Standard_test"
PROJECT=$1
SCP="Normal"
elif [ $# == 2 ]; then	#set project and SCP
SETUP="Standard_test"
PROJECT=$1
SCP=$2
elif [ $# == 3 ]; then	#set project and SCP and result directory
SETUP="Standard_test"
PROJECT=$1
SCP=$2
RESULT_PATH=$3
else
SETUP="Standard_test"
PROJECT="SampleProject_test"
SCP="Normal"
fi
echo $SETUP
echo $PROJECT
echo $SCP

tempdir=$(mktemp -t -d fifodir.XXXXXX)
trap 'rm -rf "$tempdir"' EXIT
mkfifo "$tempdir/child"

echo "this script pid = $$"
SOURCE_DIR=$(pwd)
echo $SOURCE_DIR
VIRES_DIR=$SOURCE_DIR/VIRES/VTD.2.0/bin
CPP_DIR=$SOURCE_DIR/project/vires_tutorials/cmake-build-debug
VIRES_DATASET_DIR=$SOURCE_DIR/vires_dataset/data/stereo_flow/$RESULT_PATH
VTD_SCP_GENERATOR_EXE=$SOURCE_DIR/VIRES/VTD2.0/Runtime/Tools/ScpGenerator/
PORT_SCP=48179





cd $VIRES_DATASET_DIR
rm *.png
cd $VIRES_DIR 
./vtdStart.sh -setup=$SETUP -project=$PROJECT & 
echo "vtdStart.sh pid = $!"
#variable="dummy"
#read -p "First press apply and then play in VIRES and then Enter here to start the cpp data #transfer" -t 1 -N 1 variable
#echo -e "\n"
#while [ "$variable" != $'\x0a' ] 
#do
#IFS=''
#read -p "First press apply and then play in VIRES and then Enter here to start the cpp data #transfer" -t 1 -N 1 variable
#echo -e "\n" 
#done
#This will block until and unless there is a value in the fifo. This makes sure that the processes are synchronous.
#read simserver_pid <"$tempdir/child"
#IFS=$'\n'

sleep 3
cd /local/git/PriorityGraphSensors/VIRES/VTD.2.0/Runtime/Tools/ScpGenerator/
./scpGenerator -p $PORT_SCP -c $SOURCE_DIR/SCP/apply.scp
sleep 5
./scpGenerator -p $PORT_SCP -c $SOURCE_DIR/SCP/$SCP/Run.scp
sleep 10

simserver_pid=`pgrep simServer | head -1`
echo "simserver pid is $simserver_pid"
if [[ simserver_pid -gt 0 ]]; then echo "PID is running"; else echo "PID is not running"; ./vtdStop.sh; exit; fi
cd $CPP_DIR
#./shm_reader_writer trigger & 
#trigger_pid=$!
#sleep 1
#I would like to pass the output dir to c++ as argument and then store it there.
# The procedure is clear but c++ tricks me(and I suck at it) and I dont want to waste time now.
# It would be cool if we can do this together if you have some spare time in December :)
./shm_trigger_read triggerandread > $VIRES_DATASET_DIR/file.log 2>&1 & 
read_pid=$!

for i in `seq 1 40`; do
	sleep 1
	echo $((100-$i)) seconds till abort ... 
done


#kill -9 $trigger_pid
kill -9 $read_pid
cd $VIRES_DIR 
./vtdStop.sh
cd $VIRES_DATASET_DIR
#find ./ -name '*.png' -exec convert -flip {} {} \;
mogrify -flip *.png
#Extract flow from log file
grep -e '^INDICATOR' file.log > FLow.log

