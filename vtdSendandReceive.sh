#!/bin/bash

#call: bash vtdStartandReceive Project SCPDirectory image_dir
#bash vtdStartandReceive.sh Movement movingCar image_02_moving_car

SETUP="Standard_test"

if [ $# == 1 ]; then	#set project
PROJECT=$1
SCP="Normal"
RESULT_PATH="image_02"
elif [ $# == 2 ]; then	#set project and SCP
PROJECT=$1
SCP=$2
RESULT_PATH="image_02"
elif [ $# == 3 ]; then	#set project and SCP and result directory
PROJECT=$1
SCP=$2
RESULT_PATH=$3
else
PROJECT="Movement"
SCP="Normal"
RESULT_PATH="image_02"
fi

echo $SETUP
echo $PROJECT
echo $SCP
echo $IMAGE_PATH

echo "this script pid = $$"
SOURCE_DIR=$(pwd)
echo $SOURCE_DIR
VIRES_DIR=$SOURCE_DIR/VIRES/VTD.2.0/bin
VIRES_DATASET_DIR=$SOURCE_DIR/datasets/vires_dataset
VIRES_GROUNDTRUTH_DIR=$VIRES_DATASET_DIR/data/stereo_flow/$IMAGE_PATH
VTD_SCP_GENERATOR_EXE=$SOURCE_DIR/VIRES/VTD2.0/Runtime/Tools/ScpGenerator/
PORT_SCP=48179

cd $VIRES_GROUNDTRUTH_DIR
find ./ -maxdepth 1 -name '*.png*' -exec rm {} \;
cd $VIRES_DIR 
./vtdStart.sh -setup=$SETUP -project=$PROJECT & 
echo "vtdStart.sh pid = $!"

function test()
{
	echo "test"	
}

for i in $(seq 1 10); do
	sleep 1
	echo "Wait"
done 

simserver_pid=`pgrep simServer | head -1`
echo "simserver pid is $simserver_pid"
if [[ simserver_pid -gt 0 ]]; then echo "PID is running"; else echo "PID is not running"; ./vtdStop.sh; exit; fi

cd $SOURCE_DIR/VIRES/VTD.2.0/Runtime/Tools/ScpGenerator/
./scpGenerator -p $PORT_SCP -i '<SimCtrl><Apply/></SimCtrl>'
sleep 5
./scpGenerator -p $PORT_SCP -c $SOURCE_DIR/VIRES/SCP/$SCP/Run.scp
sleep 10

function call_cpp()
{
	CPP_DIR=$SOURCE_DIR/project/main/cmake-build-debug
	#CPP_DIR=$SOURCE_DIR/project/vires_tutorials/cmake-build-debug
	cd $CPP_DIR
	./main > $VIRES_GROUNDTRUTH_DIR/file.log 2>&1 & 
	read_pid=$!
}

function stop()
{
	./vtdStop.sh
}
