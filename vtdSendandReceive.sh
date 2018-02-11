#!/bin/bash

#call: bash vtdStartandReceive Project SCPDirectory image_dir
#bash vtdStartandReceive.sh Movement movingCar image_02_moving_car

SETUP="Standard_test"

if [ $# == 1 ]; then	#set project
PROJECT=$1
else
PROJECT="Movement"
fi

echo $SETUP
echo $PROJECT

echo "this script pid = $$"
SOURCE_DIR=$(pwd)
echo $SOURCE_DIR
VIRES_DIR=$SOURCE_DIR/VIRES/VTD.2.0/bin
VIRES_DATASET_DIR=$SOURCE_DIR/datasets/vires_dataset

cd $VIRES_DIR 
./vtdStart.sh -setup=$SETUP -project=$PROJECT & 
echo "vtdStart.sh pid = $!"


#for i in $(seq 1 30); do
#	sleep 1
#	echo "Wait"
#done 

#simserver_pid=`pgrep simServer | head -1`
#echo "simserver pid is $simserver_pid"
#if [[ simserver_pid -gt 0 ]]; then echo "PID is running"; else echo "PID is not running"; ./vtdStop.sh; exit; fi

function call_cpp()
{
	CPP_DIR=$SOURCE_DIR/project/main/cmake-build-debug
	cd $CPP_DIR
	./main > file.log 2>&1 & 
	read_pid=$!
}

function stop()
{
	./vtdStop.sh
}

