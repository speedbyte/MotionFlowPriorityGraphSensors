#!/bin/bash
if [ $# == 2 ]; then
SETUP=$1
PROJECT=$2
elif [ $# == 1 ]; then
SETUP="Standard_test"
PROJECT=$1
else
SETUP="Standard_test"
PROJECT="SampleProject_test"
fi
echo $SETUP
echo $PROJECT

tempdir=$(mktemp -t -d fifodir.XXXXXX)
trap 'rm -rf "$tempdir"' EXIT
mkfifo "$tempdir/child"

echo "this script pid = $$"
SOURCE_DIR=$(pwd)
VIRES_DIR=$SOURCE_DIR/VIRES/VTD.2.0/bin
CPP_DIR=$SOURCE_DIR/project/vires_tutorials/cmake-build-debug
VIRES_DATASET_DIR=$SOURCE_DIR/vires_dataset/data/stereo_flow/image_02
cd $VIRES_DATASET_DIR
rm *.png
cd $VIRES_DIR 
./vtdStart.sh -setup=$SETUP -project=$PROJECT -autoStart &
echo "vtdStart.sh pid = $!"
#give some time
for i in `seq 1 10`; do
	sleep 1
	echo "Waiting again"
done
#This will block until and unless there is a value in the fifo. This makes sure that the processes are synchronous.
#read simserver_pid <"$tempdir/child"

simserver_pid=`pgrep simServer | head -1`
echo "simserver pid is $simserver_pid"
if [[ simserver_pid -gt 0 ]]; then echo "PID is running"; else echo "PID is not running"; ./vtdStop.sh; exit; fi
cd $CPP_DIR
#./shm_reader_writer trigger & 
#trigger_pid=$!
#sleep 1
./shm_trigger_read triggerandread > $VIRES_DATASET_DIR/file.log 2>&1 & 
read_pid=$!

for i in `seq 1 10`; do
	sleep 1
	echo $((100-$i)) seconds till abort ... 
done

#kill -9 $trigger_pid
kill -9 $read_pid
cd $VIRES_DIR 
./vtdStop.sh
cd $VIRES_DATASET_DIR
find ./ -name '*.png' -exec convert -flip {} {} \;
