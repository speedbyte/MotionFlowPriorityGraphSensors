#!/bin/bash
echo $$
read -p "enter to continue"
SOURCE_DIR=$(pwd)
VIRES_DIR=$SOURCE_DIR/VIRES/VTD.2.0/bin
CPP_DIR=$SOURCE_DIR/project/vires_tutorials/cmake-build-debug
cd $VIRES_DIR 
./vtdStart.sh -setup="Standard_test" -project="SampleProject_test" -autoStart
VTD_PID=$!
pgrep -P $VTD_PID
ps -ef
#give some time
for i in `seq 1 10`; do
	sleep 1
	echo "Waiting"
done
cd $CPP_DIR
simserver_pid=$(cut -d ' ' -f2 /tmp/vtd_pids)
if ps -p $simserver_pid > /dev/null; then echo "PID is running"; else echo "PID is not running"; exit; fi
#read -p "enter to continue"
./shm_reader_writer trigger & 
trigger_pid=$!
echo "trigger_pid $!" >> /tmp/vtd_pids
sleep 1
./shm_reader_writer read & 
read_pid=$!
echo "read_pid $!" >> /tmp/vtd_pids

#for i in `seq 1 10`; do
#	sleep 1
#	echo $((100-$i)) seconds till abort ... 

cd $VIRES_DIR 
./vtdStop.sh
kill -9 $trigger_pid
kill -9 $read_pid
