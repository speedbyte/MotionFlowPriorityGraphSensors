#!/bin/bash
#This script should only be called after Apl

SOURCE_DIR=$(pwd)
echo $SOURCE_DIR
VTD_SCP_GENERATOR_EXE=$SOURCE_DIR/VIRES/VTD.2.0/Runtime/Tools/ScpGenerator/
PORT_SCP=48179


cd $VTD_SCP_GENERATOR_EXE
#./scpGenerator -p $PORT_SCP -i '<SimCtrl><Apply/></SimCtrl>'
#sleep 5
./scpGenerator -p $PORT_SCP -c $SOURCE_DIR/VIRES/SCP/Run.scp
# come back immediately.
#sleep 10


