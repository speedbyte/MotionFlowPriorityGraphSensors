#!/bin/bash

echo "this script pid = $$"

IMAGE_PATH="image_02"

SOURCE_DIR=$(pwd)
VIRES_DIR=$SOURCE_DIR/VIRES/VTD.2.0/bin
VIRES_DATASET_DIR=$SOURCE_DIR/datasets/vires_dataset
VIRES_GROUNDTRUTH_DIR=$VIRES_DATASET_DIR/data/stereo_flow/$IMAGE_PATH

for i in {1..2}; do
	sleep 1
	echo $((100-$i)) seconds till abort ... 
done

cd $VIRES_DIR 
./vtdStop.sh
cd $VIRES_GROUNDTRUTH_DIR
#find ./ -name '*.png' -exec convert -flip {} {} \;
#mogrify -flip *.png
#Extract flow from log file
#grep -e '^INDICATOR' file.log > FLow.log

