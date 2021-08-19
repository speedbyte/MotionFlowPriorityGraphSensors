#!/bin/bash

for d in $(find data -type f); do
    INFILE="$d"
    OUTFILE=pcd/$(basename $d | cut -f 1 -d '.').pcd
    #echo ${OUTFILE}
    $(/local/gackstat/PriorityGraphSensors/kitti_eval/kitti-pcl/cmake-build-debug/bin/kitti2pcd --infile ${INFILE} --outfile ${OUTFILE})
done
