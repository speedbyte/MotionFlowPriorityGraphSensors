//
// Created by veikas on 02.10.17.
//

#ifndef MAIN_DATASETS_H_H
#define MAIN_DATASETS_H_H


#define KITTI_FLOW_DATASET_PATH "../../../kitti_flow_dataset/"
#define KITTI_RAW_DATASET_PATH "../../../kitti_raw_dataset/"

#define KITTI_RAW_CALIBRATION_PATH "../../../kitti_raw_dataset/data/"

#define MATLAB_DATASET_PATH "../../../matlab_dataset/"
#define CPP_DATASET_PATH "../../../cpp_dataset/"

#define MAX_ITERATION 716
#define MAX_ITERATION_THETA 360

//object specs
#define object_width 30
#define object_height 100

typedef enum {
    continous_frames = 0,
    pariwise_frames = 1,
    video_frames = 2
} FRAME_TYPES;






#endif //MAIN_DATASETS_H_H
