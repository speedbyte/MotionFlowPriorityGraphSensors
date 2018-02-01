//
// Created by veikas on 02.10.17.
//

#ifndef MAIN_DATASETS_H_H
#define MAIN_DATASETS_H_H

#define KITTI_FLOW_DATASET_PATH "../../../datasets/kitti_flow_dataset/"
#define KITTI_RAW_DATASET_PATH "../../../datasets/kitti_raw_dataset/"

#define KITTI_RAW_CALIBRATION_PATH "../../../datasets/kitti_raw_dataset/data/"

#define MATLAB_DATASET_PATH "../../../datasets/matlab_dataset/"
#define CPP_DATASET_PATH "../../../datasets/cpp_dataset/"

#define VIRES_DATASET_PATH "../../../datasets/vires_dataset/"


#define MAX_ITERATION_RESULTS 360
#define MAX_ITERATION_GT 360
#define MAX_ITERATION_THETA 360
#define MAX_SKIPS 20


typedef enum {
    continous_frames = 0,
    pariwise_frames = 1,
    video_frames = 2
} FRAME_TYPES;

typedef enum {
    lk,
    fb
} ALGO_TYPES;

typedef enum {
    no_noise = 0,
    static_bg_noise = 1,
    static_fg_noise = 2,
    dynamic_bg_noise = 3,
    dynamic_fg_noise =4
} NOISE_TYPES;



#endif //MAIN_DATASETS_H_H
