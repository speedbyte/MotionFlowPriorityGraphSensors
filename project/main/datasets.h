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


#define MAX_ITERATION_RESULTS 10 // generate result. this cannot be more than vector

#define MAX_ITERATION_GT_SCENE_GENERATION_VECTOR 15   // generate_obj_base_pixel_point_pixel_displacement vector
#define MAX_ITERATION_GT_SCENE_GENERATION_IMAGES 10   // generate images. this cannot be more than vector

#define MAX_ITERATION_GT_SCENE_GENERATION_DYNAMIC 120*IMAGE_SKIP_FACTOR_DYNAMIC
#define IMAGE_SKIP_FACTOR_DYNAMIC 5

#define MAX_ITERATION_THETA 360
#define MAX_SKIPS 2 // 1 means do not generate flow, 2 means generate base flow

#define DISTANCE_ERROR_TOLERANCE 4
#define ANGLE_ERROR_TOLERANCE 10

#define STRETCH_HEIGHT 10
#define STRETCH_WIDTH 10

typedef enum {
    continous_frames = 0,
    pariwise_frames = 1,
    video_frames = 2
} FRAME_TYPES;

typedef enum {
    lk,
    fb
} ALGO_TYPES;




#endif //MAIN_DATASETS_H_H
