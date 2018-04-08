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

#define STEP_SIZE 5

#define MAX_ITERATION_RESULTS 20 // 60 generate result. this cannot be more than vector

#define MAX_ITERATION_GT_SCENE_GENERATION_VECTOR MAX_ITERATION_RESULTS   // generate_obj_base_pixel_position_pixel_displacement vector
#define MAX_ITERATION_GT_SCENE_GENERATION_IMAGES MAX_ITERATION_RESULTS   // generate images. this cannot be more than vector

#define MAX_ITERATION_GT_SCENE_GENERATION_DYNAMIC (MAX_ITERATION_RESULTS)*IMAGE_SKIP_FACTOR_DYNAMIC + IMAGE_SKIP_FACTOR_DYNAMIC*5  // 70
#define IMAGE_SKIP_FACTOR_DYNAMIC 10  // 10

#define MAX_ITERATION_THETA 360
#define MAX_SKIPS 2 // 1 means do not generate flow, 2 means generate base flow

#define DISTANCE_ERROR_TOLERANCE 1
#define ANGLE_ERROR_TOLERANCE 5

#define MAX_ALLOWED_OBJECTS 3

//#define STRETCH_HEIGHT 50
//#define STRETCH_WIDTH 50

#define STRETCH_HEIGHT_EVAL 5
#define STRETCH_WIDTH_EVAL 5

#define STENCIL_GRID_COMPRESSOR 2  // every x pixels
#define STENCIL_GRID_EXTENDER 10 // 10 % more than the original size

#define DO_STENCIL_GRID_EXTENSION 0

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
