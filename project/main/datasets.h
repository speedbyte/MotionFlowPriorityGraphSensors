//
// Created by veikas on 02.10.17.
//


#ifndef MAIN_DATASETS_H_H
#define MAIN_DATASETS_H_H


//typedef std::vector<std::vector<std::vector<cv::Point2f > > > Container_List_Algorithms;


#define KITTI_FLOW_DATASET_PATH "../../../datasets/kitti_flow_dataset/"
#define KITTI_RAW_DATASET_PATH "../../../datasets/kitti_raw_dataset/"

#define KITTI_RAW_CALIBRATION_PATH "../../../datasets/kitti_raw_dataset/data/"

#define MATLAB_DATASET_PATH "../../../datasets/matlab_dataset/"
#define CPP_DATASET_PATH "../../../datasets/cpp_dataset/"

#define VIRES_DATASET_PATH "../../../datasets/vires_dataset/"

#define DISPLACEMENT_ROUND_OFF 100

#define IMAGE_SKIP_FACTOR_DYNAMIC 1

#define MAX_SKIPS_REAL 1 // 1 means do not generate flow, 2 means generate base flow

#define IMAGE_WIDTH 1200
#define IMAGE_HEIGHT 400

#define ANGLE_ERROR_TOLERANCE 5

#define MAX_ALLOWED_OBJECTS 10
#define MAX_ALLOWED_SENSORS 10

//#define MAX_ALLOWED_SENSOR_GROUPS_EVALUATION 2
//#define Dataset::SENSOR_COUNT MAX_ALLOWED_SENSOR_GROUPS_EVALUATION + (1%MAX_ALLOWED_SENSOR_GROUPS_EVALUATION)

#define STENCIL_GRID_EXTENDER 15 // 10 pixels more than roi on each side

#define DO_STENCIL_GRID_EXTENSION 0

typedef enum {
    continous_frames = 0,
    pariwise_frames = 1,
    video_frames = 2
} FRAME_TYPES;

typedef enum {
    gt,
    lk,
    fb,
    sf,
    tvl
} ALGO_TYPES;

#define FOCAL_X 600
#define FOCAL_Y 600

#define START_BENCHMARK tic = std::chrono::steady_clock::now();
#define PRINT_BENCHMARK(prefix) printf ( #prefix" - %f\n", static_cast<float>(std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - tic).count()) );



#endif //MAIN_DATASETS_H_H
