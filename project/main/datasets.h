//
// Created by veikas on 02.10.17.
//


#ifndef MAIN_DATASETS_H_H
#define MAIN_DATASETS_H_H


//typedef std::vector<std::vector<std::vector<cv::Point2f > > > Container_List_Algorithms;

#define DEFAULT_RX_PORT_CAM_0     48182   /* for image port it should be 48192 */
#define DEFAULT_RX_PORT_PERFECT_0     48183   /* for image port it should be 48192 */
#define DEFAULT_RX_PORT_PERFECT_INERTIAL_0     48184   /* for image port it should be 48192 */

#define DEFAULT_RX_PORT_CAM_1     48185   /* for image port it should be 48192 */
#define DEFAULT_RX_PORT_PERFECT_1     48186   /* for image port it should be 48192 */
#define DEFAULT_RX_PORT_PERFECT_INERTIAL_1     48187   /* for image port it should be 48192 */


#define KITTI_FLOW_DATASET_PATH "../../../datasets/kitti_flow_dataset/"
#define KITTI_RAW_DATASET_PATH "../../../datasets/kitti_raw_dataset/"

#define KITTI_RAW_CALIBRATION_PATH "../../../datasets/kitti_raw_dataset/data/"

#define MATLAB_DATASET_PATH "../../../datasets/matlab_dataset/"
#define CPP_DATASET_PATH "../../../datasets/cpp_dataset/"

#define VIRES_DATASET_PATH "../../../datasets/vires_dataset/"

#define DISPLACEMENT_ROUND_OFF 100

#define ITERATION_START_POINT 15
#define ITERATION_END_POINT 65
#define MAX_ITERATION_RESULTS (ITERATION_END_POINT - ITERATION_START_POINT) // 60 generate result. this cannot be more than vector

#define MAX_ITERATION_DATASET MAX_ITERATION_RESULTS // 60 generate result. this cannot be more than vector
#define MAX_ITERATION_GT_SCENE_GENERATION_DATASET (MAX_ITERATION_DATASET)*IMAGE_SKIP_FACTOR_DYNAMIC + IMAGE_SKIP_FACTOR_DYNAMIC*20  // generate always five images more than required.
#define IMAGE_SKIP_FACTOR_DYNAMIC 1  // 10

#define MAX_ITERATION_THETA 360

#define MAX_SKIPS_REAL 1 // 1 means do not generate flow, 2 means generate base flow

#define IMAGE_WIDTH 1200
#define IMAGE_HEIGHT 400

#define ANGLE_ERROR_TOLERANCE 5

#define MAX_ALLOWED_OBJECTS 10
#define MAX_ALLOWED_SENSORS 10

#define MAX_ALLOWED_SENSOR_GROUPS 1
#define SENSOR_COUNT MAX_ALLOWED_SENSOR_GROUPS + (1%MAX_ALLOWED_SENSOR_GROUPS)


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
    fb
} ALGO_TYPES;

#define DATAFILTER_COUNT 1



#endif //MAIN_DATASETS_H_H
