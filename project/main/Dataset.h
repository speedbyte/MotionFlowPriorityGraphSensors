//
// Created by veikas on 26.01.18.
//

#ifndef MAIN_DATASET_H
#define MAIN_DATASET_H

#include <opencv2/core/types.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <map>

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


class Dataset {

private:
    Dataset() {}

public:

    static cv::Size_<unsigned> m_frame_size;
    static ushort m_depth;
    static ushort m_cn;

    static ushort SENSOR_COUNT;
    static ushort ITERATION_START_POINT;
    static ushort ITERATION_END_POINT;
    static bool GENERATE;
    static ushort MAX_ITERATION_RESULTS;
    static ushort MAX_GENERATION_DATASET;
    static ushort MAX_ITERATION_GT_SCENE_GENERATION_DATASET;

    static boost::filesystem::path m_dataset_basepath;
    static boost::filesystem::path  m_dataset_gtpath;
    static boost::filesystem::path  m_dataset_resultpath;

    static std::map<std::string, bool> m_dataprocessing_map;
    static std::map<std::string, ushort> m_algorithm_map;
    static std::map<std::string, ushort> m_algorithm_map_original;

    static bool m_execute_algorithm;

    static void fillDataset(cv::Size_<unsigned> frame_size, ushort depth, ushort cn, std::string dataset_path,
            std::string unterordner, std::string resultordner, bool generate, ushort start, ushort stop, ushort max_frames_dataset, std::map<std::string, bool> dataprocessing_map,
                            std::map<std::string, ushort> algorithm_map, std::vector<ushort> evaulation_list);

    static const ushort getMakeType();

};


#endif //MAIN_DATASET_H
