//
// Created by veikas on 26.01.18.
//

#include <map>
#include "Dataset.h"

cv::Size_<unsigned> Dataset::m_frame_size;
ushort Dataset::m_depth;
ushort Dataset::m_cn;
ushort Dataset::Dataset::SENSOR_COUNT;
ushort Dataset::ITERATION_START_POINT;
ushort Dataset::ITERATION_END_POINT;
bool Dataset::GENERATE;
ushort Dataset::MAX_ITERATION_RESULTS;
ushort Dataset::MAX_GENERATION_DATASET;
ushort Dataset::MAX_ITERATION_GT_SCENE_GENERATION_DATASET;
boost::filesystem::path Dataset::m_dataset_basepath;
boost::filesystem::path  Dataset::m_dataset_gtpath;
boost::filesystem::path  Dataset::m_dataset_resultpath;
std::map<std::string, bool> Dataset::m_dataprocessing_map;
std::map<std::string, ushort> Dataset::m_algorithm_map;
std::map<std::string, ushort> Dataset::m_algorithm_map_original;
bool Dataset::m_execute_algorithm = false;


void Dataset::fillDataset(cv::Size_<unsigned> frame_size, ushort depth, ushort cn, std::string dataset_path,
                          std::string unterordner, std::string resultordner, bool generate, ushort start, ushort stop, ushort max_frames_dataset, std::map<std::string, bool> dataprocessing_map,
                          std::map<std::string, ushort> algorithm_map, std::vector<ushort> evaluation_list) {

    Dataset::SENSOR_COUNT = (ushort)(evaluation_list.size() + 1%evaluation_list.size());

    ITERATION_START_POINT = start;

    ITERATION_END_POINT = stop;

    GENERATE = generate;

    MAX_ITERATION_RESULTS = (ITERATION_END_POINT - ITERATION_START_POINT);// 60 generate result. this cannot be more than vector

    MAX_GENERATION_DATASET = max_frames_dataset;  // generate always twenty images more than required.

    m_frame_size = frame_size;

    m_depth = depth;

    m_cn = cn;

    m_dataprocessing_map = dataprocessing_map;

    m_algorithm_map = algorithm_map;

    m_algorithm_map_original = algorithm_map;

    for ( auto my_map: m_algorithm_map) {
        if ( my_map.second > 0 ) {
            m_execute_algorithm = true;
            break;
        }
    }

    m_dataset_basepath = dataset_path;

    m_dataset_gtpath = m_dataset_basepath.string() + unterordner;

    m_dataset_resultpath = m_dataset_basepath.string() + resultordner;

}

const ushort Dataset::getMakeType() {
    return static_cast<ushort>(CV_MAKETYPE(m_depth, m_cn));
}
