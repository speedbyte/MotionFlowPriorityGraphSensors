//
// Created by veikas on 26.01.18.
//

#include "Dataset.h"

cv::Size_<unsigned> Dataset::m_frame_size;
ushort Dataset::m_depth;
ushort Dataset::m_cn;
ushort Dataset::ITERATION_START_POINT;
ushort Dataset::ITERATION_END_POINT;
ushort Dataset::MAX_ITERATION_RESULTS;
ushort Dataset::MAX_ITERATION_DATASET;
ushort Dataset::MAX_ITERATION_GT_SCENE_GENERATION_DATASET;
boost::filesystem::path Dataset::m_dataset_basepath;
boost::filesystem::path  Dataset::m_directory_path_gt;
boost::filesystem::path  Dataset::m_directory_path_result;


void Dataset::fillDataset(cv::Size_<unsigned> frame_size, ushort depth, ushort cn, std::string dataset_path,
                          std::string unterordner, std::string resultordner, ushort start, ushort stop) {

    ITERATION_START_POINT = start;

    ITERATION_END_POINT = stop;

    MAX_ITERATION_RESULTS = (ITERATION_END_POINT - ITERATION_START_POINT);// 60 generate result. this cannot be more than vector

    MAX_ITERATION_DATASET = MAX_ITERATION_RESULTS; // 60 generate result. this cannot be more than vector

    MAX_ITERATION_GT_SCENE_GENERATION_DATASET = (MAX_ITERATION_DATASET) + (ushort)20;  // generate always twenty images more than required.

    m_frame_size = frame_size;

    m_depth = depth;

    m_cn = cn;

    m_dataset_basepath = dataset_path;

    m_directory_path_gt = m_dataset_basepath.string() + unterordner;

    m_directory_path_result = m_dataset_basepath.string() + resultordner;

}

const cv::Size_<unsigned> Dataset::getFrameSize() {
    return m_frame_size;
}

const ushort Dataset::getDepth() {
    return m_depth;
}

const ushort Dataset::getChannel() {
    return m_cn;
}

const ushort Dataset::getMakeType() {
    return static_cast<ushort>(CV_MAKETYPE(m_depth, m_cn));
}

const boost::filesystem::path Dataset::getDatasetPath() {
    return m_dataset_basepath;
}

//data/stereo_flow/
const boost::filesystem::path Dataset::getGroundTruthPath() {
    return m_directory_path_gt;
}

//resultsgenerated/
const boost::filesystem::path Dataset::getResultPath() {
    return m_directory_path_result;
}
