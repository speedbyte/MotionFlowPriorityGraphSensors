//
// Created by veikas on 26.01.18.
//

#include "Dataset.h"

void Dataset::fillDataset(cv::Size_<unsigned> frame_size, ushort depth, ushort cn, std::string dataset_path,
        std::string unterordner, std::string resultordner) {

    datasetStruct.m_frame_size = frame_size;

    datasetStruct.m_depth = depth;

    datasetStruct.m_cn = cn;

    datasetStruct.m_dataset_basepath = dataset_path;

    datasetStruct.m_directory_path_gt = datasetStruct.m_dataset_basepath.string() + unterordner;

    datasetStruct.m_directory_path_result = datasetStruct.m_dataset_basepath.string() + resultordner;

}

const cv::Size_<unsigned> Dataset::getFrameSize() {
    return datasetStruct.m_frame_size;
}

const ushort Dataset::getDepth() {
    return datasetStruct.m_depth;
}

const ushort Dataset::getChannel() {
    return datasetStruct.m_cn;
}

const ushort Dataset::getMakeType() {
    return static_cast<ushort>(CV_MAKETYPE(datasetStruct.m_depth, datasetStruct.m_cn));
}

const boost::filesystem::path Dataset::getDatasetPath() {
    return datasetStruct.m_dataset_basepath;
}

//data/stereo_flow/
const boost::filesystem::path Dataset::getGroundTruthPath() {
    return datasetStruct.m_directory_path_gt;
}

//resultsgenerated/
const boost::filesystem::path Dataset::getResultPath() {
    return datasetStruct.m_directory_path_result;
}
