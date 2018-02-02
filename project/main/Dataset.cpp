//
// Created by veikas on 26.01.18.
//

#include "Dataset.h"

void Dataset::fillDataset(cv::Size_<unsigned> frame_size, ushort depth, ushort cn, std::string dataset_path,
        std::string unterordner, std::string resultordner) {

    datasetStruct.m_frame_size = frame_size;

    datasetStruct.m_depth = depth;

    datasetStruct.m_cn = cn;

    datasetStruct.m_dataset_path = dataset_path;

    //data/stereo_flow/image_02/dummy.txt
    datasetStruct.m__directory_path_gt = datasetStruct.m_dataset_path.string() + unterordner + std::string("/dummy"
                                                                                                                   ".txt");
    //data/stereo_flow/image_02
    datasetStruct.m__directory_path_gt = datasetStruct.m__directory_path_gt.parent_path();

    //data/stereo_flow/
    datasetStruct.m__directory_path_gt_flow = datasetStruct.m__directory_path_gt.parent_path();

    //data/stereo_flow/
    datasetStruct.m__directory_path_gt_trajectory = datasetStruct.m__directory_path_gt.parent_path();

    /* result------------------------------------------------------------------------- */

    //results/resultordner/dummy.txt
    datasetStruct.m__directory_path_result = datasetStruct.m_dataset_path.string() + resultordner + std::string("/dummy.txt");
    //results/resultordner
    datasetStruct.m__directory_path_result = datasetStruct.m__directory_path_result.parent_path();

    //results/resultordner/dummy.txt
    datasetStruct.m__directory_path_result_flow = datasetStruct.m_dataset_path.string() + resultordner + std::string("/dummy.txt");
    //results/resultordner
    datasetStruct.m__directory_path_result_flow = datasetStruct.m__directory_path_result_flow.parent_path();

    //results/resultordner/dummy.txt
    datasetStruct.m__directory_path_result_trajectory_out = datasetStruct.m_dataset_path.string() + resultordner + std::string
                                                                                                              ("/dummy.txt");
    //results/resultordner
    datasetStruct.m__directory_path_result_trajectory_out = datasetStruct.m__directory_path_result_trajectory_out.parent_path();

    if (boost::filesystem::exists(datasetStruct.m__directory_path_gt) == 0 ) {
    boost::filesystem::create_directories(datasetStruct.m__directory_path_gt.string());
    }

    // Prepare 10 directories for 10 object trajectories
    for ( int i = 0; i < 10; i++ ) {
    boost::filesystem::path objectTrajectory = datasetStruct.m__directory_path_gt_trajectory.string() + std::string
            ("_object_") + std::to_string(i);

        if ( boost::filesystem::exists(objectTrajectory) == 0 ) {
        boost::filesystem::create_directories(objectTrajectory);
        }
    }
}

const cv::Size_<unsigned> Dataset::getFrameSize() {
    return datasetStruct.m_frame_size;
}

const boost::filesystem::path Dataset::getBasePath() {
    return datasetStruct.m_dataset_path;
}

const boost::filesystem::path Dataset::getGtPath() {
    return datasetStruct.m__directory_path_gt;
}

const boost::filesystem::path Dataset::getGroundTruthTrajectoryPath() {
    return datasetStruct.m__directory_path_gt_trajectory;
}

const boost::filesystem::path Dataset::getGroundTruthFlowPath() {
    return datasetStruct.m__directory_path_gt_flow;
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

const boost::filesystem::path Dataset::getResultPath() {
    return datasetStruct.m__directory_path_result;
}

const boost::filesystem::path Dataset::getResultFlowPath() {
    return datasetStruct.m__directory_path_result_flow;
}

const boost::filesystem::path Dataset::getResultTrajectoryPath() {
    return datasetStruct.m__directory_path_result_trajectory_out;
}
