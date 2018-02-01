//
// Created by veikas on 26.01.18.
//

#ifndef MAIN_DATASET_H
#define MAIN_DATASET_H


#include <opencv2/core/types.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

typedef struct {


    cv::Size_<unsigned> m_frame_size;
    // /local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset
    boost::filesystem::path m_dataset_path;

    // /local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset/data/stereo_flow/image_02
    boost::filesystem::path  m__directory_path_input;

    // /local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset/data/stereo_flow/flow_occ_01
    boost::filesystem::path  m__directory_path_gt_flow;

    // /local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset/data/stereo_flow/trajectroy_occ_01
    boost::filesystem::path  m__directory_path_gt_trajectory;

    // /local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset/results/results_LK/flow_occ_01
    boost::filesystem::path m__directory_path_result_flow;

    // /local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset/results/results_LK/flow_occ_01
    boost::filesystem::path m__directory_path_result_trajectory_out;

    ushort m_depth;

    ushort m_cn;

} DATASET_STRUCT;

static DATASET_STRUCT  datasetStruct;

class Dataset {


private:

    Dataset() {}

public:

    static void fillDataset(cv::Size_<unsigned> frame_size, ushort depth, ushort cn, std::string dataset_path,
            std::string unterordner, std::string resultordner);

    const static cv::Size_<unsigned> getFrameSize()  ;

    const static boost::filesystem::path getBasePath() ;
    
    const static boost::filesystem::path getInputPath()  ;

    const static boost::filesystem::path getResultPath();

    const static boost::filesystem::path getGroundTruthFlowPath();

    const static ushort getDepth();

    const static ushort getChannel();

    const static ushort getMakeType();

    const static boost::filesystem::path getGroundTruthTrajectoryPath();

    const static boost::filesystem::path getResultTrajectoryPath();



};


#endif //MAIN_DATASET_H
