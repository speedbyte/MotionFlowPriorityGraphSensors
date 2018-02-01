//
// Created by veikas on 26.01.18.
//

#ifndef MAIN_DATASET_H
#define MAIN_DATASET_H


#include <opencv2/core/types.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

class Dataset {

    cv::Size_<unsigned> m_frame_size;

    // /local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset
    boost::filesystem::path m_dataset_path;

    // /local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset/data/stereo_flow/image_02
    boost::filesystem::path  m__directory_path_input;

    // /local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset/data/stereo_flow/flow_occ_01
    boost::filesystem::path  m__directory_path_gt_flow;

    // /local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset/data/stereo_flow/trajectroy_occ_01
    boost::filesystem::path  m__directory_path_gt_trajectory;

private:

    // /local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset/results/results_LK/flow_occ_01
    boost::filesystem::path m__directory_path_result_flow;

    // /local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset/results/results_LK/flow_occ_01
    boost::filesystem::path m__directory_path_result_trajectory_out;

    ushort m_depth;

    ushort m_cn;
public:

    Dataset() {}

    Dataset(cv::Size_<unsigned> frame_size, ushort depth, ushort cn, std::string dataset_path, std::string unterordner,
            std::string resultordner) {

        m_frame_size = frame_size;

        m_depth = depth;

        m_cn = cn;

        m_dataset_path = dataset_path;

        //data/stereo_flow/image_02/dummy.txt
        m__directory_path_input = m_dataset_path.string() + unterordner + std::string("/dummy.txt");
        //data/stereo_flow/image_02
        m__directory_path_input = m__directory_path_input.parent_path();

        //data/stereo_flow/
        m__directory_path_gt_flow = m__directory_path_input.parent_path();

        //data/stereo_flow/
        m__directory_path_gt_trajectory = m__directory_path_input.parent_path();

        /* result------------------------------------------------------------------------- */
        //results/resultordner/dummy.txt
        m__directory_path_result_flow = m_dataset_path.string() + resultordner + std::string("/dummy.txt");
        //results/resultordner
        m__directory_path_result_flow = m__directory_path_result_flow.parent_path();


        if (boost::filesystem::exists(m__directory_path_input) == 0 ) {
            boost::filesystem::create_directories(m__directory_path_input.string());
        }

        // Prepare 10 directories for 10 object trajectories
        for ( int i = 0; i < 10; i++ ) {
            boost::filesystem::path objectTrajectory = m__directory_path_gt_trajectory.string() + std::string
                    ("_object_") + std::to_string(i);

            if ( boost::filesystem::exists(objectTrajectory) == 0 ) {
                boost::filesystem::create_directories(objectTrajectory);
            }
        }

    };

    const cv::Size_<unsigned> getFrameSize() const {
        return m_frame_size;
    }

    const boost::filesystem::path getBasePath() const {
        return m_dataset_path;
    }

    const boost::filesystem::path getInputPath() const {
        return m__directory_path_input;
    }

    const boost::filesystem::path getResultPath() const {
        return m__directory_path_result_flow;
    }

    const boost::filesystem::path getGroundTruthFlowPath() const {
        return m__directory_path_gt_flow;
    }

    const ushort getDepth() const {
        return m_depth;
    }

    const ushort getChannel() const {
        return m_cn;
    }

    const ushort getMakeType() const {
        return static_cast<ushort>(CV_MAKETYPE(m_depth, m_cn));
    }

    const boost::filesystem::path getGroundTruthTrajectoryPath() const {
        return m__directory_path_gt_trajectory;
    }

    const boost::filesystem::path getResultTrajectoryPath() const {
        return m__directory_path_result_trajectory_out;
    }

};


#endif //MAIN_DATASET_H
