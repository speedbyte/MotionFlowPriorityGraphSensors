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

    boost::filesystem::path m_dataset_path;

    boost::filesystem::path  m_base_directory_path_input_in;

    boost::filesystem::path  m_base_directory_path_gt_flow;

    boost::filesystem::path m_base_directory_path_result_out;

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

        m_base_directory_path_input_in = m_dataset_path.string() + unterordner + std::string("/dummy.txt");
        //data/stereo_flow/
        m_base_directory_path_input_in = m_base_directory_path_input_in.parent_path();

        m_base_directory_path_gt_flow = m_base_directory_path_input_in.parent_path();

        m_base_directory_path_result_out = m_dataset_path.string() + resultordner + std::string("/dummy.txt");
        //results
        m_base_directory_path_result_out = m_base_directory_path_result_out.parent_path();

        if (boost::filesystem::exists(m_base_directory_path_input_in) == 0 ) {
            boost::filesystem::create_directories(m_base_directory_path_input_in.string());
        }

    };

    const cv::Size_<unsigned> getFrameSize() const {
        return m_frame_size;
    }

    const boost::filesystem::path getBasePath() const {
        return m_dataset_path;
    }

    const boost::filesystem::path getInputPath() const {
        return m_base_directory_path_input_in;
    }

    const boost::filesystem::path getResultPath() const {
        return m_base_directory_path_result_out;
    }

    const boost::filesystem::path getGroundTruthFlowPath() const {
        return m_base_directory_path_gt_flow;
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
};


#endif //MAIN_DATASET_H
