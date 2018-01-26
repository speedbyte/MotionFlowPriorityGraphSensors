//
// Created by veikas on 26.01.18.
//

#include "Dataset.h"


Dataset::Dataset(cv::Size_<unsigned> frame_size, std::string dataset_path, std::string unterordner, std::string
resultordner) {

    m_frame_size = frame_size;

    m_dataset_path = dataset_path;

    m_base_directory_path_input_in = m_dataset_path.string() + unterordner + std::string("dummy.txt");
    //data/stereo_flow/
    m_base_directory_path_input_in = m_base_directory_path_input_in.parent_path();

    m_base_directory_path_result_out = m_dataset_path.string() + resultordner + std::string("dummy.txt");
    //results
    m_base_directory_path_result_out = m_base_directory_path_result_out.parent_path();

}

const cv::Size_<unsigned> Dataset::getFrameSize() {
    return m_frame_size;
}

const boost::filesystem::path Dataset::getBasePath() {
    return m_dataset_path;
}

const boost::filesystem::path Dataset::getInputPath() {
    return m_base_directory_path_input_in;
}

const boost::filesystem::path Dataset::getResultPath() {
    return m_base_directory_path_result_out;
}

