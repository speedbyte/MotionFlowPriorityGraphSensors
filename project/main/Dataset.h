//
// Created by veikas on 26.01.18.
//

#ifndef MAIN_DATASET_H
#define MAIN_DATASET_H


#include <opencv2/core/types.hpp>
#include <boost/filesystem/path.hpp>

class Dataset {

    cv::Size_<unsigned> m_frame_size;

    boost::filesystem::path m_dataset_path;

    boost::filesystem::path  m_base_directory_path_input_in;

    boost::filesystem::path m_base_directory_path_result_out;

public:

    Dataset() {}

    Dataset(cv::Size_<unsigned> frame_size, std::string dataset_path, std::string unterordner, std::string
    resultordner);

    const boost::filesystem::path getBasePath() const ;
    const boost::filesystem::path getInputPath() const ;
    const boost::filesystem::path getResultPath() const ;
    const cv::Size_<unsigned> getFrameSize() const ;

};


#endif //MAIN_DATASET_H
