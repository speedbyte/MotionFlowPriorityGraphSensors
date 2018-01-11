//
// Created by veikas on 11.01.18.
//

#ifndef MAIN_GROUNDTRUTH_H
#define MAIN_GROUNDTRUTH_H

class GroundTruth {

private:
    boost::filesystem::path m_dataset_path;
    std::string m_dataordner;
    boost::filesystem::path  m_base_directory_path_image_out;
    boost::filesystem::path  m_base_directory_path_flow_out;
    boost::filesystem::path m_base_directory_path_video_out;
    std::string m_gt_flow_matrix_str;
    cv::Size_<unsigned> m_frame_size;

    std::vector<ushort> m_xPos, m_yPos;


public:

    GroundTruth(boost::filesystem::path dataset_path, std::string dataordner);

    void prepare_gt_dataandflow_directories(int frame_skip);

    void generate_gt_image_and_gt_flow_vires();

    void generate_gt_image_and_gt_flow();

    void test_kitti_original();
};


#endif //MAIN_GROUNDTRUTH_H
