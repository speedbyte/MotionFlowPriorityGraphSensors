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

    cv::Mat m_pedesterianImage;
    cv::Mat m_absolutePixelLocation;

    std::string m_gt_image_path;
    std::string m_gt_flow_path;

    cv::FileStorage m_fs;

    cv::Point2i m_position, m_movement;

    std::vector<cv::Point2i> m_position_matrix;
    std::vector<std::pair<cv::Point2i, cv::Point2i> > m_flow_matrix;


public:

    GroundTruth(boost::filesystem::path dataset_path, std::string dataordner);

    void prepare_gt_dataandflow_directories();

    void generate_gt_image_and_gt_flow_vires();

    void generate_gt_image_and_gt_flow();

    void test_kitti_original();

    void extrapolate_objects( cv::Point2i pt, ushort width,
                                           ushort height, int xValue, int yValue, std::string image_path );


private:
    void store_in_yaml(const std::string &temp_flow_path, int frame_count, ushort currentPixelPositionX, ushort
    currentPixelPositionY, int XMovement, int YMovement);
};


#endif //MAIN_GROUNDTRUTH_H
