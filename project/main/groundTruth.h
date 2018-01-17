//
// Created by veikas on 11.01.18.
//

#ifndef MAIN_GROUNDTRUTH_H
#define MAIN_GROUNDTRUTH_H

/**
 * This class
 *  private - makes directories where synthetic images will be stored
 *  produces synthetic images
 *  private - makes directories where flow images will be stored
 *  produces flow images
 */

class GroundTruth {

private:

    std::string m_dataset_path;

    boost::filesystem::path  m_base_directory_path_input_in;
    boost::filesystem::path m_base_directory_path_result_out;

    cv::Size_<unsigned> m_frame_size;

    cv::Mat m_pedesterianImage;

    std::string m_gt_image_path;
    std::string m_gt_flow_path;
    std::string m_result_flow_path;

    cv::FileStorage m_fs;

    cv::Point2i m_position, m_movement;

    std::vector<cv::Point2i> m_position_matrix;
    std::vector<std::pair<cv::Point2i, cv::Point2i> > m_flow_matrix_gt, m_flow_matrix_result;


public:

    GroundTruth(std::string dataset_path, std::string unterordner, std::string resultordner);

    void generate_gt_image_and_gt_flow_vires();

    void generate_gt_image_and_gt_flow();

    void test_kitti_original();

    void extrapolate_objects( cv::Point2i pt, ushort width,
                                           ushort height, int xValue, int yValue, std::string image_path );

    void calculate_flow(const boost::filesystem::path dataset_path, const std::string image_input_sha, ALGO_TYPES algo,
    FRAME_TYPES frame_types, NOISE_TYPES noise);

    void LK();



private:

    void prepare_gt_data_and_gt_flow_directories();

    void prepare_result_directories(std::string resultordner);

    void store_in_yaml(const std::string &temp_flow_path, int frame_count, ushort currentPixelPositionX, ushort
    currentPixelPositionY, int XMovement, int YMovement);
};


#endif //MAIN_GROUNDTRUTH_H
