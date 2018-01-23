//
// Created by veikas on 11.01.18.
//

#ifndef MAIN_GROUNDTRUTH_H
#define MAIN_GROUNDTRUTH_H


#include <vires/vires_common.h>

/**
 * This class
 *  private - makes directories where synthetic images will be stored
 *  produces synthetic images
 *  private - makes directories where flow images will be stored
 *  produces flow images
 */

class GroundTruth : Framework::ViresInterface {

private:

    std::string m_dataset_path;

    boost::filesystem::path  m_base_directory_path_input_in;
    boost::filesystem::path m_base_directory_path_result_out;

    cv::Size_<unsigned> m_frame_size;

    cv::Mat m_pedesterianImage;

    //cv::FileStorage m_fs;

    std::vector<cv::Point2i> m_trajectory_1;
    // position, movement as pair


public:

    GroundTruth(std::string dataset_path, std::string unterordner, std::string resultordner);

    void generate_gt_image_and_gt_flow_vires();

    void generate_gt_image_and_gt_flow();

    void extrapolate_objects(cv::FileStorage fs, cv::Point2i pt, ushort width,
                                           ushort height, int xValue, int yValue, std::string image_path );

    void calculate_flow(const boost::filesystem::path dataset_path, const std::string image_input_sha, ALGO_TYPES algo,
    FRAME_TYPES frame_types, NOISE_TYPES noise);

    void plot(std::string resultsordner);

    void parseStartOfFrame(const double &simTime, const unsigned int &simFrame);

    void parseEndOfFrame( const double & simTime, const unsigned int & simFrame );

    void parseEntry( RDB_IMAGE_t *data, const double & simTime, const unsigned int & simFrame, const
    unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int &totalElem );

    void parseEntry( RDB_OBJECT_STATE_t *data, const double & simTime, const unsigned int & simFrame, const
    unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int &
    totalElem );

    void parseEntry( RDB_OBJECT_CFG_t *data, const double & simTime, const unsigned int & simFrame, const
    unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem );

    ~GroundTruth(){
        std::cout << "killing previous GroundTruth object\n" ;
    }


private:

    void prepare_gt_data_and_gt_flow_directories();

    void prepare_result_directories(std::string resultordner);

    void store_in_yaml(cv::FileStorage &fs, ushort currentPixelPositionX, ushort
    currentPixelPositionY, int XMovement, int YMovement);
};


#endif //MAIN_GROUNDTRUTH_H
