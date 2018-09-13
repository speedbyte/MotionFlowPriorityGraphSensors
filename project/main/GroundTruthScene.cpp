//
// Created by veikas on 27.01.18.
//

#include <iostream>
#include <boost/filesystem/operations.hpp>
#include <opencv2/core/mat.hpp>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <png++/rgb_pixel.hpp>
#include <png++/image.hpp>
#include <vires-interface/Common/viRDBIcd.h>
#include <sys/time.h>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv/cv.hpp>
#include "GroundTruthScene.h"
#include "kbhit.h"
#include "GenerateViresObjects.h"
#include "ObjectMetaData.h"
#include "Utils.h"

boost::filesystem::path GroundTruthScene::m_ground_truth_generate_path;
boost::filesystem::path GroundTruthScene::m_ground_truth_edge_path;
boost::filesystem::path GroundTruthScene::m_ground_truth_flow_path;
boost::filesystem::path GroundTruthScene::m_ground_truth_plot_path;

using namespace std::chrono;

void GroundTruthScene::prepare_directories(ushort sensor_group_index) {

    m_groundtruthpath = Dataset::m_dataset_gtpath; // data/stereo_flow

    m_ground_truth_generate_path = m_groundtruthpath.string() + "/" + m_environment;

    m_baseframepath = m_groundtruthpath.string() + "/" + "base_frame";

    m_ground_truth_framedifference_path = m_groundtruthpath.string() + "/" + "frame_difference_";

    m_ground_truth_flow_path = m_groundtruthpath.string() + "/" + "flow_occ_";

    m_ground_truth_plot_path = m_groundtruthpath.string() + "/" + "plots_";

    m_ground_truth_edge_path = m_groundtruthpath.string() + "/" + "edge_";


    char sensor_index_folder_suffix[50];
    if (Dataset::GENERATE) {
        if (m_datasetpath.string() == std::string(CPP_DATASET_PATH) || m_datasetpath.string() == std::string(VIRES_DATASET_PATH)) {

            std::cout << "prepare gt_scene directories" << std::endl;

            sprintf(sensor_index_folder_suffix, "%02d", sensor_group_index);
            std::string generate_path_sensor = m_ground_truth_generate_path.string() + "_" + sensor_index_folder_suffix;
            std::string baseframe_path_sensor = m_baseframepath.string() + "_" + sensor_index_folder_suffix;
            std::string framedifference_path_sensor = m_ground_truth_framedifference_path.string()  + sensor_index_folder_suffix;
            std::string edge_path_sensor = m_ground_truth_edge_path.string() + sensor_index_folder_suffix;
            std::string flow_path_sensor = m_ground_truth_flow_path.string() + sensor_index_folder_suffix;
            std::string plot_path_sensor = m_ground_truth_plot_path.string() + sensor_index_folder_suffix;


            if (boost::filesystem::exists(generate_path_sensor)) {
                system(("rm -rf " + generate_path_sensor).c_str());
            }
            boost::filesystem::create_directories(generate_path_sensor);

            if (boost::filesystem::exists(framedifference_path_sensor)) {
                system(("rm -rf " + framedifference_path_sensor).c_str());
            }
            boost::filesystem::create_directories(framedifference_path_sensor);

            if (boost::filesystem::exists(edge_path_sensor)) {
                system(("rm -rf " + edge_path_sensor).c_str());
            }
            boost::filesystem::create_directories(edge_path_sensor);

            if (boost::filesystem::exists(flow_path_sensor)) {
                system(("rm -rf " + flow_path_sensor).c_str());
            }
            boost::filesystem::create_directories(flow_path_sensor);

            if (boost::filesystem::exists(plot_path_sensor)) {
                system(("rm -rf " + plot_path_sensor).c_str());
            }
            boost::filesystem::create_directories(plot_path_sensor);

            /*char char_dir_append[20];
            boost::filesystem::path path;
            for (int i = 0; i < m_list_gt_objects.size(); i++) {

                sprintf(char_dir_append, "%02d", i);
                m_position_object_path = generatepath_sensor + "position_object_";
                path = m_position_object_path.string() + char_dir_append;
                boost::filesystem::create_directories(path);
            }
            */

        }
    }
}


void GroundTruthScene::generate_bird_view() {
    // the bird view needs the range information of each object
    // Assuming the camera is mounted on the floor.
    /*
        for ( auto position_index = 0;  position_index <  MAX_ITERATION_RESULTS; position_index++ ) {

            cv::Mat birdview_frame(Dataset::m_frame_size, CV_32FC1);
            for ( auto object_index= 0; object_index < get_ptr_customObjectMetaDataList().at(0).size(); object_index++ ) {
                birdview_frame.at(m_list_gt_objects.at(object_index).get_object_base_point_displacement().at(position_index).first.x, m_list_gt_objects.at(object_index).get_object_range().at(position_index)) = 100;
                cv::Mat roi_objects;
                roi_objects = birdview_frame.rowRange(m_list_gt_objects.at(object_index).get_object_range().at(position_index), m_list_gt_objects.at(object_index).get_object_dimension().at(position_index).z_offset )
                        .colRange(m_list_gt_objects.at(object_index).get_object_range().at(position_index), m_list_gt_objects.at(object_index).get_object_dimension().at(position_index).x_offset);

            }
            cv::imwrite("filename", birdview_frame);
            // time to collide with the object.

        }
        */
}





/**
 * @brief calcBBFrom3DPosition
 *
 * For the camera sensor the GT data from VTD is slightly shifted. Therefore we try to get better
 * bounding boxes by calculating the bounding box on the image from the 3D data delivered by the perfect
 * sensor.
 * @param screen_width      width of VTD display
 * @param screen_height     height of VTD display
 * @param cam_pos           position of camera relative to vehicle point of origin
 * @param fov_v             vertical fov of the VTD camera
 * @param pixSize           pixel size
 */

