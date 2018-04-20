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
#include "ViresObjects.h"
#include "ObjectMetaData.h"
#include "Utils.h"

using namespace std::chrono;

void GroundTruthScene::visualiseBoundingBox(void) {

    std::cout << "visualise boudning box at " << m_generatepath.string() + "stencil/" << std::endl;

    char file_name_image[50], file_name_image_output[50];

    cv::Mat image_data_and_shape;

    const ushort max_frame_skip = 1; // image is generated only once irrespective of skips.
    cv::Mat tempGroundTruthImage(Dataset::getFrameSize(), CV_8UC3);

    for (int frame_skip = 1; frame_skip <= max_frame_skip; frame_skip++) {

        for (ushort frame_count = 0; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_IMAGES; frame_count++) {
            //ushort frame_count = 3;

            std::cout << "frame_count " << frame_count << std::endl;

            std::string output_image_file_with_path;
            /*---------------------------------------------------------------------------------*/
            tempGroundTruthImage = cv::Scalar::all(255);

            sprintf(file_name_image, "000%03d_10.png", frame_count * frame_skip);
            std::string input_image_file_with_path = m_generatepath.string() + file_name_image;

            sprintf(file_name_image_output, "000%03d_10_bb.png", frame_count * frame_skip);
            output_image_file_with_path = m_generatepath.string() + "stencil/" + file_name_image_output;

            cv::Mat tempGroundTruthImageBase = cv::imread(input_image_file_with_path, CV_LOAD_IMAGE_ANYCOLOR);
            tempGroundTruthImage = cv::Scalar::all(255);
            tempGroundTruthImage = tempGroundTruthImageBase.clone();

            for (unsigned obj_index = 0; obj_index < m_ptr_customObjectMetaDataList.size(); obj_index++) {

                if ((m_ptr_customObjectMetaDataList.at(0)->getAll().at(frame_count).occluded == false)
                        ) {

                    cv::Rect boundingbox = cv::Rect(
                            cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_object_location_px.location_x_m),
                            cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_object_location_px.location_y_m),
                            cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_object_dimensions_px.dim_width_m),
                            cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_object_dimensions_px.dim_height_m));

                    cv::rectangle(tempGroundTruthImage, boundingbox, cv::Scalar(0, 255, 0), 1, 8, 0);

                    std::vector<cv::Point2f> box = {
                            m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_lower_bottom_px,
                            m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_lower_left_px,
                            m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_lower_top_px,
                            m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_lower_right_px,
                            m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_higher_bottom_px,
                            m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_higher_left_px,
                            m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_higher_top_px,
                            m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_higher_right_px
                    };

                    cv::Rect boundingbox2 = cv::Rect(
                            cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_higher_left_px.x),
                            cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_higher_left_px.y),
                            cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_object_dimensions_px.dim_width_m),
                            cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_object_dimensions_px.dim_height_m));

                    cv::rectangle(tempGroundTruthImage, boundingbox2, cv::Scalar(0, 0, 255), 1, 8, 0);

                    for ( auto i = 0; i < 1; i++ ) {
                        std::cout << box.at(i) << std::endl;
                        cv::circle(tempGroundTruthImage, box.at(3), 2, cv::Scalar(0, 0, 255), 3);
                    }
                }
            }


            cv::namedWindow("BB", CV_WINDOW_AUTOSIZE);
            cv::imshow("BB", tempGroundTruthImage);
            cv::waitKey(0);
            //cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
            /*---------------------------------------------------------------------------------*/
        }
    }
    cv::destroyAllWindows();
}

void GroundTruthScene::prepare_directories() {

    m_groundtruthpath = Dataset::getGroundTruthPath(); // data/stereo_flow

    m_generatepath = m_groundtruthpath.string() + "/" + m_environment + "/";

    if (m_regenerate_yaml_file) {
        if (!m_datasetpath.string().compare(CPP_DATASET_PATH) || !m_datasetpath.string().compare(VIRES_DATASET_PATH)) {

            std::cout << "prepare gt_scene directories" << std::endl;

            if (boost::filesystem::exists(m_generatepath)) {
                system(("rm -rf " + m_generatepath.string()).c_str());
            }
            boost::filesystem::create_directories(m_generatepath);

            char char_dir_append[20];
            boost::filesystem::path path;

            for (int i = 0; i < m_list_gt_objects.size(); i++) {

                sprintf(char_dir_append, "%02d", i);
                m_position_obj_path = m_generatepath.string() + "position_obj_";
                path = m_position_obj_path.string() + char_dir_append;
                boost::filesystem::create_directories(path);

            }
        }
    } else {
        // post processing step. At the moment I dont need this.
        boost::filesystem::path bbox_dir = m_generatepath.string() + "bounding_box/";
        if (boost::filesystem::exists(m_generatepath)) {
            system(("rm -rf " + bbox_dir.string()).c_str());
        }
        //boost::filesystem::create_directories(bbox_dir);
    }

}

void GroundTruthScene::writePositionInYaml(std::string suffix) {

    cv::FileStorage write_fs;
    write_fs.open("../position_" + suffix + ".yml", cv::FileStorage::WRITE);

    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

        std::cout << "write yaml file for frame_skip  " << (frame_skip - 1) << std::endl;

        char temp_str_fs[20];
        sprintf(temp_str_fs, "frame_skip_%03d", frame_skip);
        //write_fs << temp_str_fs << "[";

        unsigned long FRAME_COUNT = MAX_ITERATION_GT_SCENE_GENERATION_DYNAMIC;
        assert(FRAME_COUNT > 0);

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
            char temp_str_fc[20];
            sprintf(temp_str_fc, "frame_count_%03d", frame_count);
            write_fs << temp_str_fc << "[";
            for (int obj_index = 0; obj_index < m_ptr_customObjectMetaDataList.size(); obj_index++) {
                write_fs

                        << "{:" << "name" << m_ptr_customObjectMetaDataList.at(obj_index)->getObjectName()

                        << "visMask" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).visMask
                        << "x_camera" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_location_px.location_x_m
                        << "y_camera" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_location_px.location_y_m
                        << "z_camera" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_location_px.location_z_m

                        << "x_inertial" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_location_inertial_m.location_x_m
                        << "y_inertial" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_location_inertial_m.location_y_m
                        << "z_inertial" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_location_inertial_m.location_z_m

                        << "x_usk" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_location_m.location_x_m
                        << "y_usk" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_location_m.location_y_m
                        << "z_usk" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_location_m.location_z_m

                        << "dim_x_camera" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_dimensions_px.dim_width_m
                        << "dim_y_camera" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_dimensions_px.dim_height_m

                        << "dim_x_realworld" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_realworld_dim_m.dim_length_m
                        << "dim_y_realworld" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_realworld_dim_m.dim_width_m
                        << "dim_z_realworld" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_realworld_dim_m.dim_height_m

                        << "speed_x_inertial"
                        << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_object_speed_inertial.x
                        << "speed_y_inertial"
                        << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_object_speed_inertial.y
                        << "speed_z_inertial"
                        << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_object_speed_inertial.z

                        << "speed_x" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_object_speed.x
                        << "speed_y" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_object_speed.y

                        << "off_x"
                        << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_object_offset_m.offset_x
                        << "off_y"
                        << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_object_offset_m.offset_y
                        << "off_z"
                        << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_object_offset_m.offset_z

                        << "h_inertial" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_rotation_inertial_rad.rotation_rz_yaw_rad
                        << "p_inertial" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_rotation_inertial_rad.rotation_ry_pitch_rad
                        << "r_inertial" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_rotation_inertial_rad.rotation_rx_roll_rad

                        << "h_usk" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_rotation_rad.rotation_rz_yaw_rad
                        << "p_usk" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_rotation_rad.rotation_ry_pitch_rad
                        << "r_usk" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_rotation_rad.rotation_rx_roll_rad

                        << "dist_cam_to_obj" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_distances.sensor_to_obj
                        << "total_distance_covered" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_distances.total_distance_covered

                        << "occ_px" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_occlusion.occlusion_px
                        << "occ_usk" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_occlusion.occlusion_usk
                        << "occ_inertial" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_occlusion.occlusion_inertial


                        << "}";  //dont close the brace yet
            }
            for (int sen_index = 0; sen_index < m_ptr_customSensorMetaDataList.size(); sen_index++) {
                write_fs

                        << "{:" << "name" << m_ptr_customSensorMetaDataList.at(sen_index)->getSensorName()

                        << "visible" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(frame_count).visMask

                        << "x_carrier" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                        frame_count).m_sensor_location_carrier_m.location_x_m
                        << "y_carrier" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                        frame_count).m_sensor_location_carrier_m.location_y_m
                        << "z_carrier" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                        frame_count).m_sensor_location_carrier_m.location_z_m

                        //<< "x_sensor" <<  m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(frame_count).m_sensor_location_m.location_x_m
                        //<< "y_sensor" <<  m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(frame_count).m_sensor_location_m.location_y_m
                        //<< "z_sensor" <<  m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(frame_count).m_sensor_location_m.location_z_m

                        << "h_carrier" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                        frame_count).m_sensor_rotation_carrier_rad.rotation_rz_yaw_rad
                        << "p_carrier" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                        frame_count).m_sensor_rotation_carrier_rad.rotation_ry_pitch_rad
                        << "r_carrier" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                        frame_count).m_sensor_rotation_carrier_rad.rotation_rx_roll_rad

                        << "h_sensor" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                        frame_count).m_sensor_rotation_rad.rotation_rz_yaw_rad
                        << "p_sensor" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                        frame_count).m_sensor_rotation_rad.rotation_ry_pitch_rad
                        << "r_sensor" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                        frame_count).m_sensor_rotation_rad.rotation_rx_roll_rad

                        << "off_x_sensor"
                        << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(frame_count).m_sensor_offset_m.offset_x
                        << "off_y_sensor"
                        << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(frame_count).m_sensor_offset_m.offset_y
                        << "off_z_sensor"
                        << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(frame_count).m_sensor_offset_m.offset_z

                        << "fov_horizontal"
                        << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(frame_count).m_sensor_fov_rad.horizontal
                        << "fov_vertical"
                        << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(frame_count).m_sensor_fov_rad.vertical
                        << "fov_horizontal_off" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                        frame_count).m_sensor_fov_rad.horizontal_offset
                        << "fov_vertical_off" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                        frame_count).m_sensor_fov_rad.vertical_offset

                        << "}";
            }

            write_fs << "]";
        }
    }
    //write_fs << "]";
    write_fs.release();
}

void GroundTruthScene::readPositionFromFile(std::string positionFileName) {

    cv::FileStorage fs(positionFileName, cv::FileStorage::READ);

    cv::Point2f speed_usk, speed_inertial;
    cv::Point2f dimension_pixel, sensor_fov;
    cv::Point3f offset, position_inertial, position_usk, position_pixel, dimension_realworld;
    cv::Point3f position_inertial_pre, position_usk_pre, position_pixel_pre;
    float totalDistanceTravelled;

    cv::Point3f orientation_inertial, orientation_usk;


    cv::FileNode file_node;
    cv::FileNodeIterator file_node_iterator_begin, file_node_iterator_end, file_node_iterator;

    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

        //std::string temp_str = "frame_skip" + frame_skip;
        char temp_str_fs[20];
        sprintf(temp_str_fs, "frame_skip_%03d", frame_skip);
        std::cout << "read yaml file for frame_skip " << (frame_skip - 1) << std::endl;
        //unsigned long FRAME_COUNT = m_list_gt_objects.at(0).get_obj_extrapolated_shape_pixel_point_pixel_displacement().at(frame_skip-1).size();
        unsigned long FRAME_COUNT = MAX_ITERATION_RESULTS;
        assert(FRAME_COUNT > 0);

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

            std::cout << frame_count << std::endl;
            char temp_str_fc[20];
            sprintf(temp_str_fc, "frame_count_%03d", frame_count);
            file_node = fs[temp_str_fc];
            if (file_node.isNone() || file_node.empty()) {
                std::cout << temp_str_fc << " cannot be found" << std::endl;
            } else {

                //std::cout << file_node.size() << " found" << std::endl;
                file_node_iterator_begin = file_node.begin();
                file_node_iterator_end = file_node.end();

                for (file_node_iterator = file_node_iterator_begin; file_node_iterator != file_node_iterator_end;
                     file_node_iterator++) {

                    if (((*file_node_iterator)["name"].string()).find("Sensor") == std::string::npos) {

                        if (m_mapObjectNameToObjectMetaData.count((*file_node_iterator)["name"].string()) == 0) {
                            m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(m_objectCount));
                            m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()] = m_ptr_customObjectMetaDataList.at(
                                    m_objectCount);
                            m_ptr_customObjectMetaDataList.at(m_objectCount)->setObjectName(
                                    (*file_node_iterator)["name"].string());
                            Rectangle rectangle(Dataset::getFrameSize().width,
                                                Dataset::getFrameSize().height); // width, height
                            m_ptr_customObjectMetaDataList.at(m_objectCount)->setObjectShape(rectangle);
                            m_ptr_customObjectMetaDataList.at(m_objectCount)->setStartPoint(0);
                            m_objectCount += 1;
                        }

                        std::cout << (*file_node_iterator)["name"].string() << " "
                                  << (double) (*file_node_iterator)["x_camera"] << " "
                                  << (double) (*file_node_iterator)["y_camera"] << std::endl;

                        position_pixel = cv::Point3f((double) (*file_node_iterator)["x_camera"],
                                                     (double) (*file_node_iterator)["y_camera"],
                                                     (double) (*file_node_iterator)["z_camera"]);

                        position_inertial = cv::Point3f((double) (*file_node_iterator)["x_inertial"],
                                                        (double) (*file_node_iterator)["y_inertial"],
                                                        (double) (*file_node_iterator)["z_inertial"]);

                        position_usk = cv::Point3f((double) (*file_node_iterator)["x_usk"],
                                                   (double) (*file_node_iterator)["y_usk"],
                                                   (double) (*file_node_iterator)["z_usk"]);

                        dimension_pixel = cv::Point2f((int) (*file_node_iterator)["dim_x_camera"],
                                                      (int) (*file_node_iterator)["dim_y_camera"]);

                        dimension_realworld = cv::Point3f((double) (*file_node_iterator)["dim_x_realworld"],
                                                          (double) (*file_node_iterator)["dim_y_realworld"],
                                                          (double) (*file_node_iterator)["dim_z_realworld"]);

                        speed_inertial = cv::Point2f((int) (*file_node_iterator)["speed_inertial"],
                                                     (int) (*file_node_iterator)["speed_inertial"]);

                        speed_usk = cv::Point2f((int) (*file_node_iterator)["speed_x"],
                                                (int) (*file_node_iterator)["speed_y"]);

                        offset = cv::Point3f((double) (*file_node_iterator)["off_x"],
                                             (double) (*file_node_iterator)["off_y"],
                                             (double) (*file_node_iterator)["off_z"]);

                        orientation_inertial = cv::Point3f((double) (*file_node_iterator)["h_inertial"],
                                                           (double) (*file_node_iterator)["p_inertial"],
                                                           (double) (*file_node_iterator)["r_inertial"]);

                        orientation_usk = cv::Point3f((double) (*file_node_iterator)["h_usk"],
                                                      (double) (*file_node_iterator)["p_usk"],
                                                      (double) (*file_node_iterator)["r_usk"]);

                        totalDistanceTravelled = (float) (*file_node_iterator)["total_distance_covered"];

                        (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberCameraSensor(
                                frame_count, position_pixel, offset, dimension_pixel, totalDistanceTravelled);
                        (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberPerfectSensor(
                                frame_count, position_usk, orientation_usk, dimension_realworld, speed_usk,
                                totalDistanceTravelled);
                        (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberPerfectSensorInertial(
                                frame_count, position_inertial, orientation_inertial, dimension_realworld,
                                speed_inertial, totalDistanceTravelled);

                        (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberVisibility(
                                frame_count, (int) (*file_node_iterator)["visMask"]);

                        (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberOcclusionWindow(
                                frame_count, (int) (*file_node_iterator)["occ_px"]);

                        (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberOcclusionUsk(
                                frame_count, (int) (*file_node_iterator)["occ_usk"]);

                        (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberOcclusionInertial(
                                frame_count, (int) (*file_node_iterator)["occ_inertial"]);

                        position_inertial_pre = position_inertial;
                        position_pixel_pre = position_pixel;
                        position_usk_pre = position_usk;

                    } else {
                        if (m_mapSensorNameToSensorMetaData.count((*file_node_iterator)["name"].string()) == 0) {
                            m_ptr_customSensorMetaDataList.push_back(&sensorMetaDataList.at(m_sensorCount));
                            m_mapSensorNameToSensorMetaData[(*file_node_iterator)["name"].string()] = m_ptr_customSensorMetaDataList.at(
                                    m_sensorCount);
                            m_ptr_customSensorMetaDataList.at(m_sensorCount)->setSensorName(
                                    (*file_node_iterator)["name"].string());
                            m_ptr_customSensorMetaDataList.at(m_sensorCount)->setStartPoint(0);
                            m_sensorCount += 1;
                        }

                        std::cout << (*file_node_iterator)["name"].string() << " "
                                  << (double) (*file_node_iterator)["x_carrier"] << " "
                                  << (double) (*file_node_iterator)["y_carrier"] << std::endl;

                        position_inertial = cv::Point3f((double) (*file_node_iterator)["x_carrier"],
                                                        (double) (*file_node_iterator)["y_carrier"],
                                                        (double) (*file_node_iterator)["z_carrier"]);

                        offset = cv::Point3f((double) (*file_node_iterator)["off_x_sensor"],
                                             (double) (*file_node_iterator)["off_y_sensor"],
                                             (double) (*file_node_iterator)["off_z_sensor"]);

                        orientation_inertial = cv::Point3f((double) (*file_node_iterator)["h_carrier"],
                                                           (double) (*file_node_iterator)["p_carrier"],
                                                           (double) (*file_node_iterator)["r_carrier"]);

                        orientation_usk = cv::Point3f((double) (*file_node_iterator)["h_sensor"],
                                                      (double) (*file_node_iterator)["p_sensor"],
                                                      (double) (*file_node_iterator)["r_sensor"]);

                        sensor_fov = cv::Point2f((double) (*file_node_iterator)["fov_horizontal"],
                                                 (double) (*file_node_iterator)["fov_vertical"]);

                        (m_mapSensorNameToSensorMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberSensorState(
                                frame_count, position_inertial, orientation_inertial, orientation_usk, offset,
                                sensor_fov);

                        (m_mapSensorNameToSensorMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberVisibility(
                                frame_count, (int) (*file_node_iterator)["visible"]);
                    }
                }
            }
        }
    }
    fs.release();
}


void GroundTruthScene::startEvaluating(std::string dataset, Noise noise) {


    if (m_regenerate_yaml_file) {
        writePositionInYaml(dataset);
    }

    calcBBFrom3DPosition();
    //visualiseBoundingBox();


    for (auto obj_index = 0; obj_index < m_ptr_customObjectMetaDataList.size(); obj_index++) {

        GroundTruthObjects gt_obj(m_ptr_customObjectMetaDataList.at(obj_index)->getObjectShape(),
                                  m_ptr_customObjectMetaDataList.at(obj_index)->getObjectStartPoint(), noise,
                                  m_ptr_customObjectMetaDataList.at(obj_index)->getObjectName());
        m_list_gt_objects.push_back(gt_obj);
        m_list_gt_objects.at(obj_index).beginGroundTruthGeneration(*m_ptr_customObjectMetaDataList.at(obj_index));

    }

    for (auto obj_index = 0; obj_index < m_ptr_customSensorMetaDataList.size(); obj_index++) {

        Sensors gt_sen(*m_ptr_customSensorMetaDataList.at(obj_index),
                       m_ptr_customSensorMetaDataList.at(obj_index)->getSensorStartPoint(), &noise,
                       m_ptr_customSensorMetaDataList.at(obj_index)->getSensorName());
        m_list_gt_sensors.push_back(gt_sen);
        m_list_gt_sensors.at(obj_index).beginGroundTruthGeneration(*m_ptr_customSensorMetaDataList.at(obj_index));
    }

}

void GroundTruthSceneInternal::generate_gt_scene(void) {


    /*
    cv::RNG rng(-1);
    for ( unsigned i = 0 ; i < MAX_ITERATION_THETA; i++ ) {
        float a        = (float) rng.uniform(100., 1000.);
        float b        = (float) rng.uniform(100., 300.);
        cv::Point2f points(a,b);
    }
*/


    ColorfulNoise colorfulNoise;

    if (m_environment == "none") {


        if (!m_regenerate_yaml_file) { // dont generate, just read

            readPositionFromFile("../position_cpp.yml");

            ushort map_pair_count = 0;
            for (const auto &myPair : m_mapObjectNameToObjectMetaData) {
                std::cout << myPair.first << "\n";
                map_pair_count++;
            }
        } else { // genreate yaml file
            boost::filesystem::remove("../position_cpp.yml");

            Rectangle rectangle(Dataset::getFrameSize().width, Dataset::getFrameSize().height); // width, height

            Achterbahn achterbahn;

            achterbahn = Achterbahn(rectangle, "rectangle_long", 60);
            achterbahn.process(Dataset::getFrameSize());
            objectMetaDataList.at(0) = achterbahn;
            m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(0));

            achterbahn = Achterbahn(rectangle, "random_object", 120);
            achterbahn.process(Dataset::getFrameSize());
            objectMetaDataList.at(1) = achterbahn;
            m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(1));

        }

        startEvaluating("cpp", colorfulNoise);

    }

    cv::Mat tempGroundTruthImage, tempGroundTruthImageBase;
    tempGroundTruthImage.create(Dataset::getFrameSize(), CV_32FC3);
    assert(tempGroundTruthImage.channels() == 3);

    cv::Mat tempGroundTruthPosition;
    tempGroundTruthPosition.create(Dataset::getFrameSize(), CV_32FC3);
    assert(tempGroundTruthPosition.channels() == 3);
    tempGroundTruthPosition = cv::Scalar::all(255);

    cv::Mat tempGroundTruthPosition_2;
    tempGroundTruthPosition_2.create(Dataset::getFrameSize(), CV_32FC3);
    assert(tempGroundTruthPosition_2.channels() == 3);
    tempGroundTruthPosition_2 = cv::Scalar::all(255);


    std::map<std::string, double> time_map = {{"generate_single_scene_image", 0},
                                              {"generate_all_scene_image",    0}};

    std::cout << "generate_gt_scene at " << m_groundtruthpath.string() << std::endl;

    char file_name_image[50];

    cv::Mat image_data_and_shape;
    cv::Mat positionShape;

    const ushort frame_skip = 1; // image is generated only once irrespective of skips.

    auto tic_all = steady_clock::now();

    // apply black noise in case of night
    if (m_environment == "night") {
        BlackNoise noise;
        ObjectImageShapeData newCanvas = m_canvas.getCanvasShapeAndData();
        newCanvas.applyNoise(&noise);
        tempGroundTruthImageBase = newCanvas.get().clone();
    } else {
        tempGroundTruthImageBase = m_canvas.getCanvasShapeAndData().get().clone();
    }

    for (ushort frame_count = 0; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_IMAGES; frame_count++) {

        auto tic = steady_clock::now();

        sprintf(file_name_image, "000%03d_10.png", frame_count * frame_skip);
        std::string input_image_file_with_path = m_generatepath.string() + file_name_image;


        //draw new ground truth image.
        tempGroundTruthImage = tempGroundTruthImageBase.clone();

        char frame_skip_folder_suffix[50];

        for (unsigned obj_index = 0; obj_index < m_ptr_customObjectMetaDataList.size(); obj_index++) {

            sprintf(frame_skip_folder_suffix, "%02d", m_list_gt_objects.at(obj_index).getObjectId());
            std::string position_image_file_with_path = m_position_obj_path.string() +
                                                        frame_skip_folder_suffix + "/" + file_name_image;

            image_data_and_shape = m_list_gt_objects.at(obj_index).getImageShapeAndData().get().clone();
            image_data_and_shape = image_data_and_shape.rowRange(0, cvRound(m_list_gt_objects.at(
                    obj_index).getExtrapolatedGroundTruthDetails().at(frame_skip - 1).at(
                    frame_count).m_object_dimensions_px.dim_height_m)).colRange(0, cvRound(m_list_gt_objects.at(
                    obj_index).getExtrapolatedGroundTruthDetails().at(frame_skip - 1).at(
                    frame_count).m_object_dimensions_px.dim_width_m));
            positionShape = m_list_gt_objects.at(obj_index).getImageShapeAndData().get().clone();
            positionShape = positionShape.rowRange(0, cvRound(m_list_gt_objects.at(
                    obj_index).getExtrapolatedGroundTruthDetails().at(frame_skip - 1).at(
                    frame_count).m_object_dimensions_px.dim_height_m)).colRange(0, cvRound(m_list_gt_objects.at(
                    obj_index).getExtrapolatedGroundTruthDetails().at(frame_skip - 1).at(
                    frame_count).m_object_dimensions_px.dim_width_m));

            if ((m_ptr_customObjectMetaDataList.at(0)->getAll().at(frame_count).occluded == false)
                    ) {

                image_data_and_shape.copyTo(tempGroundTruthImage(
                        cv::Rect(cvRound(m_list_gt_objects.at(obj_index).get_obj_base_pixel_position_pixel_displacement().at(
                                frame_count).first.x),
                                 cvRound(m_list_gt_objects.at(obj_index).get_obj_base_pixel_position_pixel_displacement().at(
                                         frame_count).first.y),
                                 cvRound(m_ptr_customObjectMetaDataList.at(0)->getAll().at(frame_count).m_object_dimensions_px.dim_width_m),
                                 cvRound(m_ptr_customObjectMetaDataList.at(0)->getAll().at(frame_count).m_object_dimensions_px.dim_height_m))));

            }
        }

        cv::imwrite(input_image_file_with_path, tempGroundTruthImage);
        auto toc = steady_clock::now();
        time_map["generate_single_scene_image"] = duration_cast<milliseconds>(toc - tic).count();

    }

    auto toc_all = steady_clock::now();
    time_map["generate_all_scene_image"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth scene generation time - " << time_map["generate_all_scene_image"] << "ms" << std::endl;

}

void GroundTruthScene::generate_bird_view() {
    // the bird view needs the range information of each object
    // Assuming the camera is mounted on the floor.
    /*
        for ( auto position_index = 0;  position_index <  MAX_ITERATION_RESULTS; position_index++ ) {

            cv::Mat birdview_frame(Dataset::getFrameSize(), CV_32FC1);
            for ( auto object_index= 0; object_index < m_ptr_customObjectMetaDataList.size(); object_index++ ) {
                birdview_frame.at(m_list_gt_objects.at(object_index).get_obj_base_pixel_position_pixel_displacement().at(position_index).first.x, m_list_gt_objects.at(object_index).get_obj_range().at(position_index)) = 100;
                cv::Mat roi_objects;
                roi_objects = birdview_frame.rowRange(m_list_gt_objects.at(object_index).get_obj_range().at(position_index), m_list_gt_objects.at(object_index).get_obj_dimension().at(position_index).z_offset )
                        .colRange(m_list_gt_objects.at(object_index).get_obj_range().at(position_index), m_list_gt_objects.at(object_index).get_obj_dimension().at(position_index).x_offset);

            }
            cv::imwrite("filename", birdview_frame);
            // time to collide with the object.

        }
        */
}


void GroundTruthScene::calcBBFrom3DPosition() {

    cv::FileStorage write_fs;
    char file_name_image[50], file_name_image_output[50];

    cv::Mat image_data_and_shape;

    const ushort max_frame_skip = 1; // image is generated only once irrespective of skips.
    cv::Mat tempGroundTruthImage(Dataset::getFrameSize(), CV_8UC3);

    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

        char temp_str_fs[20];
        sprintf(temp_str_fs, "frame_skip_%03d", frame_skip);
        //write_fs << temp_str_fs << "[";

        unsigned long FRAME_COUNT = MAX_ITERATION_RESULTS;
        assert(FRAME_COUNT > 0);

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

            tempGroundTruthImage = cv::Scalar::all(255);

            sprintf(file_name_image, "000%03d_10.png", frame_count * frame_skip);
            std::string input_image_file_with_path = m_generatepath.string() + file_name_image;

            sprintf(file_name_image_output, "000%03d_10_bb.png", frame_count * frame_skip);
            //output_image_file_with_path = m_generatepath.string() + "stencil/" + file_name_image_output;

            cv::Mat tempGroundTruthImageBase = cv::imread(input_image_file_with_path, CV_LOAD_IMAGE_ANYCOLOR);
            tempGroundTruthImage = cv::Scalar::all(255);
            tempGroundTruthImage = tempGroundTruthImageBase.clone();

            for (int obj_index = 0; obj_index < m_ptr_customObjectMetaDataList.size(); obj_index++) {

                std::vector<cv::Point3f> bounding_points_3d(9);
                std::vector<cv::Point2f> bounding_points_2d(9);

                object_realworld_dim_m_str object_realworld_dim_m = m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_realworld_dim_m;

                object_location_inertial_m_str pos_obj_inertial = m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_location_inertial_m;

                object_location_m_str object_location_m = m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_location_m;

                object_rotation_inertial_rad_str orientation_obj_inertial = m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_rotation_inertial_rad;

                sensor_location_carrier_m_str pos_sensor_carrier_inertial = m_ptr_customSensorMetaDataList.at(0)->getAll().at(frame_count).m_sensor_location_carrier_m;

                sensor_rotation_carrier_rad_str sensor_rotation_carrier_rad = m_ptr_customSensorMetaDataList.at(0)->getAll().at(
                        frame_count).m_sensor_rotation_carrier_rad;

                sensor_offset_m_str sensor_offset_m = m_ptr_customSensorMetaDataList.at(0)->getAll().at(
                        frame_count).m_sensor_offset_m;

                float offset_x = m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_offset_m.offset_x;
                float offset_y = m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_offset_m.offset_y;
                float offset_z = m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        frame_count).m_object_offset_m.offset_z;

/*
 * assuming vehicle co-ordinate system of the object.
 * create a box placed on z = 0 ( street level ). Find 8 points of the box.
 *
 */
                bounding_points_3d.at(0) = cv::Point3f(object_realworld_dim_m.dim_length_m / 2, object_realworld_dim_m.dim_width_m/ 2, 0);
                bounding_points_3d.at(1) = cv::Point3f(-object_realworld_dim_m.dim_length_m / 2, object_realworld_dim_m.dim_width_m / 2, 0);
                bounding_points_3d.at(2) = cv::Point3f(object_realworld_dim_m.dim_length_m / 2, -object_realworld_dim_m.dim_width_m / 2, 0);
                bounding_points_3d.at(3) = cv::Point3f(-object_realworld_dim_m.dim_length_m / 2, -object_realworld_dim_m.dim_width_m / 2, 0);
                bounding_points_3d.at(4) = cv::Point3f(object_realworld_dim_m.dim_length_m / 2, object_realworld_dim_m.dim_width_m / 2, object_realworld_dim_m.dim_height_m);
                bounding_points_3d.at(5) = cv::Point3f(-object_realworld_dim_m.dim_length_m / 2, object_realworld_dim_m.dim_width_m / 2, object_realworld_dim_m.dim_height_m);
                bounding_points_3d.at(6) = cv::Point3f(object_realworld_dim_m.dim_length_m / 2, -object_realworld_dim_m.dim_width_m / 2, object_realworld_dim_m.dim_height_m);
                bounding_points_3d.at(7) = cv::Point3f(-object_realworld_dim_m.dim_length_m / 2, -object_realworld_dim_m.dim_width_m / 2, object_realworld_dim_m.dim_height_m);
                bounding_points_3d.at(8) = cv::Point3f(0,0,object_realworld_dim_m.dim_height_m/2); // This is the center of gravity

                cv::Point3f final;

                sensor_fov_rad_str fov_rad = m_ptr_customSensorMetaDataList.at(0)->getAll().at(0).m_sensor_fov_rad;


                for ( auto i = 0; i < 9; i++ ) {

                    //Add the offset for each point. These points are in the vehicle coordinate system.
                    final = Utils::translate_and_rotate_points(bounding_points_3d.at(i), cv::Point3f(offset_x, offset_y, offset_z),cv::Point3f(0,0,0));

                    //Then rotate the box to inertial coordinate system. hpr. Now the BB points are in the inertial co-ordinate system with the origin at the position.
                    final = Utils::translate_and_rotate_points(bounding_points_3d.at(i), cv::Point3f(0,0,0),cv::Point3f(orientation_obj_inertial.rotation_rz_yaw_rad,orientation_obj_inertial.rotation_ry_pitch_rad, orientation_obj_inertial.rotation_rx_roll_rad));

                    //Translate the axis to the master origin. add the BB vector to the object position.
                    //Now we are in the master co-ordinate system.
                    final = Utils::translate_and_rotate_points(final, cv::Point3f(pos_obj_inertial.location_x_m, pos_obj_inertial.location_y_m,
                                                                                  pos_obj_inertial.location_z_m), cv::Point3f(0,0,0));

                    // Change to sensor object by changing the axis to the sensor object.
                    final = Utils::translate_and_rotate_points(final, cv::Point3f(-pos_sensor_carrier_inertial.location_x_m, -pos_sensor_carrier_inertial.location_y_m, -pos_sensor_carrier_inertial.location_z_m), cv::Point3f(0,0,0));

                    // now rotate the axis to align to the car
                    final = Utils::translate_and_rotate_points(final, cv::Point3f(0,0,0), cv::Point3f(-sensor_rotation_carrier_rad.rotation_rz_yaw_rad, -sensor_rotation_carrier_rad.rotation_ry_pitch_rad, -sensor_rotation_carrier_rad.rotation_rx_roll_rad));

                    // We are in the vehicle coordinate system now.
                    // Translate to cam position in the car ... The offset_x should be negative - but it doesnt work !!!!!!
                    final = Utils::translate_and_rotate_points(final, cv::Point3f(sensor_offset_m.offset_x, -sensor_offset_m.offset_y, -sensor_offset_m.offset_z), cv::Point3f(0,0,0));

                    // The resulting points are the bounding box points in the USK co-ordinate system.
                    bounding_points_3d.at(i) = final;

                    cv::Point2f camPoint = Utils::worldToCamera(final, fov_rad.vertical, 980, 980);
                    bounding_points_2d.at(i) = cv::Point2f(camPoint.x, camPoint.y);

                }

                auto dist = cv::norm(cv::Point2f(bounding_points_3d.at(8).x+sensor_offset_m.offset_x, bounding_points_3d.at(8).y+sensor_offset_m.offset_y));
                auto dist_usk = cv::norm(
                        cv::Point2f(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                frame_count).m_object_location_m.location_x_m,
                                    m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                            frame_count).m_object_location_m.location_y_m));
                //assert(std::abs(dist - dist_usk) < 1);
                std::cout << "distance is " << dist << " for " << m_ptr_customObjectMetaDataList.at(obj_index)->getObjectName() << std::endl;

                m_ptr_customObjectMetaDataList.at(obj_index)->setBoundingBoxPoints(frame_count, bounding_points_2d);

                cv::Point2f xx = Utils::worldToCamera(cv::Point3f(object_location_m.location_y_m, object_location_m.location_z_m, object_location_m.location_x_m), fov_rad.vertical, 980, 980);

                if ((m_ptr_customObjectMetaDataList.at(0)->getAll().at(frame_count).occluded == false)
                        ) {

                    cv::Rect boundingbox = cv::Rect(
                            cvRound(xx.x),
                            cvRound(xx.y),
                            cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_object_dimensions_px.dim_width_m),
                            cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_object_dimensions_px.dim_height_m));

                    cv::rectangle(tempGroundTruthImage, boundingbox, cv::Scalar(0, 255, 0), 1, 8, 0);

                    std::vector<cv::Point2f> box = {
                            m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_lower_bottom_px,
                            m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_lower_left_px,
                            m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_lower_top_px,
                            m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_lower_right_px,
                            m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_higher_bottom_px,
                            m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_higher_left_px,
                            m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_higher_top_px,
                            m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(frame_count).m_bounding_box.bb_higher_right_px
                    };

                    for ( auto i = 0; i < 8; i++ ) {
                        //std::cout << box.at(i) << std::endl;
                        cv::circle(tempGroundTruthImage, box.at(i), 2, cv::Scalar(0, 0, 255), 3);
                    }
                    cv::Rect box_points = cv::boundingRect(box);
                    cv::rectangle(tempGroundTruthImage, cv::boundingRect(box), cv::Scalar(0, 0, 255), 1, 8, 0);

                }
            }

            //cv::namedWindow("BB", CV_WINDOW_AUTOSIZE);
            //cv::imshow("BB", tempGroundTruthImage);
            //cv::waitKey(0);
            //cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
            /*---------------------------------------------------------------------------------*/

        }
    }
}

void GroundTruthSceneExternal::generate_gt_scene() {

    Noise noNoise;

    if (m_environment == "none") {

        if (!m_regenerate_yaml_file) { // dont generate, just read

            readPositionFromFile("../position_vires.yml");

            ushort map_pair_count = 0;
            for (const auto &myPair : m_mapObjectNameToObjectMetaData) {
                std::cout << myPair.first << "\n";
                map_pair_count++;
            }
        }
    }

    if (m_regenerate_yaml_file) { // call VIRES only at the time of generating the files

        char command[1024];

        std::string project = "Movement";

        std::vector<std::string> list_of_scenarios = {"carTypesComplete.xml", "crossing8Demo.xml",
                                                      "crossing8DualExt.xml",
                                                      "crossing8Static.xml", "HighwayPulk.xml", "invisibleCar.xml",
                                                      "ParkPerp.xml",
                                                      "RouteAndPathShapeSCP.xml",
                                                      "staticCar.xml", "TownActionsPath.xml", "TownPathLong.xml",
                                                      "traffic_demo2Ext.xml",
                                                      "trafficDemoClosePath.xml", "trafficDemoPath.xml",
                                                      "trafficDemoPed.xml", "traffic_demoReverse.xml",
                                                      "trafficDemoTrailer.xml", "trafficDemoUK.xml", "traffic_demo.xml"
                                                              "car.xml", "moving_car_near.xml", "moving_car.xml",
                                                      "moving_truck.xml", "moving.xml", "one.xml",
                                                      "truck.xml", "two.xml"};

        sleep(5); // Wait before starting vtd again.

        if (m_environment == "none") {
            sprintf(command, "cd %s../../ ; bash vtdSendandReceive.sh %s", (m_datasetpath.string()).c_str(),
                    project.c_str());
            std::cout << command << std::endl;
            system(command);
            std::cout << " I am out of bash" << std::endl;
        }

        sleep(5); // Give some time before you send SCP commands.

        // std::string m_server;
        boost::filesystem::path m_ts_gt_out_dir;

        int initCounter = 6;

        // initalize the server variable
        std::string serverName = "127.0.0.1";

        setServer(serverName.c_str());

        fprintf(stderr, "ValidateArgs: key = 0x%x, checkMask = 0x%x, mForceBuffer = %d\n", mShmKey, getCheckMask(),
                getForceBuffer());

        bool connected_trigger_port = false;
        bool connected_module_manager_port = false;
        bool connected_scp_port = false;

        //if ( m_environment == "none" ) {
        m_scpSocket = openNetwork(SCP_DEFAULT_PORT);
        //}
        std::cout << "scp socket - " << m_scpSocket << std::endl;
        if (m_scpSocket != -1) { // this is blocking until the network has been opened
            connected_scp_port = true;
        }

        sleep(1); // Give some time before you send the next SCP command.

        sendSCPMessage(m_scpSocket, apply.c_str());

        sleep(5); // This is very important !! Mimimum 5 seconds of wait, till you start the simulation

        sendSCPMessage(m_scpSocket, project_name.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, image_generator.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, rdbtrigger_portnumber.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, scenario_name.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, module_manager_libModuleCameraSensor.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, module_manager_libModulePerfectSensor.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, module_manager_libModulePerfectSensorInertial.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, view_parameters_sensorpoint_openglfrustum.c_str());
        //sendSCPMessage(m_scpSocket, view_parameters_sensorpoint_intrinsicparams.c_str());
        sleep(1);

        sendSCPMessage(m_scpSocket, display_parameters.c_str());

        sleep(2);

        sendSCPMessage(m_scpSocket, m_environment_scp_message.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, message_scp.c_str());

        sleep(1);

        //sendSCPMessage(m_scpSocket, popup_scp.c_str());

        //sleep(1);

        sendSCPMessage(m_scpSocket, eyepoint.c_str());

        sleep(1);


        sendSCPMessage(m_scpSocket, elevation.c_str());

        sleep(1);

        //sendSCPMessage(m_scpSocket, bbox.c_str());

        sleep(1);

        sprintf(command, "cd %s../../ ; bash vtdRunScp.sh", (m_datasetpath.string()).c_str());
        std::cout << command << std::endl;
        system(command);

        sleep(10);  // Give some time before you start the trigger and module manager ports.

        //readScpNetwork(m_scpSocket);

        //readScpNetwork(m_scpSocket);

        //sleep(1);


        // open the network connection to the taskControl (so triggers may be sent)
        fprintf(stderr, "creating network connection....\n");
        //if ( m_environment == "none") {
        m_triggerSocket = openNetwork(DEFAULT_PORT);
        //}
        std::cout << "trigger socket - " << m_triggerSocket << std::endl;
        if (m_triggerSocket != -1) { // this is blocking until the network has been opened
            connected_trigger_port = true;
        }

        //if ( m_environment == "none") {
        m_moduleManagerSocket_Camera = openNetwork(DEFAULT_RX_PORT);
        m_moduleManagerSocket_Perfect = openNetwork(DEFAULT_RX_PORT_PERFECT);
        m_moduleManagerSocket_PerfectInertial = openNetwork(DEFAULT_RX_PORT_PERFECT_INERTIAL);
        //}

        std::cout << "mm socket - " << m_moduleManagerSocket_Camera << std::endl;
        if (m_moduleManagerSocket_Camera != -1) {
            connected_module_manager_port = true;
        }

        if (connected_trigger_port && connected_module_manager_port && connected_scp_port) {

            // open the shared memory for IG image output (try to attach without creating a new segment)
            fprintf(stderr, "openCommunication: attaching to shared memory (IG image output) 0x%x....\n", mShmKey);

            while (!getShmPtr()) {
                openShm(mShmKey);
                usleep(1000);     // do not overload the CPU
            }

            // now check the SHM for the time being
            bool breaking = false;
            int count = 0;

            try {
                while (1) {

                    if (m_breaking) {
                        break;
                    }

                    fprintf(stderr,
                            "------------------------------------------------------------------------------------\n");
                    if (m_dumpInitialFrames) {

                        mCheckForImage = false;
                        fprintf(stderr, "sendRDBTrigger: sending trigger, deltaT = %.4lf, requestImage = false\n",
                                mDeltaTime);
                        sendRDBTrigger(m_triggerSocket, 0, 0, false, 0.01); // Extend the simulation by 10 ms

                    } else {
                        mCheckForImage = true;
                        fprintf(stderr, "sendRDBTrigger: sending trigger, deltaT = %.4lf, requestImage = true \n",
                                mDeltaTime);
                        sendRDBTrigger(m_triggerSocket, 0, 0, true, 0.1); // Extend the simulation by 100 ms
                    }

                    while (mCheckForImage) {

                        checkShm();

                        if (mHaveImage) {
                            //fprintf( stderr, "main: got it! at %d\n", mSimFrame );
                            mHaveImage = false;
                            mCheckForImage = false;
                        }
                        usleep(10);
                    }

                    if (!mHaveFirstFrame) {

                        usleep(100000);
                        std::cerr << "Flushing images in shared memory" << std::endl;
                        checkShm();  //empty IG buffer of spurious images
                        // only in the beginning.

                    }

                    readNetwork(m_moduleManagerSocket_Camera);  // this calls parseRDBMessage() in vires_common.cpp

                    readNetwork(m_moduleManagerSocket_Perfect);  // this calls parseRDBMessage() in vires_common.cpp

                    readNetwork(m_moduleManagerSocket_PerfectInertial);  // this calls parseRDBMessage() in vires_common.cpp


                    usleep(100000); // wait, 100 ms which is equivalent to 10 Hz. Normally VIRES runs with 60 Hz. So this number should not be a problem.
                    //std::cout << "getting data from VIRES\n";
                }
            }
            catch (std::bad_alloc e) {
                std::cerr << "Error in generation" << std::endl;
                stopSimulation();
                return;
            };

            configVires();
        }
    }
    try {
        if (m_environment == "none") {

            startEvaluating("vires", noNoise);

        }
    }
    catch (...) {
        std::cerr << "VTD Generation complete, but error in generating images" << std::endl;
        stopSimulation();
    }
}


void GroundTruthSceneExternal::parseEntry(RDB_TRIGGER_t *data, const double &simTime, const unsigned int &
simFrame, const unsigned short &pkgId, const unsigned short &flags, const unsigned int &elemId,
                                          const unsigned int &totalElem) {
    fprintf(stderr, "RDBTrigger answer = %.3f, simFrame = %d\n", simTime, simFrame);
}


void GroundTruthSceneExternal::parseStartOfFrame(const double &simTime, const unsigned int &simFrame) {
    //fprintf(stderr, "I am in GroundTruthFlow %d\n,", RDB_PKG_ID_START_OF_FRAME);
    //mHaveFirstFrame = true;
    //fprintf(stderr, "RDBHandler::parseStartOfFrame: simTime = %.3f, simFrame = %d\n", simTime, simFrame);
}

void GroundTruthSceneExternal::parseEndOfFrame(const double &simTime, const unsigned int &simFrame) {

    if (simFrame == MAX_DUMPS) {
        m_dumpInitialFrames = false;
    }
    mHaveFirstFrame = true;

    //mLastNetworkFrame = simFrame;
    fprintf(stderr, "RDBHandler::parseEndOfFrame: simTime = %.3f, simFrame = %d\n", simTime, simFrame);

    if (simFrame > MAX_ITERATION_GT_SCENE_GENERATION_DYNAMIC + MAX_DUMPS) {
        m_breaking = true;
    }

}

/** ------ state of an object (may be extended by the next structure) ------- */
typedef struct {
    uint32_t id;                         /**< unique object ID                                              @unit _                                   */
    uint8_t category;                   /**< object category                                               @unit @link RDB_OBJECT_CATEGORY @endlink  */
    uint8_t type;                       /**< object type                                                   @unit @link RDB_OBJECT_TYPE     @endlink  */
    uint16_t visMask;                    /**< visibility mask                                               @unit @link RDB_OBJECT_VIS_FLAG @endlink  */
    char name[RDB_SIZE_OBJECT_NAME]; /**< symbolic name                                                 @unit _                                   */
    RDB_GEOMETRY_t geo;                        /**< info about object's geometry                                  @unit m,m,m,m,m,m                         */
    RDB_COORD_t pos;                        /**< position and orientation of object's reference point          @unit m,m,m,rad,rad,rad                   */
    uint32_t parent;                     /**< unique ID of parent object                                    @unit _                                   */
    uint16_t cfgFlags;                   /**< configuration flags                                           @unit @link RDB_OBJECT_CFG_FLAG @endlink  */
    int16_t cfgModelId;                 /**< visual model ID (configuration parameter)                     @unit _                                   */
} RDB_OBJECT_STATE_BASE_DUMMY_t;

/** ------ extended object data (e.g. for dynamic objects) ------- */
typedef struct {
    RDB_COORD_t speed;                      /**< speed and rates                                               @unit m/s,m/s,m/s,rad/s,rad/s,rad/s           */
    RDB_COORD_t accel;                      /**< acceleration                                                  @unit m/s2,m/s2,m/s2,rad/s2,rad/s2/rad/s2     */
    float traveledDist;               /**< traveled distance                                             @unit m                                      a */
    uint32_t spare[3];                   /**< reserved for future use                                       @unit _                                       */
} RDB_OBJECT_STATE_EXT_DUMMY_t;

/** ------ sensor definition and state ------ */
typedef struct {
    uint32_t id;                          /**< id of the sensor                                      @unit _                                      */
    uint8_t type;                        /**< type of the sensor                                    @unit @link RDB_SENSOR_TYPE     @endlink     */
    uint8_t hostCategory;                /**< category of the object hosting the sensor             @unit @link RDB_OBJECT_CATEGORY @endlink     */
    uint16_t spare0;                      /**< for future use                                        @unit _                                      */
    uint32_t hostId;                      /**< unique id of the sensor's host                        @unit _                                      */
    char name[RDB_SIZE_OBJECT_NAME];  /**< name of the sensor                                    @unit _                                      */
    float fovHV[2];                    /**< field-of-view (horizontal/vertical)                   @unit rad,rad                                */
    float clipNF[2];                   /**< clipping ranges (near/far)                            @unit m,m                                    */
    RDB_COORD_t pos;                         /**< position and orientation of sensor's reference point  @unit m,m,m,rad,rad,rad                      */
    RDB_COORD_t originCoordSys;              /**< position and orientation of sensor's coord origin     @unit m,m,m,rad,rad,rad                      */
    float fovOffHV[2];                 /**< field-of-view offset (horizontal/vertical)            @unit rad, rad                              B */
    int32_t spare[2];                    /**< for future use                                        @unit _                                      */
} RDB_SENSOR_STATE_DUMMY_t;

/** ------ information about an object registered within a sensor ------ */
typedef struct {
    uint8_t category;    /**< object category                                                                @unit @link RDB_OBJECT_CATEGORY    @endlink   */
    uint8_t type;        /**< object type                                                                    @unit @link RDB_OBJECT_TYPE        @endlink   */
    uint16_t flags;       /**< sensor object flags                                                            @unit @link RDB_SENSOR_OBJECT_FLAG @endlink   */
    uint32_t id;          /**< id of the object                                                               @unit _                                       */
    uint32_t sensorId;    /**< id of the detecting sensor                                                     @unit _                                       */
    double dist;        /**< distance between object and referring device                                   @unit m                                       */
    RDB_COORD_t sensorPos;   /**< position and orientation of object in sensor coord                             @unit m,m,m,rad,rad,rad                       */
    int8_t occlusion;   /**< degree of occlusion for viewer (-1 = not valid, 0..127 = 0..100% occlusion)    @unit [-1, 0..127]                            */
    uint8_t spare0[3];   /**< for future use                                                                 @unit _                                       */
    uint32_t spare[3];    /**< for future use                                                                 @unit _                                       */
} RDB_SENSOR_OBJECT_DUMMY_t;

void GroundTruthSceneExternal::parseEntry(RDB_OBJECT_CFG_t *data, const double &simTime, const unsigned int &
simFrame, const
                                          unsigned short &pkgId, const unsigned short &flags,
                                          const unsigned int &elemId, const unsigned int &totalElem) {

    RDB_OBJECT_CFG_t *object = reinterpret_cast<RDB_OBJECT_CFG_t *>(data); /// raw image data
    std::cout << object->type;


}

void GroundTruthSceneExternal::parseEntry(RDB_SENSOR_STATE_t *data, const double &simTime, const unsigned int &simFrame,
                                          const unsigned short &pkgId, const unsigned short &flags,
                                          const unsigned int &elemId, const unsigned int &totalElem) {

    cv::Point3f position_sensor_carrier, orientation_sensor_carrier, position_sensor, orientation_sensor, offset_sensor;
    cv::Point2f fov;

    if (m_environment == "none") {

        if (data->type == RDB_SENSOR_TYPE_VIDEO || data->type == RDB_SENSOR_TYPE_RADAR) {

            if (!m_dumpInitialFrames && (simFrame > (MAX_DUMPS+1)) && ((simFrame - MAX_DUMPS - 1) < MAX_ITERATION_GT_SCENE_GENERATION_DYNAMIC ) ) {

                fprintf(stderr, "saving sensor truth for simFrame = %d, simTime %f %s\n", simFrame, simTime,
                        data->name);

                if (m_mapSensorNameToSensorMetaData.count(data->name) == 0) {

                    m_mapSensorIdToSensorName[data->id] = data->name;
                    m_ptr_customSensorMetaDataList.push_back(&sensorMetaDataList.at(m_sensorCount));
                    m_mapSensorNameToSensorMetaData[data->name] = m_ptr_customSensorMetaDataList.at(m_sensorCount);
                    m_ptr_customSensorMetaDataList.at(m_sensorCount)->setSensorName(data->name);
                    m_ptr_customSensorMetaDataList.at(m_sensorCount)->setStartPoint(0);
                    m_sensorCount += 1;
                }

                position_sensor_carrier = cv::Point3f((float) data->originCoordSys.x, (float) data->originCoordSys.y,
                                                      (float) data->originCoordSys.z);
                orientation_sensor_carrier = cv::Point3f((float) data->originCoordSys.h, (float) data->originCoordSys.p,
                                                         (float) data->originCoordSys.r);

                offset_sensor = cv::Point3f((float) data->pos.x, (float) data->pos.y,
                                            (float) data->pos.z);
                orientation_sensor = cv::Point3f((float) data->pos.h, (float) data->pos.p,
                                                 (float) data->pos.r);

                fov = cv::Point2f(data->fovHV[0], data->fovHV[1]);

                m_mapSensorNameToSensorMetaData[data->name]->atFrameNumberSensorState(
                        (ushort) ((simFrame - MAX_DUMPS - 2) / IMAGE_SKIP_FACTOR_DYNAMIC), position_sensor_carrier,
                        orientation_sensor_carrier, orientation_sensor, offset_sensor, fov);
            }
        }
    }
}

void GroundTruthSceneExternal::parseEntry(RDB_OBJECT_STATE_t *data, const double &simTime, const unsigned int &
simFrame, const
                                          unsigned
                                          short &pkgId, const unsigned short &flags, const unsigned int &elemId,
                                          const unsigned int &totalElem) {

    cv::Point3f offset, position_inertial, position_usk, position_pixel, dimension_realworld;
    cv::Point2f dimension_pixel;
    cv::Point2f speed_inertial, speed_usk;
    cv::Point3f orientation_usk, orientation_inertial;
    float dist_cam_to_obj;
    float total_distance_travelled;

    if (m_environment == "none") {

        if (data->base.type == RDB_OBJECT_TYPE_PLAYER_PEDESTRIAN || data->base.type == RDB_OBJECT_TYPE_PLAYER_CAR) {
            if (!m_dumpInitialFrames && (simFrame > (MAX_DUMPS+1)) && ((simFrame - MAX_DUMPS - 1) < MAX_ITERATION_GT_SCENE_GENERATION_DYNAMIC ) ) {

                /*
                fprintf(stderr, "%s: %d %.3lf %.3lf %.3lf %.3lf \n",
                        data->base.name, simFrame, data->base.pos.x, data->base.pos.y, data->base.geo.dimX, data->base
                                .geo.dimY);
*/
                fprintf(stderr, "saving ground truth for simFrame = %d, simTime %f %s\n", simFrame, simTime,
                        data->base.name);

                if (m_mapObjectNameToObjectMetaData.count(data->base.name) == 0) {

                    m_mapObjectIdToObjectName[data->base.id] = data->base.name;
                    m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(m_objectCount));
                    Rectangle rectangle((int) (data->base.geo.dimX), (int) (data->base.geo.dimY)); // width, height
                    m_mapObjectNameToObjectMetaData[data->base.name] = m_ptr_customObjectMetaDataList.at(m_objectCount);
                    m_ptr_customObjectMetaDataList.at(m_objectCount)->setObjectShape(rectangle);
                    m_ptr_customObjectMetaDataList.at(m_objectCount)->setObjectName(data->base.name);
                    m_ptr_customObjectMetaDataList.at(m_objectCount)->setStartPoint(0);
                    m_objectCount += 1;
                }
                if (data->base.pos.type == RDB_COORD_TYPE_WINDOW) {

                    position_pixel = cv::Point3f((float) data->base.pos.x, (float) data->base.pos.y,
                                                 float(data->base.pos.z));
                    dimension_pixel = cv::Point2f((float) data->base.geo.dimX, (float) data->base.geo.dimY);
                    offset = cv::Point3f((float) data->base.geo.offX, (float) data->base.geo.offY,
                                         (float) data->base.geo.offZ);
                    total_distance_travelled = data->ext.traveledDist;
                    m_mapObjectNameToObjectMetaData[data->base.name]->atFrameNumberCameraSensor(
                            (ushort) ((simFrame - MAX_DUMPS - 2) / IMAGE_SKIP_FACTOR_DYNAMIC), position_pixel, offset,
                            dimension_pixel, total_distance_travelled);
                    m_mapObjectNameToObjectMetaData[data->base.name]->atFrameNumberVisibility(
                            (ushort) ((simFrame - MAX_DUMPS - 2) / IMAGE_SKIP_FACTOR_DYNAMIC), data->base.visMask);

                } else if (data->base.pos.type == RDB_COORD_TYPE_USK) {

                    position_usk = cv::Point3f((float) data->base.pos.x, (float) data->base.pos.y,
                                               (float) data->base.pos.z);
                    orientation_usk = cv::Point3f((float) data->base.pos.h, (float) data->base.pos.p,
                                                  (float) data->base.pos.r);
                    dimension_realworld = cv::Point3f((float) data->base.geo.dimX, (float) data->base.geo.dimY,
                                                      (float) data->base.geo.dimZ);
                    speed_usk = cv::Point2f((float) data->ext.speed.x, (float) data->ext.speed.y);
                    total_distance_travelled = data->ext.traveledDist;
                    m_mapObjectNameToObjectMetaData[data->base.name]->atFrameNumberPerfectSensor(
                            (ushort) ((simFrame - MAX_DUMPS - 2) / IMAGE_SKIP_FACTOR_DYNAMIC), position_usk,
                            orientation_usk, dimension_realworld, speed_usk, total_distance_travelled);
                    m_mapObjectNameToObjectMetaData[data->base.name]->atFrameNumberVisibility(
                            (ushort) ((simFrame - MAX_DUMPS - 2) / IMAGE_SKIP_FACTOR_DYNAMIC), data->base.visMask);

                } else if (data->base.pos.type == RDB_COORD_TYPE_INERTIAL) {

                    position_inertial = cv::Point3f((float) data->base.pos.x, (float) data->base.pos.y,
                                                    (float) data->base.pos.z);
                    orientation_inertial = cv::Point3f((float) data->base.pos.h, (float) data->base.pos.p,
                                                       (float) data->base.pos.r);
                    dimension_realworld = cv::Point3f((float) data->base.geo.dimX, (float) data->base.geo.dimY,
                                                      (float) data->base.geo.dimZ);
                    speed_inertial = cv::Point2f((float) data->ext.speed.x, (float) data->ext.speed.y);
                    total_distance_travelled = data->ext.traveledDist;
                    m_mapObjectNameToObjectMetaData[data->base.name]->atFrameNumberPerfectSensorInertial(
                            (ushort) ((simFrame - MAX_DUMPS - 2) / IMAGE_SKIP_FACTOR_DYNAMIC), position_inertial,
                            orientation_inertial, dimension_realworld, speed_inertial, total_distance_travelled);
                    m_mapObjectNameToObjectMetaData[data->base.name]->atFrameNumberVisibility(
                            (ushort) ((simFrame - MAX_DUMPS - 2) / IMAGE_SKIP_FACTOR_DYNAMIC), data->base.visMask);
                }
            } else {
                //std::cout << data->base.type << std::endl;
            }
        }
    }
}


void GroundTruthSceneExternal::parseEntry(RDB_SENSOR_OBJECT_t *data, const double &simTime, const unsigned int &
simFrame, const unsigned short &pkgId, const unsigned short &flags, const unsigned int &elemId,
        const unsigned int &totalElem) {

    cv::Point3f offset, position_inertial, position_usk, position_pixel, dimension_realworld;
    cv::Point2f dimension_pixel;
    cv::Point2f speed_inertial, speed_usk;
    cv::Point3f orientation_usk, orientation_inertial;
    float dist_cam_to_obj;
    float total_distance_travelled;

    if (m_environment == "none") {

        if (data->type == RDB_OBJECT_TYPE_PLAYER_PEDESTRIAN || data->type == RDB_OBJECT_TYPE_PLAYER_CAR) {
            if (!m_dumpInitialFrames && (simFrame > (MAX_DUMPS+1)) && ((simFrame - MAX_DUMPS - 1) < MAX_ITERATION_GT_SCENE_GENERATION_DYNAMIC ) ) {

                //fprintf(stderr, "%s: %d %.3lf %.3lf %.3lf %.3lf \n",
                //        data->name, simFrame, data->pos.x, data->pos.y, data->geo.dimX, data->
                //                .geo.dimY);

                fprintf(stderr, "saving ground truth for simFrame = %d, simTime %f %s\n", simFrame, simTime,
                        m_mapObjectIdToObjectName[data->id]);

                if (m_mapObjectNameToObjectMetaData.count(m_mapObjectIdToObjectName[data->id]) == 0) {

                    m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(m_objectCount));
                    //Rectangle rectangle((int) (data->geo.dimX), (int) (data->geo.dimY)); // width, height
                    m_mapObjectNameToObjectMetaData[m_mapObjectIdToObjectName[data->id]] = m_ptr_customObjectMetaDataList.at(m_objectCount);
                    //m_ptr_customObjectMetaDataList.at(m_objectCount)->setObjectShape(rectangle);
                    m_ptr_customObjectMetaDataList.at(m_objectCount)->setObjectName(m_mapObjectIdToObjectName[data->id]);
                    m_ptr_customObjectMetaDataList.at(m_objectCount)->setStartPoint(0);
                    m_objectCount += 1;
                }
                if (data->sensorPos.type == RDB_COORD_TYPE_WINDOW) {

                    m_mapObjectNameToObjectMetaData[m_mapObjectIdToObjectName[data->id]]->atFrameNumberOcclusionWindow(
                            (ushort) ((simFrame - MAX_DUMPS - 2) / IMAGE_SKIP_FACTOR_DYNAMIC), data->occlusion);
                } else if (data->sensorPos.type == RDB_COORD_TYPE_USK) {

                    m_mapObjectNameToObjectMetaData[m_mapObjectIdToObjectName[data->id]]->atFrameNumberOcclusionUsk(
                            (ushort) ((simFrame - MAX_DUMPS - 2) / IMAGE_SKIP_FACTOR_DYNAMIC), data->occlusion);
                } else if (data->sensorPos.type== RDB_COORD_TYPE_INERTIAL) {

                    m_mapObjectNameToObjectMetaData[m_mapObjectIdToObjectName[data->id]]->atFrameNumberOcclusionInertial(
                            (ushort) ((simFrame - MAX_DUMPS - 2) / IMAGE_SKIP_FACTOR_DYNAMIC), data->occlusion);
                }
            } else {
                //std::cout << data->base.type << std::endl;
            }
        }
    }
}


void GroundTruthSceneExternal::parseEntry(RDB_IMAGE_t *data, const double &simTime, const unsigned int &simFrame,
                                          const
                                          unsigned short &pkgId, const unsigned short &flags,
                                          const unsigned int &elemId, const unsigned int &totalElem) {

    if (!data)
        return;
    //fprintf(stderr, "handleRDBitem: image at simFrame %d\n", simFrame);
    //fprintf(stderr, "    simTime = %.3lf, simFrame = %d, mLastShmFrame = %d\n", simTime, simFrame, getLastShmFrame());
    //fprintf(stderr, "    width / height = %d / %d\n", data->width, data->height);
    //fprintf(stderr, "    dataSize = %d\n", data->imgSize);

    // ok, I have an image, but it might be the first one

    //analyzeImage(  data  , simFrame, 0 );

    if ( simFrame > 1 ) {

        mHaveFirstImage = true;
    }
    else {
        fprintf(stderr, "ignoring intial images for simFrame = %d, simTime = %.3f, dataSize = %d with image id %d\n",
                simFrame, simTime, data->imgSize, data->id);
    }

    if (mHaveFirstImage) { // always ignore the first image after real acquisition.
        char *image_data_ = NULL;
        RDB_IMAGE_t *image = reinterpret_cast<RDB_IMAGE_t *>(data); /// raw image data

        /// RDB image information of \see image_data_
        RDB_IMAGE_t image_info_;
        memcpy(&image_info_, image, sizeof(RDB_IMAGE_t));

        if (NULL == image_data_) {
            image_data_ = reinterpret_cast<char *>(malloc(image_info_.imgSize));
        } else {
            image_data_ = reinterpret_cast<char *>(realloc(image_data_, image_info_.imgSize));
        }
        // jump data header
        memcpy(image_data_, reinterpret_cast<char *>(image) + sizeof(RDB_IMAGE_t), image_info_.imgSize);

        if (image_info_.imgSize == image_info_.width * image_info_.height * 3) {
            png::image<png::rgb_pixel> save_image(image_info_.width, image_info_.height);
            unsigned int count = 0;
            for (int32_t v = 0; v < image_info_.height; v++) {
                for (int32_t u = 0; u < image_info_.width; u++) {
                    png::rgb_pixel val;
                    val.red = (unsigned char) image_data_[count++];
                    val.green = (unsigned char) image_data_[count++];
                    val.blue = (unsigned char) image_data_[count++];
                    //val.alpha = (unsigned char)image_data_[count++];
                    save_image.set_pixel(u, v, val);
                }
            }

            char file_name_image[50];

            if (!m_dumpInitialFrames) {
                sprintf(file_name_image, "000%03d_10.png", mImageCount);
                std::string input_image_file_with_path = m_generatepath.string() + file_name_image;
                if ( simFrame > (MAX_DUMPS+1) ) {
                    fprintf(stderr, "saving image for simFrame = %d, simTime = %.3f, dataSize = %d with image id %d\n",
                            simFrame, simTime, data->imgSize, data->id);
                    save_image.write(input_image_file_with_path);
                }
                else {
                    fprintf(stderr, "ignoring image for simFrame = %d, simTime = %.3f, dataSize = %d with image id %d\n",
                            simFrame, simTime, data->imgSize, data->id);
                }
                mImageCount++;
            } else {
                fprintf(stderr, "ignoring image for simFrame = %d, simTime = %.3f, dataSize = %d with image id %d\n",
                        simFrame, simTime, data->imgSize, data->id);
            }
        } else {
            fprintf(stderr, "ignoring image for simFrame = %d, simTime = %.3f, dataSize = %d with image id %d\n",
                    simFrame, simTime, data->imgSize, data->id);
        }
        mHaveImage = true;
    }
}


/**
* handle driver control input and compute vehicle dynamics output
*/
void GroundTruthSceneExternal::parseEntry(RDB_DRIVER_CTRL_t *data, const double &simTime, const unsigned int &simFrame,
                                          const unsigned short &pkgId, const unsigned short &flags,
                                          const unsigned int &elemId, const unsigned int &totalElem) {

    if (!data)
        return;

    static bool sVerbose = true;
    static bool sShowMessage = false;
    static unsigned int sMyPlayerId = 1;             // this may also be determined from incoming OBJECT_CFG messages
    static double sLastSimTime = -1.0;

    fprintf(stderr, "handleRDBitem: handling driver control for player %d\n", data->playerId);

    // is this a new message?
    //if ( simTime == sLastSimTime )
    //    return;

    // is this message for me?
    if (data->playerId != sMyPlayerId)
        return;

    // check for valid inputs (only some may be valid)
    float mdSteeringAngleRequest = (data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_STEERING_WHEEL) ?
                                   data->steeringWheel / 19.0 : 0.0;
    float mdThrottlePedal = (data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_THROTTLE) ? data->throttlePedal : 0.0;
    float mdBrakePedal = (data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_BRAKE) ? data->brakePedal : 0.0;
    float mInputAccel = (data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL) ? data->accelTgt : 0.0;
    float mInputSteering = (data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING) ? data->steeringTgt : 0.0;
    float mdSteeringRequest = (data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING) ? data->steeringTgt : 0.0;
    float mdAccRequest = (data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL) ? data->accelTgt : 0.0;
    int mInputGear = 0;

    // check the input validity
    unsigned int validFlagsLat = RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING | RDB_DRIVER_INPUT_VALIDITY_STEERING_WHEEL;
    unsigned int validFlagsLong =
            RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL | RDB_DRIVER_INPUT_VALIDITY_THROTTLE | RDB_DRIVER_INPUT_VALIDITY_BRAKE;
    unsigned int checkFlags = data->validityFlags & 0x00000fff;

    if (checkFlags) {
        if ((checkFlags & validFlagsLat) && (checkFlags & validFlagsLong))
            sShowMessage = false;
        else if (checkFlags != RDB_DRIVER_INPUT_VALIDITY_GEAR) // "gear only" is also fine
        {
            if (!sShowMessage)
                fprintf(stderr, "Invalid driver input for vehicle dynamics");

            sShowMessage = true;
        }
    }

    // use pedals/wheel or targets?
    bool mUseSteeringTarget = ((data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING) != 0);
    bool mUseAccelTarget = ((data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL) != 0);

    if (data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_GEAR) {
        if (data->gear == RDB_GEAR_BOX_POS_R)
            mInputGear = -1;
        else if (data->gear == RDB_GEAR_BOX_POS_N)
            mInputGear = 0;
        else if (data->gear == RDB_GEAR_BOX_POS_D)
            mInputGear = 1;
        else
            mInputGear = 1;
    }

    // now, depending on the inputs, select the control mode and compute outputs
    if (mUseSteeringTarget && mUseAccelTarget) {
        fprintf(stderr, "Compute new vehicle position from acceleration target and steering target.\n");

        // call your methods here
    } else if (!mUseSteeringTarget && !mUseAccelTarget) {
        fprintf(stderr, "Compute new vehicle position from brake pedal, throttle pedal and steering wheel angle.\n");

        // call your methods here
    } else {
        fprintf(stderr, "Compute new vehicle position from a mix of targets and pedals / steering wheel angle.\n");

        // call your methods here
    }

    bool useDummy = true;

    RDB_OBJECT_STATE_t sOwnObjectState;

    // the following assignments are for dummy purposes only
    // vehicle moves along x-axis with given speed
    // ignore first message
    if (useDummy && (sLastSimTime >= 0.0)) {
        double speedX = 5.0;    // m/s
        double speedY = 0.0;    // m/s
        double speedZ = 0.0;    // m/s
        double dt = simTime - sLastSimTime;

        sOwnObjectState.base.id = sMyPlayerId;
        sOwnObjectState.base.category = RDB_OBJECT_CATEGORY_PLAYER;
        sOwnObjectState.base.type = RDB_OBJECT_TYPE_PLAYER_CAR;
        strcpy(sOwnObjectState.base.name, "Ego");

        // dimensions of own vehicle
        sOwnObjectState.base.geo.dimX = 4.60;
        sOwnObjectState.base.geo.dimY = 1.86;
        sOwnObjectState.base.geo.dimZ = 1.60;

        // offset between reference point and center of geometry
        sOwnObjectState.base.geo.offX = 0.80;
        sOwnObjectState.base.geo.offY = 0.00;
        sOwnObjectState.base.geo.offZ = 0.30;

        sOwnObjectState.base.pos.x += dt * speedX;
        sOwnObjectState.base.pos.y += dt * speedY;
        sOwnObjectState.base.pos.z += dt * speedZ;
        sOwnObjectState.base.pos.h = 0.0;
        sOwnObjectState.base.pos.p = 0.0;
        sOwnObjectState.base.pos.r = 0.0;
        sOwnObjectState.base.pos.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;

        sOwnObjectState.ext.speed.x = speedX;
        sOwnObjectState.ext.speed.y = speedY;
        sOwnObjectState.ext.speed.z = speedZ;
        sOwnObjectState.ext.speed.h = 0.0;
        sOwnObjectState.ext.speed.p = 0.0;
        sOwnObjectState.ext.speed.r = 0.0;
        sOwnObjectState.ext.speed.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;

        sOwnObjectState.ext.accel.x = 0.0;
        sOwnObjectState.ext.accel.y = 0.0;
        sOwnObjectState.ext.accel.z = 0.0;
        sOwnObjectState.ext.accel.flags = RDB_COORD_FLAG_POINT_VALID;

        sOwnObjectState.base.visMask = RDB_OBJECT_VIS_FLAG_TRAFFIC | RDB_OBJECT_VIS_FLAG_RECORDER;
    }

    // ok, I have a new object state, so let's send the data
    sendOwnObjectState(sOwnObjectState, m_triggerSocket, simTime, simFrame);

    // remember last simulation time
    sLastSimTime = simTime;
}

double GroundTruthSceneExternal::getTime() {
    struct timeval tme;
    gettimeofday(&tme, 0);

    double now = tme.tv_sec + 1.0e-6 * tme.tv_usec;

    if (mStartTime < 0.0)
        mStartTime = now;

    return now;
}


void GroundTruthSceneExternal::calcStatistics() {
    double now = getTime();

    double dt = now - mStartTime;

    if (dt < 1.e-6)
        return;

    fprintf(stderr,
            "calcStatistics: received %d/%d images in %.3lf seconds (i.e. %.3lf/%.3lf images per second ), total number of errors = %d\n",
            mTotalNoImages, dt, mTotalNoImages / dt, mTotalErrorCount);
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

