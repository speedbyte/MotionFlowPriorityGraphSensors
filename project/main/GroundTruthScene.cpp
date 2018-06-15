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
#include <vires-interface/vires_configuration.h>
#include "Utils.h"

using namespace std::chrono;


void GroundTruthScene::prepare_directories() {

    m_groundtruthpath = Dataset::getGroundTruthPath(); // data/stereo_flow

    m_generatepath = m_groundtruthpath.string() + "/" + m_environment;

    if (m_regenerate_yaml_file) {
        if (m_datasetpath.string() == std::string(CPP_DATASET_PATH) || m_datasetpath.string() == std::string(VIRES_DATASET_PATH)) {

            std::cout << "prepare gt_scene directories" << std::endl;

            for ( ushort i = 0 ; i < MAX_ALLOWED_SENSOR_GROUPS ; i++) {
                std::string m_generatepath_sensor = m_generatepath.string() + "_" + std::to_string(i);
                if (boost::filesystem::exists(m_generatepath_sensor)) {
                    system(("rm -rf " + m_generatepath_sensor).c_str());
                }
                boost::filesystem::create_directories(m_generatepath_sensor);

                char char_dir_append[20];
                boost::filesystem::path path;

                for (int i = 0; i < m_list_gt_objects.size(); i++) {

                    sprintf(char_dir_append, "%02d", i);
                    m_position_object_path = m_generatepath_sensor + "position_object_";
                    path = m_position_object_path.string() + char_dir_append;
                    //boost::filesystem::create_directories(path);
                }
            }

        }
    }
}

void GroundTruthScene::readPositionFromFile(std::string positionFileName) {


    cv::Point2f speed_usk, speed_inertial;
    cv::Point2f dimension_pixel, sensor_fov;
    cv::Point3f offset, position_inertial, position_usk, position_pixel, dimension_realworld;
    cv::Point3f position_inertial_pre, position_usk_pre, position_pixel_pre;
    float totalDistanceTravelled;

    cv::Point3f orientation_inertial, orientation_usk;


    cv::FileNode file_node;
    cv::FileNodeIterator file_node_iterator_begin, file_node_iterator_end, file_node_iterator;

    for (unsigned sensor_index = 0; sensor_index< MAX_ALLOWED_SENSOR_GROUPS; sensor_index++) {

        ushort  objectCount = 0;
        ushort  sensorCount = 0;
        cv::FileStorage fs;
        if ( sensor_index == 0 ) {
            fs.open("../position_vires_0.yml", cv::FileStorage::READ);
        }
        else {
            fs.open("../position_vires_1.yml", cv::FileStorage::READ);
        }

        assert(fs.isOpened());

        //std::string temp_str = "_sensor_count_" + sensor_index;
        char temp_str_fs[20];
        sprintf(temp_str_fs, "sensor_index_%03d", sensor_index);
        std::cout << "read yaml file for sensor_index " << (sensor_index) << std::endl;
        //unsigned long FRAME_COUNT = m_list_gt_objects.at(0).get_object_shape_point_displacement().at(sensor_index).size();
        unsigned long FRAME_COUNT = ITERATION_END_POINT;
        assert(FRAME_COUNT > 0);

        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

            std::cout << current_frame_index << std::endl;
            char temp_str_fc[20];
            sprintf(temp_str_fc, "frame_count_%03d", current_frame_index);
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

                        if (m_mapObjectNameToObjectMetaData[sensor_index].count((*file_node_iterator)["name"].string()) == 0
                                ) {
                            m_ptr_customObjectMetaDataList.at(sensor_index).push_back(&objectMetaDataList.at(m_objectCount));
                            m_mapObjectNameToObjectMetaData[sensor_index][(*file_node_iterator)["name"].string()] = m_ptr_customObjectMetaDataList.at(sensor_index).at(
                                    objectCount);
                            m_ptr_customObjectMetaDataList.at(sensor_index).at(objectCount)->setObjectName(
                                    (*file_node_iterator)["name"].string());
                            Rectangle rectangle(Dataset::getFrameSize().width,
                                                Dataset::getFrameSize().height); // width, height
                            m_ptr_customObjectMetaDataList.at(sensor_index).at(objectCount)->setObjectShape(rectangle);
                            m_ptr_customObjectMetaDataList.at(sensor_index).at(objectCount)->setStartPoint(ITERATION_START_POINT);
                            m_objectCount += 1;
                            objectCount += 1;
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

                        (m_mapObjectNameToObjectMetaData[sensor_index][(*file_node_iterator)["name"].string()])->atFrameNumberFrameCount(
                                current_frame_index);
                        (m_mapObjectNameToObjectMetaData[sensor_index][(*file_node_iterator)["name"].string()])->atFrameNumberCameraSensor(
                                current_frame_index, position_pixel, offset, dimension_pixel, totalDistanceTravelled);
                        (m_mapObjectNameToObjectMetaData[sensor_index][(*file_node_iterator)["name"].string()])->atFrameNumberPerfectSensor(
                                current_frame_index, position_usk, orientation_usk, dimension_realworld, speed_usk,
                                totalDistanceTravelled);
                        (m_mapObjectNameToObjectMetaData[sensor_index][(*file_node_iterator)["name"].string()])->atFrameNumberPerfectSensorInertial(
                                current_frame_index, position_inertial, orientation_inertial, dimension_realworld,
                                speed_inertial, totalDistanceTravelled);

                        (m_mapObjectNameToObjectMetaData[sensor_index][(*file_node_iterator)["name"].string()])->atFrameNumberVisibility(
                                current_frame_index, (int) (*file_node_iterator)["visMask"]);

                        (m_mapObjectNameToObjectMetaData[sensor_index][(*file_node_iterator)["name"].string()])->atFrameNumberOcclusionWindow(
                                current_frame_index, (int) (*file_node_iterator)["occ_px"]);

                        (m_mapObjectNameToObjectMetaData[sensor_index][(*file_node_iterator)["name"].string()])->atFrameNumberOcclusionUsk(
                                current_frame_index, (int) (*file_node_iterator)["occ_usk"]);

                        (m_mapObjectNameToObjectMetaData[sensor_index][(*file_node_iterator)["name"].string()])->atFrameNumberOcclusionInertial(
                                current_frame_index, (int) (*file_node_iterator)["occ_inertial"]);

                        position_inertial_pre = position_inertial;
                        position_pixel_pre = position_pixel;
                        position_usk_pre = position_usk;

                    }
                    else
                    {
                        if (m_mapSensorNameToSensorMetaData[sensor_index].count((*file_node_iterator)["name"].string()) == 0) {
                            m_ptr_customSensorMetaDataList[sensor_index].push_back(&sensorMetaDataList.at(m_sensorCount));
                            m_mapSensorNameToSensorMetaData[sensor_index][(*file_node_iterator)["name"].string()] = m_ptr_customSensorMetaDataList.at(sensor_index).at(sensorCount);
                            m_ptr_customSensorMetaDataList.at(sensor_index).at(sensorCount)->setSensorName(
                                    (*file_node_iterator)["name"].string());
                            m_ptr_customSensorMetaDataList.at(sensor_index).at(sensorCount)->setStartPoint(0);
                            m_sensorCount += 1;
                            sensorCount += 1;
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

                        (m_mapSensorNameToSensorMetaData[sensor_index][(*file_node_iterator)["name"].string()])->atFrameNumberSensorState(
                                current_frame_index, position_inertial, orientation_inertial, orientation_usk, offset,
                                sensor_fov);

                        (m_mapSensorNameToSensorMetaData[sensor_index][(*file_node_iterator)["name"].string()])->atFrameNumberVisibility(
                                current_frame_index, (int) (*file_node_iterator)["visible"]);
                    }
                }
            }
        }
        fs.release();
    }
}

void GroundTruthScene::startEvaluating(Noise noise) {

    calcBBFrom3DPosition();

    for (ushort obj_index = 0; obj_index < m_ptr_customObjectMetaDataList.at(0).size(); obj_index++) {

        GroundTruthObjects gt_obj;

        // how many objects were found. The assumption is that in each frame, the number of objects would be constant.

        for ( ushort sensor_index_group = 0; sensor_index_group < m_ptr_customSensorMetaDataList.size(); sensor_index_group++) {

            std::cout << "send object name " << m_ptr_customObjectMetaDataList.at(sensor_index_group).at(obj_index)->getObjectName() << std::endl;

            if ( sensor_index_group == 0 ) {
                gt_obj = GroundTruthObjects(m_ptr_customObjectMetaDataList.at(sensor_index_group).at(obj_index)->getObjectShape(),
                                          m_ptr_customObjectMetaDataList.at(sensor_index_group).at(obj_index)->getObjectStartPoint(), noise,
                                          m_ptr_customObjectMetaDataList.at(sensor_index_group).at(obj_index)->getObjectName());
                m_list_gt_objects.push_back(gt_obj);
            }
            // each object is monitored by n sensor group.
            m_list_gt_objects.at(obj_index).beginGroundTruthGeneration(*m_ptr_customObjectMetaDataList.at(sensor_index_group).at(obj_index));

        }
        if ( MAX_ALLOWED_SENSOR_GROUPS >= 2 ) {
            m_list_gt_objects.at(obj_index).generate_combined_sensor_data();
        }

    }

    for (ushort sen_index = 0; sen_index < m_ptr_customSensorMetaDataList.at(0).size(); sen_index++) {

        // how many sensors were found. The assumption is that in each frame, the number of sensors would be constant.


        for ( ushort sensor_index_group = 0; sensor_index_group < m_ptr_customSensorMetaDataList.size(); sensor_index_group++) {

            Sensors gt_sen(m_ptr_customSensorMetaDataList.at(0).at(sensor_index_group)->getSensorStartPoint(), &noise,
                           m_ptr_customSensorMetaDataList.at(0).at(sensor_index_group)->getSensorName());
            m_list_gt_sensors.push_back(gt_sen);
            // each sensor is monitored by n sensor group.

            m_list_gt_sensors.at(sen_index).beginGroundTruthGeneration(*m_ptr_customSensorMetaDataList.at(sensor_index_group).at(sen_index));

        }

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

    if (m_environment == "blue_sky") {


        if (!m_regenerate_yaml_file) { // dont generate, just read

            readPositionFromFile("../position_cpp.yml");

            ushort map_pair_count = 0;
            for (const auto &myPair : m_mapObjectNameToObjectMetaData[0]) {
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
            m_ptr_customObjectMetaDataList.at(0).push_back(&objectMetaDataList.at(0));

            achterbahn = Achterbahn(rectangle, "random_object", 120);
            achterbahn.process(Dataset::getFrameSize());
            objectMetaDataList.at(1) = achterbahn;
            m_ptr_customObjectMetaDataList.at(0).push_back(&objectMetaDataList.at(1));

        }

        startEvaluating(colorfulNoise);

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

    const ushort sensor_index = 0; // image is generated only once irrespective of skips.

    // apply black noise in case of night
    if (m_environment == "night") {
        BlackNoise noise;
        ObjectImageShapeData newCanvas = m_canvas.getCanvasShapeAndData();
        newCanvas.applyNoise(&noise);
        tempGroundTruthImageBase = newCanvas.get().clone();
    } else {
        tempGroundTruthImageBase = m_canvas.getCanvasShapeAndData().get().clone();
    }

    for (ushort current_frame_index = 0; current_frame_index < MAX_ITERATION_GT_SCENE_GENERATION_DATASET; current_frame_index++) {

        sprintf(file_name_image, "000%03d_10.png", current_frame_index);
        std::string input_image_file_with_path = m_generatepath.string() + "_" + std::to_string(0) + "/" + file_name_image; //+ file_name_image;


        //draw new ground truth image.
        tempGroundTruthImage = tempGroundTruthImageBase.clone();

        char sensor_index_folder_suffix[50];

        for (unsigned obj_index = 0; obj_index < m_ptr_customObjectMetaDataList.at(0).size(); obj_index++) {

            sprintf(sensor_index_folder_suffix, "%02d", m_list_gt_objects.at(obj_index).getObjectId());
            std::string position_image_file_with_path = m_position_object_path.string() +
                                                        sensor_index_folder_suffix + "/" + file_name_image;

            image_data_and_shape = m_list_gt_objects.at(obj_index).getImageShapeAndData().get().clone();
            image_data_and_shape = image_data_and_shape.rowRange(0, cvRound(m_list_gt_objects.at(
                    obj_index).getExtrapolatedGroundTruthDetails().at(sensor_index).at(
                    current_frame_index).m_region_of_interest_px.height_px)).colRange(0, cvRound(m_list_gt_objects.at(
                    obj_index).getExtrapolatedGroundTruthDetails().at(sensor_index).at(
                    current_frame_index).m_region_of_interest_px.width_px));
            positionShape = m_list_gt_objects.at(obj_index).getImageShapeAndData().get().clone();
            positionShape = positionShape.rowRange(0, cvRound(m_list_gt_objects.at(
                    obj_index).getExtrapolatedGroundTruthDetails().at(sensor_index).at(
                    current_frame_index).m_region_of_interest_px.height_px)).colRange(0, cvRound(m_list_gt_objects.at(
                    obj_index).getExtrapolatedGroundTruthDetails().at(sensor_index).at(
                    current_frame_index).m_region_of_interest_px.width_px));

            if ((!m_ptr_customObjectMetaDataList.at(0).at(0)->getAll().at(current_frame_index).occluded )) {

                image_data_and_shape.copyTo(tempGroundTruthImage(
                        cv::Rect(cvRound(m_list_gt_objects.at(obj_index).get_object_base_point_displacement().at(
                                current_frame_index).first.x),
                                 cvRound(m_list_gt_objects.at(obj_index).get_object_base_point_displacement().at(
                                         current_frame_index).first.y),
                                 cvRound(m_ptr_customObjectMetaDataList.at(0).at(0)->getAll().at(current_frame_index).m_region_of_interest_px.width_px),
                                 cvRound(m_ptr_customObjectMetaDataList.at(0).at(0)->getAll().at(current_frame_index).m_region_of_interest_px.height_px))));

            }
        }

        cv::imwrite(input_image_file_with_path, tempGroundTruthImage);

    }

}

void GroundTruthScene::generate_bird_view() {
    // the bird view needs the range information of each object
    // Assuming the camera is mounted on the floor.
    /*
        for ( auto position_index = 0;  position_index <  MAX_ITERATION_RESULTS; position_index++ ) {

            cv::Mat birdview_frame(Dataset::getFrameSize(), CV_32FC1);
            for ( auto object_index= 0; object_index < m_ptr_customObjectMetaDataList.at(0).size(); object_index++ ) {
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

void GroundTruthScene::calcBBFrom3DPosition() {

    cv::FileStorage write_fs;
    char file_name_image[50], file_name_image_output[50];

    cv::Mat image_data_and_shape;

    cv::Mat tempGroundTruthImage(Dataset::getFrameSize(), CV_8UC3);

    for ( ushort sensor_index = 0; sensor_index < m_ptr_customSensorMetaDataList.size(); sensor_index++) {

        char temp_str_fs[20];
        sprintf(temp_str_fs, "sensor_index_%03d", sensor_index);
        //write_fs << temp_str_fs << "[";

        unsigned long FRAME_COUNT = ITERATION_END_POINT;
        assert(FRAME_COUNT > 0);

        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

            tempGroundTruthImage = cv::Scalar::all(255);

            sprintf(file_name_image, "000%03d_10.png", current_frame_index );
            std::string input_image_file_with_path = m_generatepath.string() + "_" + std::to_string(sensor_index) + "/" + file_name_image;

            sprintf(file_name_image_output, "000%03d_10_bb.png", current_frame_index );
            //output_image_file_with_path = m_generatepath.string() + "stencil/" + file_name_image_output;

            cv::Mat tempGroundTruthImageBase = cv::imread(input_image_file_with_path, CV_LOAD_IMAGE_ANYCOLOR);
            if ( tempGroundTruthImageBase.data == NULL ) {
                std::cerr << input_image_file_with_path << " not found" << std::endl;
                throw ("No image file found error");
            }
            tempGroundTruthImage = cv::Scalar::all(255);
            tempGroundTruthImage = tempGroundTruthImageBase.clone();

            for (ushort obj_index = 0;
                 obj_index < m_ptr_customObjectMetaDataList.at(sensor_index).size(); obj_index++) {


                object_realworld_dim_m_str object_realworld_dim_m = m_ptr_customObjectMetaDataList.at(
                        sensor_index).at(obj_index)->getAll().at(
                        current_frame_index).m_object_realworld_dim_m;

                object_location_inertial_m_str pos_object_inertial = m_ptr_customObjectMetaDataList.at(
                        sensor_index).at(obj_index)->getAll().at(
                        current_frame_index).m_object_location_inertial_m;

                object_location_m_str object_location_m = m_ptr_customObjectMetaDataList.at(sensor_index).at(
                        obj_index)->getAll().at(
                        current_frame_index).m_object_location_m;

                object_rotation_inertial_rad_str orientation_object_inertial = m_ptr_customObjectMetaDataList.at(
                        sensor_index).at(obj_index)->getAll().at(
                        current_frame_index).m_object_rotation_inertial_rad;

                sensor_location_carrier_m_str pos_sensor_carrier_inertial = m_ptr_customSensorMetaDataList.at(
                        sensor_index).at(0)->getAll().at(current_frame_index).m_sensor_location_carrier_m;

                sensor_rotation_carrier_rad_str sensor_rotation_carrier_rad = m_ptr_customSensorMetaDataList.at(
                        sensor_index).at(0)->getAll().at(
                        current_frame_index).m_sensor_rotation_carrier_rad;

                sensor_offset_m_str sensor_offset_m = m_ptr_customSensorMetaDataList.at(sensor_index).at(
                        2)->getAll().at(
                        current_frame_index).m_sensor_offset_m;

                float offset_x = m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                        current_frame_index).m_object_offset_m.offset_x;
                float offset_y = m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                        current_frame_index).m_object_offset_m.offset_y;
                float offset_z = m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                        current_frame_index).m_object_offset_m.offset_z;

                if (m_ptr_customObjectMetaDataList.at(
                        sensor_index).at(obj_index)->getAll().at(current_frame_index).visMask && pos_object_inertial.location_x_m != 0) {

                    std::vector<cv::Point3f> bounding_points_3d(9);
                    std::vector<cv::Point2f> bounding_points_2d(9);


                    /*
                     * assuming vehicle co-ordinate system of the object.
                     * create a box placed on z = 0 ( street level ). Find 8 points of the box.
                     *
                     */

                    bounding_points_3d.at(0) = cv::Point3f(object_realworld_dim_m.dim_length_m / 2,
                            object_realworld_dim_m.dim_width_m / 2, 0);
                    bounding_points_3d.at(1) = cv::Point3f(-object_realworld_dim_m.dim_length_m / 2,
                            object_realworld_dim_m.dim_width_m / 2, 0);
                    bounding_points_3d.at(2) = cv::Point3f(object_realworld_dim_m.dim_length_m / 2,
                            -object_realworld_dim_m.dim_width_m / 2, 0);
                    bounding_points_3d.at(3) = cv::Point3f(-object_realworld_dim_m.dim_length_m / 2,
                            -object_realworld_dim_m.dim_width_m / 2, 0);
                    bounding_points_3d.at(4) = cv::Point3f(object_realworld_dim_m.dim_length_m / 2,
                            object_realworld_dim_m.dim_width_m / 2,
                            object_realworld_dim_m.dim_height_m);
                    bounding_points_3d.at(5) = cv::Point3f(-object_realworld_dim_m.dim_length_m / 2,
                            object_realworld_dim_m.dim_width_m / 2,
                            object_realworld_dim_m.dim_height_m);
                    bounding_points_3d.at(6) = cv::Point3f(object_realworld_dim_m.dim_length_m / 2,
                            -object_realworld_dim_m.dim_width_m / 2,
                            object_realworld_dim_m.dim_height_m);
                    bounding_points_3d.at(7) = cv::Point3f(-object_realworld_dim_m.dim_length_m / 2,
                            -object_realworld_dim_m.dim_width_m / 2,
                            object_realworld_dim_m.dim_height_m);
                    bounding_points_3d.at(8) = cv::Point3f(0, 0, object_realworld_dim_m.dim_height_m /
                            2); // This is the center of gravity

                    cv::Point3f final;

                    sensor_fov_rad_str fov_rad = m_ptr_customSensorMetaDataList.at(sensor_index).at(0)->getAll().at(
                            current_frame_index).m_sensor_fov_rad;


                    for (auto i = 0; i < 9; i++) {

                        //Add the offset for each point. These points are in the vehicle coordinate system.
                        final = Utils::translate_and_rotate_points(bounding_points_3d.at(i),
                                cv::Point3f(offset_x, offset_y, offset_z),
                                cv::Point3f(0, 0, 0));

                        //Then rotate the box to inertial coordinate system. hpr. Now the BB points are in the inertial co-ordinate system with the origin at the position.
                        final = Utils::translate_and_rotate_points(final, cv::Point3f(0, 0, 0),
                                cv::Point3f(
                                        orientation_object_inertial.rotation_rz_yaw_rad,
                                        orientation_object_inertial.rotation_ry_pitch_rad,
                                        orientation_object_inertial.rotation_rx_roll_rad));

                        //Translate the axis to the inertial origin. add the BB vector to the object position.
                        //Now we are in the inertial co-ordinate system.
                        final = Utils::translate_and_rotate_points(final, cv::Point3f(pos_object_inertial.location_x_m,
                                pos_object_inertial.location_y_m,
                                pos_object_inertial.location_z_m),
                                cv::Point3f(0, 0, 0));

                        // Change to sensor object by changing the axis to the sensor object.
                        final = Utils::translate_and_rotate_points(final, cv::Point3f(
                                -pos_sensor_carrier_inertial.location_x_m, -pos_sensor_carrier_inertial.location_y_m,
                                -pos_sensor_carrier_inertial.location_z_m), cv::Point3f(0, 0, 0));


                        // now rotate the axis to align to the car
                        final = Utils::translate_and_rotate_points(final, cv::Point3f(0, 0, 0), cv::Point3f(
                                -sensor_rotation_carrier_rad.rotation_rz_yaw_rad,
                                -sensor_rotation_carrier_rad.rotation_ry_pitch_rad,
                                -sensor_rotation_carrier_rad.rotation_rx_roll_rad));

                        // We are in the vehicle coordinate system now.
                        // Translate to cam position in the car.
                        final = Utils::translate_and_rotate_points(final, cv::Point3f(-sensor_offset_m.offset_x,
                                -sensor_offset_m.offset_y,
                                -sensor_offset_m.offset_z),
                                cv::Point3f(0, 0, 0));

                        // The resulting points are the bounding box points in the USK co-ordinate system.
                        bounding_points_3d.at(i) = final;

                        cv::Point2f camPoint = Utils::worldToCameraIntrinsc(final, fov_rad.vertical, 600, 600);
                        //cv::Point2f camPoint_openglfrustum = Utils::worldToCamera(final, fov_rad.vertical, 600, 600);
                        bounding_points_2d.at(i) = cv::Point2f(camPoint.x, camPoint.y);

                    }

                    std::cout << "cam" << cv::Point2f(bounding_points_3d.at(8).x, bounding_points_3d.at(8).y)
                            << std::endl;
                    std::cout << "usk"
                            << cv::Point2f(m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                                    current_frame_index).m_object_location_m.location_x_m,
                                    m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                                            current_frame_index).m_object_location_m.location_y_m) << std::endl;

                    auto dist = cv::norm(cv::Point2f(bounding_points_3d.at(8).x + sensor_offset_m.offset_x,
                            bounding_points_3d.at(8).y + sensor_offset_m.offset_y));
                    auto dist_usk = cv::norm(
                            cv::Point2f(m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                                    current_frame_index).m_object_location_m.location_x_m,
                                    m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                                            current_frame_index).m_object_location_m.location_y_m));
                    assert(std::abs(dist - dist_usk) < 1.5);
                    std::cout << "distance is " << dist << " for "
                            << m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getObjectName()
                            << std::endl;

                    m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->setBoundingBoxPoints(current_frame_index,
                            bounding_points_2d);

                    cv::Point2f xx = Utils::worldToCameraIntrinsc(
                            cv::Point3f(object_location_m.location_y_m, object_location_m.location_z_m,
                                    object_location_m.location_x_m), fov_rad.vertical, 980, 980);

                    //if (( !m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(current_frame_index).occluded )) {

                    cv::Rect boundingbox = cv::Rect(
                            cvRound(xx.x),
                            cvRound(xx.y),
                            cvRound(m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                                    current_frame_index).m_object_dimensions_px.width_px),
                            cvRound(m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                                    current_frame_index).m_object_dimensions_px.height_px));

                    //cv::rectangle(tempGroundTruthImage, boundingbox, cv::Scalar(0, 255, 0), 1, 8, 0);

                    std::vector<cv::Point2f> box = {
                            m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                                    current_frame_index).m_bounding_box.bb_lower_bottom_px,
                            m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                                    current_frame_index).m_bounding_box.bb_lower_left_px,
                            m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                                    current_frame_index).m_bounding_box.bb_lower_top_px,
                            m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                                    current_frame_index).m_bounding_box.bb_lower_right_px,
                            m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                                    current_frame_index).m_bounding_box.bb_higher_bottom_px,
                            m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                                    current_frame_index).m_bounding_box.bb_higher_left_px,
                            m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                                    current_frame_index).m_bounding_box.bb_higher_top_px,
                            m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                                    current_frame_index).m_bounding_box.bb_higher_right_px
                    };

                    for (auto i = 0; i < 8; i++) {
                        //std::cout << box.at(i) << std::endl;
                        cv::circle(tempGroundTruthImage, box.at(i), 2, cv::Scalar(0, 0, 255), 3);
                    }
                    cv::circle(tempGroundTruthImage, cv::Point2i(
                            m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                                    current_frame_index).m_object_location_px.cog_px.x,
                            m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getAll().at(
                                    current_frame_index).m_object_location_px.cog_px.y), 2, cv::Scalar(0, 255, 0), 4);
                    cv::Rect box_points = cv::boundingRect(box);
                    cv::rectangle(tempGroundTruthImage, box_points, cv::Scalar(0, 0, 255), 1, 8, 0);

                    //}
                }
                else {
                    std::cout << m_ptr_customObjectMetaDataList.at(sensor_index).at(obj_index)->getObjectName() << " is occlued" << std::endl;
                }
            }

            //cv::namedWindow("BB", CV_WINDOW_AUTOSIZE);
            //cv::imshow("BB", tempGroundTruthImage);
            //cv::waitKey(0);
            //cv::destroyAllWindows();
            //cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
            /*---------------------------------------------------------------------------------*/

        }
    }
}

void GroundTruthSceneExternal::generate_gt_scene() {


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

        if (m_environment == "blue_sky") {
            sprintf(command, "cd %s../../ ; bash vtdSendandReceive.sh %s", (m_datasetpath.string()).c_str(),
                    project.c_str());
            std::cout << command << std::endl;
            system(command);
            std::cout << " I am out of bash" << std::endl;
        }

        sleep(5); // Give some time before you send SCP commands.

        // std::string m_server;
        boost::filesystem::path m_ts_gt_out_dir;


        //fprintf(stderr, "ValidateArgs: key = 0x%x, checkMask = 0x%x, mForceBuffer = %d\n", viresObjects.at(0).getShmKey(), getCheckMask(),
        //        getForceBuffer());

        bool connected_trigger_port = false;
        bool connected_scp_port = false;
        bool connected_module_manager_port = false;

        m_scpSocket = openNetwork(SCP_DEFAULT_PORT);

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

        for ( ushort i = 0; i < MAX_ALLOWED_SENSOR_GROUPS; i++ ) {

            for ( ushort j = 0; j < 3; j++ ) {
                sendSCPMessage(m_scpSocket, viresObjects.at(i).getSensorConfiguration().at(j).get<0>().c_str());

                sleep(1);
            }
        }

        //sendSCPMessage(m_scpSocket, view_parameters_sensorpoint_openglfrustum.c_str());
        sendSCPMessage(m_scpSocket, view_parameters_sensorpoint_intrinsicparams_left.c_str());
        sleep(1);

        if ( MAX_ALLOWED_SENSOR_GROUPS > 1 ) {
            sendSCPMessage(m_scpSocket, view_parameters_sensorpoint_intrinsicparams_right.c_str());
            sleep(1);
        }

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

        m_triggerSocket = openNetwork(DEFAULT_PORT);

        std::cout << "trigger socket - " << m_triggerSocket << std::endl;
        if (m_triggerSocket != -1) { // this is blocking until the network has been opened
            connected_trigger_port = true;
        }

        for (ushort i = 0 ; i <= MAX_ALLOWED_SENSOR_GROUPS ; i++ ) {
            viresObjects.at(i).openNetworkSockets();
            std::cout << "mm socket - " << viresObjects.at(0).getSensorConfiguration().at(0).get<1>() << std::endl;
            if (viresObjects.at(i).getCameraSocketHandler() != -1 &&
                viresObjects.at(i).getPerfectSocketHandler() != -1 &&
                viresObjects.at(i).getPerfectInertialSocketHandler() != -1 ) {
                connected_module_manager_port = true;
            }
            else {
                connected_module_manager_port = false;
                break;
            }
        }


        if (connected_trigger_port && connected_module_manager_port && connected_scp_port) {

            // open the shared memory for IG image output (try to attach without creating a new segment)
            fprintf(stderr, "openCommunication: attaching to shared memory (IG image output) 0x%x....\n", viresObjects.at(0).getShmKey());

            for (ushort i = 0 ; i <= MAX_ALLOWED_SENSOR_GROUPS ; i++ ) {
                viresObjects.at(i).openShmWrapper();
                usleep(1000);     // do not overload the CPU
            }

            // now check the SHM for the time being
            bool breaking = false;
            int count = 0;

            try {
                while (1) {

                    if (viresObjects.at(0).getBreaking()) {
                        break;
                    }

                    viresObjects.at(0).getGroundTruthInformation(true, m_triggerSocket, (m_environment == "blue_sky"));
                    viresObjects.at(1).getGroundTruthInformation(false, m_triggerSocket, (m_environment == "blue_sky"));

                    usleep(100000); // wait, 100 ms which is equivalent to 10 Hz. Normally VIRES runs with 60 Hz. So this number should not be a problem.
                    //std::cout << "getting data from VIRES\n";
                }
            }
            catch (...) {
                std::cerr << "Error in generation" << std::endl;
                stopSimulation();
                return;
            };

            configVires();
        }
    }

    Noise noNoise;

    if (m_environment == "blue_sky") {

        if (m_regenerate_yaml_file) {

            try {
                for ( ushort i = 0 ; i < MAX_ALLOWED_SENSOR_GROUPS ; i++) {
                    viresObjects.at(i).writePositionInYaml("vires");
                }
            }
            catch (...) {
                std::cerr << "VTD Generation complete, but error in generating images" << std::endl;
                stopSimulation();
            }

        } else { // dont generate, just read

            readPositionFromFile("dummy");

            ushort map_pair_count = 0;
            for (const auto &myPair : m_mapObjectNameToObjectMetaData[0]) {
                std::cout << myPair.first << "\n";
                map_pair_count++;
            }

            startEvaluating(noNoise);
        }
    }
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

