//
// Created by veikas on 10.02.18.
//

#include <opencv2/core/types.hpp>
#include <opencv2/core/persistence.hpp>
#include <assert.h>
#include <png++/rgb_pixel.hpp>
#include <png++/image.hpp>
#include "ViresObjects.h"
#include "datasets.h"
#include "Utils.h"


void BasicObjects::calcBBFrom3DPosition() {

    cv::FileStorage write_fs;
    char file_name_image[50], file_name_image_output[50];

    cv::Mat image_data_and_shape;

    cv::Mat tempGroundTruthImage(Dataset::getFrameSize(), CV_8UC3);

    unsigned long FRAME_COUNT = ITERATION_END_POINT;
    assert(FRAME_COUNT > 0);

    for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

        tempGroundTruthImage = cv::Scalar::all(255);

        sprintf(file_name_image, "000%03d_10.png", current_frame_index );
        std::string input_image_file_with_path = m_generatepath.string() + "_" + std::to_string(m_sensorGroupCount) + "/" + file_name_image;

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
             obj_index < m_ptr_customObjectMetaDataList.size(); obj_index++) {


            object_realworld_dim_m_str object_realworld_dim_m = m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_dimension_realworld_m;

            object_location_inertial_m_str pos_object_inertial = m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_location_inertial_m;

            object_location_m_str object_location_m = m_ptr_customObjectMetaDataList.at(
                    obj_index)->getAll().at(
                    current_frame_index).m_object_location_usk_m;

            object_rotation_inertial_rad_str orientation_object_inertial = m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_rotation_inertial_rad;

            sensor_location_carrier_m_str pos_sensor_carrier_inertial = m_ptr_customSensorMetaDataList.at(0)->getAll().at(current_frame_index).m_sensor_location_carrier_m;

            sensor_rotation_carrier_rad_str sensor_rotation_carrier_rad = m_ptr_customSensorMetaDataList.at(0)->getAll().at(
                    current_frame_index).m_sensor_rotation_carrier_rad;

            sensor_offset_m_str sensor_offset_m = m_ptr_customSensorMetaDataList.at(
                    2)->getAll().at(
                    current_frame_index).m_sensor_offset_m;

            float offset_x = m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_offset_m.offset_x;
            float offset_y = m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_offset_m.offset_y;
            float offset_z = m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_offset_m.offset_z;

            if (m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).visMask && pos_object_inertial.location_x_m != 0) {

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

                sensor_fov_rad_str fov_rad = m_ptr_customSensorMetaDataList.at(0)->getAll().at(
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

                    cv::Point2f camPoint = Utils::worldToCameraIntrinsc(final, fov_rad.vertical, FOCAL_X, FOCAL_Y);
                    //cv::Point2f camPoint_openglfrustum = Utils::worldToCamera(final, fov_rad.vertical, FOCAL_X, FOCAL_Y);
                    bounding_points_2d.at(i) = cv::Point2f(camPoint.x, camPoint.y);

                }

                std::cout << "converting inertial to usk" << cv::Point2f(bounding_points_3d.at(8).x, bounding_points_3d.at(8).y)
                          << std::endl;
                std::cout << "original usk"
                          << cv::Point2f(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                  current_frame_index).m_object_location_usk_m.location_x_m,
                                         m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                                 current_frame_index).m_object_location_usk_m.location_y_m) << std::endl;

                auto dist_usk_from_inertial = cv::norm(cv::Point2f(bounding_points_3d.at(8).x + sensor_offset_m.offset_x,
                                                                   bounding_points_3d.at(8).y + sensor_offset_m.offset_y));
                auto dist_usk_original = cv::norm(
                        cv::Point2f(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                current_frame_index).m_object_location_usk_m.location_x_m,
                                    m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                            current_frame_index).m_object_location_usk_m.location_y_m));
                assert(std::abs(dist_usk_from_inertial - dist_usk_original) < 0.5);
                std::cout << "distance is " << dist_usk_from_inertial << " for "
                          << m_ptr_customObjectMetaDataList.at(obj_index)->getObjectName()
                          << std::endl;

                m_ptr_customObjectMetaDataList.at(obj_index)->setBoundingBoxPoints(current_frame_index,
                                                                                                    bounding_points_2d);

                cv::Point2f xx = Utils::worldToCameraIntrinsc(
                        cv::Point3f(object_location_m.location_y_m, object_location_m.location_z_m,
                                    object_location_m.location_x_m), fov_rad.vertical, 980, 980);

                //if (( !m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).occluded )) {

                cv::Rect boundingbox = cv::Rect(
                        cvRound(xx.x),
                        cvRound(xx.y),
                        cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                current_frame_index).m_object_dimension_camera_px.width_px),
                        cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                current_frame_index).m_object_dimension_camera_px.height_px));

                //cv::rectangle(tempGroundTruthImage, boundingbox, cv::Scalar(0, 255, 0), 1, 8, 0);

                std::vector<cv::Point2f> box = {
                        m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                current_frame_index).m_bounding_box.bb_lower_bottom_px,
                        m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                current_frame_index).m_bounding_box.bb_lower_left_px,
                        m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                current_frame_index).m_bounding_box.bb_lower_top_px,
                        m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                current_frame_index).m_bounding_box.bb_lower_right_px,
                        m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                current_frame_index).m_bounding_box.bb_higher_bottom_px,
                        m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                current_frame_index).m_bounding_box.bb_higher_left_px,
                        m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                current_frame_index).m_bounding_box.bb_higher_top_px,
                        m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                current_frame_index).m_bounding_box.bb_higher_right_px
                };

                for (auto i = 0; i < 8; i++) {
                    //std::cout << box.at(i) << std::endl;
                    cv::circle(tempGroundTruthImage, box.at(i), 2, cv::Scalar(0, 0, 255), 3);
                }
                cv::circle(tempGroundTruthImage, cv::Point2i(
                        m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                current_frame_index).m_object_location_camera_px.cog_px.x,
                        m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                current_frame_index).m_object_location_camera_px.cog_px.y), 2, cv::Scalar(0, 255, 0), 4);
                cv::Rect box_points = cv::boundingRect(box);
                cv::rectangle(tempGroundTruthImage, box_points, cv::Scalar(0, 0, 255), 1, 8, 0);

                //}
            }
            else {
                std::cout << m_ptr_customObjectMetaDataList.at(obj_index)->getObjectName() << " is occlued" << std::endl;
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


void ViresObjects::readPositionFromFile(std::string positionFileName) {


    cv::Point2f speed_usk, speed_inertial;
    cv::Point2f dimension_pixel, sensor_fov;
    cv::Point3f offset, position_inertial, position_usk, position_pixel, dimension_realworld;
    cv::Point3f position_inertial_pre, position_usk_pre, position_pixel_pre;
    float totalDistanceTravelled;

    cv::Point3f orientation_inertial, orientation_usk;


    cv::FileNode file_node;
    cv::FileNodeIterator file_node_iterator_begin, file_node_iterator_end, file_node_iterator;


    ushort  objectCount = 0;
    ushort  sensorCount = 0;
    cv::FileStorage fs;
    fs.open("../position_vires_" + std::to_string(m_sensorGroupCount) + ".yml", cv::FileStorage::READ);

    assert(fs.isOpened());

    //std::string temp_str = "_sensor_count_" + sensor_index;
    std::cout << "read yaml file for sensor_index " << (m_sensorGroupCount) << std::endl;
    //unsigned long FRAME_COUNT = m_list_gt_objects.at(0).get_object_shape_point_displacement().size();
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

                    if (m_mapObjectNameToObjectMetaData.count((*file_node_iterator)["name"].string()) == 0
                            ) {
                        m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(m_objectCount));
                        m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()] = m_ptr_customObjectMetaDataList.at(
                                objectCount);
                        m_ptr_customObjectMetaDataList.at(objectCount)->setObjectName(
                                (*file_node_iterator)["name"].string());
                        Rectangle rectangle(Dataset::getFrameSize().width,
                                            Dataset::getFrameSize().height); // width, height
                        m_ptr_customObjectMetaDataList.at(objectCount)->setObjectShape(rectangle);
                        m_ptr_customObjectMetaDataList.at(objectCount)->setStartPoint(ITERATION_START_POINT);
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

                    (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberFrameCount(
                            current_frame_index);
                    (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberCameraSensor(
                            current_frame_index, position_pixel, offset, dimension_pixel, totalDistanceTravelled);
                    (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberPerfectSensor(
                            current_frame_index, position_usk, orientation_usk, dimension_realworld, speed_usk,
                            totalDistanceTravelled);
                    (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberPerfectSensorInertial(
                            current_frame_index, position_inertial, orientation_inertial, dimension_realworld,
                            speed_inertial, totalDistanceTravelled);

                    (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberVisibility(
                            current_frame_index, (int) (*file_node_iterator)["visMask"]);

                    (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberOcclusionWindow(
                            current_frame_index, (int) (*file_node_iterator)["occ_px"]);

                    (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberOcclusionUsk(
                            current_frame_index, (int) (*file_node_iterator)["occ_usk"]);

                    (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberOcclusionInertial(
                            current_frame_index, (int) (*file_node_iterator)["occ_inertial"]);

                    position_inertial_pre = position_inertial;
                    position_pixel_pre = position_pixel;
                    position_usk_pre = position_usk;

                }
                else
                {
                    if (m_mapSensorNameToSensorMetaData.count((*file_node_iterator)["name"].string()) == 0) {
                        m_ptr_customSensorMetaDataList.push_back(&sensorMetaDataList.at(m_sensorCount));
                        m_mapSensorNameToSensorMetaData[(*file_node_iterator)["name"].string()] = m_ptr_customSensorMetaDataList.at(sensorCount);
                        m_ptr_customSensorMetaDataList.at(sensorCount)->setSensorName(
                                (*file_node_iterator)["name"].string());
                        m_ptr_customSensorMetaDataList.at(sensorCount)->setStartPoint(0);
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

                    (m_mapSensorNameToSensorMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberSensorState(
                            current_frame_index, position_inertial, orientation_inertial, orientation_usk, offset,
                            sensor_fov);

                    (m_mapSensorNameToSensorMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberVisibility(
                            current_frame_index, (int) (*file_node_iterator)["visible"]);
                }
            }
        }
    }
    fs.release();
}


void ViresObjects::readObjectStateFromBinaryFile(std::string suffix) {

    cv::Point3f offset, position_inertial, position_usk, position_pixel, dimension_realworld;
    cv::Point2f dimension_pixel;
    cv::Point2f speed_inertial, speed_usk;
    cv::Point3f orientation_usk, orientation_inertial;
    float dist_cam_to_obj;
    float total_distance_travelled;


    RDB_OBJECT_STATE_t object_state_data;
    RDB_OBJECT_STATE_t *data = &object_state_data;

    ushort frame_number;

    std::ifstream fstream_input_object_state = std::ifstream("../object_state_" + std::to_string(m_sensorGroupCount) + ".bin", std::ios::binary);

    while ( !fstream_input_object_state.eof() ) {
        char marker[3];
        fstream_input_object_state.read(marker, sizeof(marker - 1));
        marker[2] = '\0';
        if (strcmp(marker, "$$") != 0) {
            std::cout << "not in sync" << std::endl;
            throw;
        }
        fstream_input_object_state.read((char *) &frame_number, sizeof(ushort));
        fstream_input_object_state.read((char *) data, sizeof(RDB_OBJECT_STATE_t));

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

        m_mapObjectNameToObjectMetaData[data->base.name]->atAllObjectStateData(
                (ushort) frame_number, data);


        if (data->base.pos.type == RDB_COORD_TYPE_WINDOW) {

            position_pixel = cv::Point3f((float) data->base.pos.x, (float) data->base.pos.y,
                                         float(data->base.pos.z));
            dimension_pixel = cv::Point2f((float) data->base.geo.dimX, (float) data->base.geo.dimY);
            offset = cv::Point3f((float) data->base.geo.offX, (float) data->base.geo.offY,
                                 (float) data->base.geo.offZ);
            total_distance_travelled = data->ext.traveledDist;


            m_mapObjectNameToObjectMetaData[data->base.name]->atFrameNumberCameraSensor(
                    (ushort) frame_number, position_pixel, offset,
                    dimension_pixel, total_distance_travelled);
            m_mapObjectNameToObjectMetaData[data->base.name]->atFrameNumberVisibility(
                    (ushort) frame_number, data->base.visMask);

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
                    (ushort) frame_number, position_usk,
                    orientation_usk, dimension_realworld, speed_usk, total_distance_travelled);
            m_mapObjectNameToObjectMetaData[data->base.name]->atFrameNumberVisibility(
                    (ushort) frame_number, data->base.visMask);

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
                    (ushort) frame_number, position_inertial,
                    orientation_inertial, dimension_realworld, speed_inertial, total_distance_travelled);
            m_mapObjectNameToObjectMetaData[data->base.name]->atFrameNumberVisibility(
                    (ushort) frame_number, data->base.visMask);
            }
        }
    }

    void ViresObjects::readSensorObjectFromBinaryFile(std::string suffix) {

        RDB_SENSOR_OBJECT_t sensor_object_data;
        RDB_SENSOR_OBJECT_t *data = &sensor_object_data;

        ushort frame_number;

        std::ifstream fstream_input_sensor_object = std::ifstream("../sensor_object_" + std::to_string(m_sensorGroupCount) + ".bin", std::ios::binary);

        while ( !fstream_input_sensor_object.eof() ) {

            char marker[3];
            fstream_input_sensor_object.read(marker, sizeof(marker - 1));
            marker[2] = '\0';
            if (strcmp(marker, "$$") != 0) {
                std::cout << "not in sync" << std::endl;
                throw;
            }
            fstream_input_sensor_object.read((char *) &frame_number, sizeof(ushort));
            fstream_input_sensor_object.read((char *) data, sizeof(RDB_SENSOR_OBJECT_t));

            if (m_mapObjectNameToObjectMetaData.count(m_mapObjectIdToObjectName[data->id]) == 0) {

                m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(m_objectCount));
                //Rectangle rectangle((int) (data->geo.dimX), (int) (data->geo.dimY)); // width, height
                m_mapObjectNameToObjectMetaData[m_mapObjectIdToObjectName[data->id]] = m_ptr_customObjectMetaDataList.at(
                        m_objectCount);
                //m_ptr_customObjectMetaDataList.at(m_objectCount)->setObjectShape(rectangle);
                m_ptr_customObjectMetaDataList.at(m_objectCount)->setObjectName(m_mapObjectIdToObjectName[data->id]);
                m_ptr_customObjectMetaDataList.at(m_objectCount)->setStartPoint(0);
                m_objectCount += 1;
            }
            if (data->sensorPos.type == RDB_COORD_TYPE_WINDOW) {

                m_mapObjectNameToObjectMetaData[m_mapObjectIdToObjectName[data->id]]->atFrameNumberOcclusionWindow(
                        (ushort) frame_number, data->occlusion);
            } else if (data->sensorPos.type == RDB_COORD_TYPE_USK) {

                m_mapObjectNameToObjectMetaData[m_mapObjectIdToObjectName[data->id]]->atFrameNumberOcclusionUsk(
                        (ushort) frame_number, data->occlusion);
            } else if (data->sensorPos.type == RDB_COORD_TYPE_INERTIAL) {

                m_mapObjectNameToObjectMetaData[m_mapObjectIdToObjectName[data->id]]->atFrameNumberOcclusionInertial(
                        (ushort) frame_number, data->occlusion);
            }
        }
    }


void ViresObjects::readSensorStateFromBinaryFile(std::string suffix) {

    cv::Point3f position_sensor_carrier, orientation_sensor_carrier, position_sensor, orientation_sensor, offset_sensor;
    cv::Point2f fov;

    RDB_SENSOR_STATE_t sensor_state_data;
    RDB_SENSOR_STATE_t *data = &sensor_state_data;

    ushort frame_number;

    std::ifstream fstream_input_sensor_state = std::ifstream("../sensor_state_" + std::to_string(m_sensorGroupCount) + ".bin", std::ios::binary);

    while ( !fstream_input_sensor_state.eof() ) {
        char marker[3];
        fstream_input_sensor_state.read(marker, sizeof(marker - 1));
        marker[2] = '\0';
        if (strcmp(marker, "$$") != 0) {
            std::cout << "not in sync" << std::endl;
            throw;
        }
        fstream_input_sensor_state.read((char *) &frame_number, sizeof(ushort));
        fstream_input_sensor_state.read((char *) data, sizeof(RDB_SENSOR_STATE_t));
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
                (ushort) frame_number, position_sensor_carrier,
                orientation_sensor_carrier, orientation_sensor, offset_sensor, fov);
    }

}



void ViresObjects::writePositionInYaml(std::string suffix) {

    cv::FileStorage write_fs;
    write_fs.open("../position_" + suffix + std::to_string(m_sensorGroupCount) + ".yml", cv::FileStorage::WRITE);

    unsigned long FRAME_COUNT = MAX_ITERATION_GT_SCENE_GENERATION_DATASET;
    assert(FRAME_COUNT > 0);

    for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {
        char temp_str_fc[20];
        sprintf(temp_str_fc, "frame_count_%03d", current_frame_index);

        //if ( current_frame_index < 25 ) {
        //    continue;
        //}

        write_fs << temp_str_fc << "[";
        for (int obj_index = 0; obj_index < m_ptr_customObjectMetaDataList.size(); obj_index++) {
            write_fs

                    << "{:" << "name" << m_ptr_customObjectMetaDataList.at(obj_index)->getObjectName()

                    << "visMask" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).visMask
                    << "x_camera" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_location_camera_px.location_x_px
                    << "y_camera" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_location_camera_px.location_y_px
                    << "z_camera" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_location_camera_px.location_z_px

                    << "x_inertial" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_location_inertial_m.location_x_m
                    << "y_inertial" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_location_inertial_m.location_y_m
                    << "z_inertial" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_location_inertial_m.location_z_m

                    << "x_usk" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_location_usk_m.location_x_m
                    << "y_usk" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_location_usk_m.location_y_m
                    << "z_usk" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_location_usk_m.location_z_m

                    << "dim_x_camera" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_dimension_camera_px.width_px
                    << "dim_y_camera" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_dimension_camera_px.height_px

                    << "dim_x_realworld" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_dimension_realworld_m.dim_length_m
                    << "dim_y_realworld" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_dimension_realworld_m.dim_width_m
                    << "dim_z_realworld" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_dimension_realworld_m.dim_height_m

                    << "speed_x_inertial"
                    << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_speed_inertial.x
                    << "speed_y_inertial"
                    << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_speed_inertial.y
                    << "speed_z_inertial"
                    << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_speed_inertial.z

                    << "speed_x" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_speed.x
                    << "speed_y" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_speed.y

                    << "off_x"
                    << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_offset_m.offset_x
                    << "off_y"
                    << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_offset_m.offset_y
                    << "off_z"
                    << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_offset_m.offset_z

                    << "h_inertial" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_rotation_inertial_rad.rotation_rz_yaw_rad
                    << "p_inertial" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_rotation_inertial_rad.rotation_ry_pitch_rad
                    << "r_inertial" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_rotation_inertial_rad.rotation_rx_roll_rad

                    << "h_usk" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_rotation_rad.rotation_rz_yaw_rad
                    << "p_usk" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_rotation_rad.rotation_ry_pitch_rad
                    << "r_usk" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_rotation_rad.rotation_rx_roll_rad

                    << "dist_cam_to_obj" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_distances.sensor_to_obj
                    << "total_distance_covered" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_distances.total_distance_covered

                    << "occ_px" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_occlusion.occlusion_px
                    << "occ_usk" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_occlusion.occlusion_usk
                    << "occ_inertial" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_occlusion.occlusion_inertial


                    << "}";  //dont close the brace yet
        }
        for (int sen_index = 0; sen_index < m_ptr_customSensorMetaDataList.size(); sen_index++) {
            write_fs

                    << "{:" << "name" << m_ptr_customSensorMetaDataList.at(sen_index)->getSensorName()

                    << "visible" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(current_frame_index).visMask

                    << "x_carrier" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                    current_frame_index).m_sensor_location_carrier_m.location_x_m
                    << "y_carrier" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                    current_frame_index).m_sensor_location_carrier_m.location_y_m
                    << "z_carrier" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                    current_frame_index).m_sensor_location_carrier_m.location_z_m

                    //<< "x_sensor" <<  m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(current_frame_index).m_sensor_location_m.location_x_m
                    //<< "y_sensor" <<  m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(current_frame_index).m_sensor_location_m.location_y_m
                    //<< "z_sensor" <<  m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(current_frame_index).m_sensor_location_m.location_z_m

                    << "h_carrier" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                    current_frame_index).m_sensor_rotation_carrier_rad.rotation_rz_yaw_rad
                    << "p_carrier" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                    current_frame_index).m_sensor_rotation_carrier_rad.rotation_ry_pitch_rad
                    << "r_carrier" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                    current_frame_index).m_sensor_rotation_carrier_rad.rotation_rx_roll_rad

                    << "h_sensor" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                    current_frame_index).m_sensor_rotation_rad.rotation_rz_yaw_rad
                    << "p_sensor" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                    current_frame_index).m_sensor_rotation_rad.rotation_ry_pitch_rad
                    << "r_sensor" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                    current_frame_index).m_sensor_rotation_rad.rotation_rx_roll_rad

                    << "off_x_sensor"
                    << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(current_frame_index).m_sensor_offset_m.offset_x
                    << "off_y_sensor"
                    << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(current_frame_index).m_sensor_offset_m.offset_y
                    << "off_z_sensor"
                    << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(current_frame_index).m_sensor_offset_m.offset_z

                    << "fov_horizontal"
                    << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(current_frame_index).m_sensor_fov_rad.horizontal
                    << "fov_vertical"
                    << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(current_frame_index).m_sensor_fov_rad.vertical
                    << "fov_horizontal_off" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                    current_frame_index).m_sensor_fov_rad.horizontal_offset
                    << "fov_vertical_off" << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(
                    current_frame_index).m_sensor_fov_rad.vertical_offset

                    << "}";
        }

        write_fs << "]";
    }
    //write_fs << "]";
    write_fs.release();

}


void ViresObjects::getGroundTruthInformation(void* shmPtr, bool withTrigger, int triggerSocket, bool getGroundTruthData, bool getGroundTruthImages,
ushort m_moduleManagerSocket_Camera, ushort m_moduleManagerSocket_Perfect, ushort m_moduleManagerSocket_PerfectInertial) {

    fprintf(stderr,
            "------------------------------------------------------------------------------------\n");
    float deltaTime;

    if (m_dumpInitialFrames) {

        deltaTime = 0.03;
        mCheckForImage = false;
        if ( withTrigger) {
            fprintf(stderr, "sendRDBTrigger: sending trigger, deltaT = %.4lf, requestImage = false\n",
                    deltaTime);
            sendRDBTrigger(triggerSocket, 0, 0, mCheckForImage, deltaTime); // Extend the simulation by 10 ms
        }

    } else {
        deltaTime = 0.03;
        mCheckForImage = getGroundTruthImages;
        if ( withTrigger) {
            fprintf(stderr, "sendRDBTrigger: sending trigger, deltaT = %.4lf, requestImage = true \n",
                    deltaTime);
            sendRDBTrigger(triggerSocket, 0, 0, mCheckForImage, deltaTime); // Extend the simulation by 100 ms
        }
    }

    while (mCheckForImage) {

        checkShm(shmPtr);

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
        checkShm(shmPtr);  //empty IG buffer of spurious images
        // only in the beginning.

    }


    readNetwork(m_moduleManagerSocket_Camera);  // this calls parseRDBMessage() in vires_common.cpp

    readNetwork(m_moduleManagerSocket_Perfect);  // this calls parseRDBMessage() in vires_common.cpp

    readNetwork(m_moduleManagerSocket_PerfectInertial);  // this calls parseRDBMessage() in vires_common.cpp


}



void ViresObjects::parseEntry(RDB_TRIGGER_t *data, const double &simTime, const unsigned int &
simFrame, const unsigned short &pkgId, const unsigned short &flags, const unsigned int &elemId,
                                          const unsigned int &totalElem) {
    fprintf(stderr, "RDBTrigger answer = %.3f, simFrame = %d\n", simTime, simFrame);
}

void ViresObjects::parseStartOfFrame(const double &simTime, const unsigned int &simFrame) {
    //fprintf(stderr, "I am in GroundTruthFlow %d\n,", RDB_PKG_ID_START_OF_FRAME);
    //mHaveFirstFrame = true;
    //fprintf(stderr, "RDBHandler::parseStartOfFrame: simTime = %.3f, simFrame = %d\n", simTime, simFrame);
}

void ViresObjects::parseEndOfFrame(const double &simTime, const unsigned int &simFrame) {

    if (simFrame == MAX_DUMPS) {
        m_dumpInitialFrames = false;
    }
    mHaveFirstFrame = true;

    //mLastNetworkFrame = simFrame;
    fprintf(stderr, "RDBHandler::parseEndOfFrame: simTime = %.3f, simFrame = %d\n", simTime, simFrame);

    if (simFrame > MAX_ITERATION_GT_SCENE_GENERATION_DATASET + MAX_DUMPS) {
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
    RDB_COORD_t pos;                         /**< position and orientation of sensor's reference point  ( this is the sensor position with respect to the carrier )@unit m,m,m,rad,rad,rad                      */
    RDB_COORD_t originCoordSys;              /**< position and orientation of sensor's coord origin     ( this is the carrier with respect to inertial ) @unit m,m,m,rad,rad,rad                      */
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

/** ------ configuration of an object (sent at start of sim and when triggered via SCP) ------ */
typedef struct
{
    uint32_t id;                                    /**< unique object ID                                              @unit _                                  @version 0x0100 */
    uint8_t  category;                              /**< object category                                               @unit @link RDB_OBJECT_CATEGORY @endlink @version 0x0100 */
    uint8_t  type;                                  /**< object type                                                   @unit @link RDB_OBJECT_TYPE     @endlink @version 0x0100 */
    int16_t  modelId;                               /**< visual model ID                                               @unit _                                  @version 0x0100 */
    char     name[RDB_SIZE_OBJECT_NAME];            /**< symbolic name                                                 @unit _                                  @version 0x0100 */
    char     modelName[RDB_SIZE_OBJECT_NAME];       /**< model name associated to an object                            @unit _                                  @version 0x0100 */
    char     fileName[RDB_SIZE_FILENAME];           /**< filename associated to an object                              @unit _                                  @version 0x0100 */
    uint16_t flags;                                 /**< object configuration flags                                    @unit @link RDB_OBJECT_CFG_FLAG @endlink @version 0x0100 */
    uint16_t spare0;                                /**< reserved for future use                                       @unit _                                  @version 0x0100 */
    uint32_t spare1;                                /**< reserved for future use                                       @unit _                                  @version 0x0100 */
} RDB_OBJECT_CFG_DUMMY_t;

void ViresObjects::parseEntry(RDB_OBJECT_CFG_t *data, const double &simTime, const unsigned int &
simFrame, const
                                          unsigned short &pkgId, const unsigned short &flags,
                                          const unsigned int &elemId, const unsigned int &totalElem) {

    RDB_OBJECT_CFG_t *object = reinterpret_cast<RDB_OBJECT_CFG_t *>(data); /// raw image data
    std::cout << object->type;


}

void ViresObjects::parseEntry(RDB_SENSOR_STATE_t *data, const double &simTime, const unsigned int &simFrame,
                                          const unsigned short &pkgId, const unsigned short &flags,
                                          const unsigned int &elemId, const unsigned int &totalElem) {



    if (data->type == RDB_SENSOR_TYPE_VIDEO || data->type == RDB_SENSOR_TYPE_RADAR) {

        if (!m_dumpInitialFrames && (simFrame > (MAX_DUMPS+1)) && ((simFrame - MAX_DUMPS - 1) < MAX_ITERATION_GT_SCENE_GENERATION_DATASET ) ) {

            fprintf(stderr, "saving sensor truth for sensor_state simFrame = %d, simTime %f %s\n", simFrame, simTime,
                    data->name);

            ushort frame_number = (ushort) ((simFrame - MAX_DUMPS - 2) / IMAGE_SKIP_FACTOR_DYNAMIC);
            const char marker[] = "$$";
            std::cout << sizeof(marker) << " " << sizeof(frame_number) << " " << sizeof(RDB_SENSOR_STATE_t) << std::endl;
            fstream_output_sensor_state.write(marker, sizeof(marker-1));
            fstream_output_sensor_state.write((char *)&frame_number, sizeof(frame_number));
            fstream_output_sensor_state.write((char *)data, sizeof(RDB_SENSOR_STATE_t));

        }
    }
}

void ViresObjects::parseEntry(RDB_OBJECT_STATE_t *data, const double &simTime, const unsigned int &
simFrame, const
                                          unsigned
                                          short &pkgId, const unsigned short &flags, const unsigned int &elemId,
                                          const unsigned int &totalElem) {


    if (data->base.type == RDB_OBJECT_TYPE_PLAYER_PEDESTRIAN || data->base.type == RDB_OBJECT_TYPE_PLAYER_CAR) {

        if (!m_dumpInitialFrames && (simFrame > (MAX_DUMPS+1)) && ((simFrame - MAX_DUMPS - 1) < MAX_ITERATION_GT_SCENE_GENERATION_DATASET ) ) {

            /*
            fprintf(stderr, "%s: %d %.3lf %.3lf %.3lf %.3lf \n",
                    data->base.name, simFrame, data->base.pos.x, data->base.pos.y, data->base.geo.dimX, data->base
                            .geo.dimY);
*/
            fprintf(stderr, "saving ground truth for object_state simFrame = %d, simTime %f %s\n", simFrame, simTime,
                    data->base.name);

            ushort frame_number = (ushort) ((simFrame - MAX_DUMPS - 2) / IMAGE_SKIP_FACTOR_DYNAMIC);
            const char marker[] = "$$";
            std::cout << sizeof(marker) << " " << sizeof(frame_number) << " " << sizeof(RDB_OBJECT_STATE_t) << std::endl;
            fstream_output_object_state.write(marker, sizeof(marker-1));
            fstream_output_object_state.write((char *)&frame_number, sizeof(frame_number));
            fstream_output_object_state.write((char *)data, sizeof(RDB_OBJECT_STATE_t));

        } else {
            //std::cout << data->base.type << std::endl;
        }
    }
}

void ViresObjects::parseEntry(RDB_SENSOR_OBJECT_t *data, const double &simTime, const unsigned int &
simFrame, const unsigned short &pkgId, const unsigned short &flags, const unsigned int &elemId,
                                          const unsigned int &totalElem) {

    cv::Point3f offset, position_inertial, position_usk, position_pixel, dimension_realworld;
    cv::Point2f dimension_pixel;
    cv::Point2f speed_inertial, speed_usk;
    cv::Point3f orientation_usk, orientation_inertial;
    float dist_cam_to_obj;
    float total_distance_travelled;


    if (data->type == RDB_OBJECT_TYPE_PLAYER_PEDESTRIAN || data->type == RDB_OBJECT_TYPE_PLAYER_CAR) {
        if (!m_dumpInitialFrames && (simFrame > (MAX_DUMPS+1)) && ((simFrame - MAX_DUMPS - 1) < MAX_ITERATION_GT_SCENE_GENERATION_DATASET ) ) {

            //fprintf(stderr, "%s: %d %.3lf %.3lf %.3lf %.3lf \n",
            //        data->name, simFrame, data->pos.x, data->pos.y, data->geo.dimX, data->
            //                .geo.dimY);

            fprintf(stderr, "saving ground truth for sensor_object simFrame = %d, simTime %f %d\n", simFrame, simTime,
            data->id); //m_mapObjectIdToObjectName[data->id]);

            ushort frame_number = (ushort) ((simFrame - MAX_DUMPS - 2) / IMAGE_SKIP_FACTOR_DYNAMIC);
            const char marker[] = "$$";
            std::cout << sizeof(marker) << " " << sizeof(frame_number) << " " << sizeof(RDB_SENSOR_OBJECT_t) << std::endl;
            fstream_output_sensor_object.write(marker, sizeof(marker-1));
            fstream_output_sensor_object.write((char *)&frame_number, sizeof(frame_number));
            fstream_output_sensor_object.write((char *)data, sizeof(RDB_SENSOR_OBJECT_t));


        } else {
            //std::cout << data->base.type << std::endl;
        }
    }
}

void ViresObjects::parseEntry(RDB_IMAGE_t *data, const double &simTime, const unsigned int &simFrame,
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
                std::string input_image_file_with_path = m_generatepath.string() + "_" + std::to_string(m_sensorGroupCount) + "/" + file_name_image; //+ "/" +  file_name_image;
                if ( simFrame > (MAX_DUMPS) ) {
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
void ViresObjects::parseEntry(RDB_DRIVER_CTRL_t *data, const double &simTime, const unsigned int &simFrame,
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
    //TODO sendOwnObjectState(sOwnObjectState, m_triggerSocket, simTime, simFrame);

    // remember last simulation time
    sLastSimTime = simTime;
}

