//
// Created by veikas on 26.06.18.
//



#include <opencv2/core/persistence.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <iostream>
#include "BasicObjects.h"
#include "Dataset.h"
#include "datasets.h"
#include "ObjectMetaData.h"
#include "SensorMetaData.h"
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

        for (ushort obj_index = 0; obj_index < m_ptr_customObjectMetaDataList.size(); obj_index++) {


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

