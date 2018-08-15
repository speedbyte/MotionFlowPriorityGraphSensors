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

void BasicObjects::calcBBFrom3DPosition(std::string suffix) {

    cv::FileStorage write_fs;
    char file_name_image[50], file_name_image_output[50];

    cv::Mat image_data_and_shape;

    cv::Mat tempGroundTruthImage(Dataset::m_frame_size, CV_8UC3);

    unsigned long FRAME_COUNT = Dataset::ITERATION_END_POINT;
    assert(FRAME_COUNT > 0);

    if ( suffix == "vires_") {
        //
        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

            tempGroundTruthImage = cv::Scalar::all(255);
            char sensor_index_folder_suffix[20];
            sprintf(sensor_index_folder_suffix, "%02d", m_sensorGroupCount);

            sprintf(file_name_image, "000%03d_10.png", current_frame_index);
            std::string input_image_file_with_path =
                    m_generatepath.string() + "_" + sensor_index_folder_suffix + "/" + file_name_image;

            sprintf(file_name_image_output, "000%03d_10_bb.png", current_frame_index);
            //output_image_file_with_path = m_generatepath.string() + "stencil/" + file_name_image_output;

            cv::Mat tempGroundTruthImageBase = cv::imread(input_image_file_with_path, CV_LOAD_IMAGE_ANYCOLOR);
            if (tempGroundTruthImageBase.data == NULL) {
                std::cerr << input_image_file_with_path << " not found" << std::endl;
                throw ("No image file found error");
            }
            tempGroundTruthImage = cv::Scalar::all(255);
            tempGroundTruthImage = tempGroundTruthImageBase.clone();

            for (ushort obj_index = 0; obj_index < m_ptr_customObjectMetaDataList.size(); obj_index++) {

                object_realworld_dim_m_str object_realworld_dim_m = m_ptr_customObjectMetaDataList.at(
                        obj_index)->getAll().at(
                        current_frame_index).m_object_dimension_realworld_m;

                object_location_inertial_m_str pos_object_inertial = m_ptr_customObjectMetaDataList.at(
                        obj_index)->getAll().at(
                        current_frame_index).m_object_location_inertial_m;

                object_location_m_str object_location_m = m_ptr_customObjectMetaDataList.at(
                        obj_index)->getAll().at(
                        current_frame_index).m_object_location_usk_m;

                object_rotation_inertial_rad_str orientation_object_inertial = m_ptr_customObjectMetaDataList.at(
                        obj_index)->getAll().at(
                        current_frame_index).m_object_rotation_inertial_rad;

                sensor_location_carrier_m_str pos_sensor_carrier_inertial = m_ptr_customSensorMetaDataList.at(
                        0)->getAll().at(current_frame_index).m_sensor_location_carrier_m;

                sensor_rotation_carrier_rad_str sensor_rotation_carrier_rad = m_ptr_customSensorMetaDataList.at(
                        0)->getAll().at(
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

                if (m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).visMask &&
                    pos_object_inertial.location_x_m != 0) {

                    std::vector<cv::Point3f> bounding_points_3d(9);
                    std::vector<cv::Point2f> bounding_points_2d(9);


                    //
                    // assuming vehicle co-ordinate system of the object.
                    // create a box placed on z = 0 ( street level ). Find 8 points of the box.
                    //
                    //

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

                    sensor_cam_info_str sensor_fov_rad = m_ptr_customSensorMetaDataList.at(0)->getAll().at(
                            current_frame_index).m_sensor_cam_info;


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

                        cv::Point2f camPoint = Utils::worldToCameraIntrinsc(final, sensor_fov_rad.fov_vertical_rad,
                                                                            FOCAL_X, FOCAL_Y);
                        // FOCAL_X, FOCAL_Y, principalx and principaly can be obtained from the camera info from VIRES
                        //cv::Point2f camPoint_openglfrustum = Utils::worldToCamera(final, fov_rad.vertical, FOCAL_X, FOCAL_Y);
                        bounding_points_2d.at(i) = cv::Point2f(camPoint.x, camPoint.y);

                    }

                    std::cout << "converting inertial to usk"
                              << cv::Point2f(bounding_points_3d.at(8).x, bounding_points_3d.at(8).y)
                              << std::endl;
                    std::cout << "original usk"
                              << cv::Point2f(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                      current_frame_index).m_object_location_usk_m.location_x_m,
                                             m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                                     current_frame_index).m_object_location_usk_m.location_y_m)
                              << std::endl;

                    auto dist_usk_from_inertial = cv::norm(
                            cv::Point2f(bounding_points_3d.at(8).x + sensor_offset_m.offset_x,
                                        bounding_points_3d.at(8).y + sensor_offset_m.offset_y));
                    auto dist_usk_original = cv::norm(
                            cv::Point2f(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                    current_frame_index).m_object_location_usk_m.location_x_m,
                                        m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                                current_frame_index).m_object_location_usk_m.location_y_m));
                    auto dist_usk_from_vires = m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                            current_frame_index).m_object_distances.sensor_to_obj_usk;


                    assert(std::abs(dist_usk_from_inertial - dist_usk_original) < 0.5);
                    assert(std::floor((float)dist_usk_original*1000) == std::floor(dist_usk_from_vires*1000));

                    std::cout << "distance is " << dist_usk_from_inertial << " for "
                              << m_ptr_customObjectMetaDataList.at(obj_index)->getObjectName()
                              << std::endl;

                    m_ptr_customObjectMetaDataList.at(obj_index)->setBoundingBoxPoints(current_frame_index,
                                                                                       bounding_points_2d);

                    cv::Point2f xx = Utils::worldToCameraIntrinsc(
                            cv::Point3f(object_location_m.location_y_m, object_location_m.location_z_m,
                                        object_location_m.location_x_m), sensor_fov_rad.fov_vertical_rad, 980, 980);

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
                                    current_frame_index).m_object_location_camera_px.cog_px.y), 2,
                               cv::Scalar(0, 255, 0), 4);
                    cv::Rect box_points = cv::boundingRect(box);
                    cv::rectangle(tempGroundTruthImage, box_points, cv::Scalar(0, 0, 255), 1, 8, 0);

                    //}
                } else {
                    std::cout << m_ptr_customObjectMetaDataList.at(obj_index)->getObjectName() << " is occlued"
                              << std::endl;
                }
            }
        }
    }
    else if ( suffix == "cpp_") {
        for (ushort obj_index = 0; obj_index < m_ptr_customObjectMetaDataList.size(); obj_index++) {
            m_ptr_customObjectMetaDataList.at(obj_index)->setCppData();
        }
    }


    //cv::namedWindow("BB", CV_WINDOW_AUTOSIZE);
    //cv::imshow("BB", tempGroundTruthImage);
    //cv::waitKey(0);
    //cv::destroyAllWindows();
    //cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
    //---------------------------------------------------------------------------------

}

void BasicObjects::generateFrameDifferenceImage() {
    // Frame Differencing

    unsigned long FRAME_COUNT = 0;
    FRAME_COUNT = Dataset::ITERATION_END_POINT;
    assert(FRAME_COUNT > 0);

    for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {
        char sensor_index_folder_suffix[50];
        sprintf(sensor_index_folder_suffix, "%02d", m_sensorGroupCount);

        char file_name_input_image[50];
        sprintf(file_name_input_image, "000%03d_10.png", current_frame_index);
        std::string input_image_path =
                m_generatepath.string() + "_" + sensor_index_folder_suffix + "/" + file_name_input_image;

        cv::Mat image_02_frame = cv::imread(input_image_path, CV_LOAD_IMAGE_COLOR);
        if (image_02_frame.data == NULL) {
            std::cerr << input_image_path << " not found" << std::endl;
            throw ("No image file found error");
        }
        std::string input_image_path_background =
                Dataset::m_dataset_gtpath.string() + "base_frame_" + sensor_index_folder_suffix + "/" +
                file_name_input_image;
        cv::Mat backgroundImage = cv::imread(input_image_path_background, CV_LOAD_IMAGE_COLOR);
        if (backgroundImage.data == NULL) {
            std::cerr << input_image_path_background << " not found" << std::endl;
            throw ("No image file found error");
        }
        cv::Mat frameDifference;
        cv::cvtColor(image_02_frame, image_02_frame, cv::COLOR_BGR2GRAY);
        cv::cvtColor(backgroundImage, backgroundImage, cv::COLOR_BGR2GRAY);
        cv::compare(backgroundImage, image_02_frame, frameDifference, cv::CMP_EQ);
        // When the comparison result is true, the corresponding element of output
        // array is set to 255. The comparison operations can be replaced with the
        // equivalent matrix expressions:

        //cv::imshow("try", frameDifference);
        //cv::waitKey(0);
        std::string frame_difference_image_path =
                m_framedifferencepath.string() + "_" + sensor_index_folder_suffix + "/" + file_name_input_image;
        cv::imwrite(frame_difference_image_path, frameDifference);
    }
}

void BasicObjects::writePositionInYaml(std::string suffix) {

    char sensor_index_folder_suffix[50];
    sprintf(sensor_index_folder_suffix, "%02d", m_sensorGroupCount);

    boost::filesystem::remove("../position_" + suffix + sensor_index_folder_suffix + ".yml");

    cv::FileStorage write_fs;
    write_fs.open("../position_" + suffix + sensor_index_folder_suffix + ".yml", cv::FileStorage::WRITE);
    unsigned long FRAME_COUNT = 0;
    FRAME_COUNT = Dataset::ITERATION_END_POINT;
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

                    << "dist_cam_to_obj_px" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_distances.sensor_to_obj_px
                    << "dist_cam_to_obj_usk" << m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_distances.sensor_to_obj_usk
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
                    << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(current_frame_index).m_sensor_cam_info.fov_horizontal_rad
                    << "fov_vertical"
                    << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(current_frame_index).m_sensor_cam_info.fov_vertical_rad
                    << "clip_horizontal"
                    << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(current_frame_index).m_sensor_cam_info.near_m
                    << "clip_vertical"
                    << m_ptr_customSensorMetaDataList.at(sen_index)->getAll().at(current_frame_index).m_sensor_cam_info.far_m

                    << "}";
        }

        write_fs << "]";
    }
    //write_fs << "]";
    write_fs.release();

}

void BasicObjects::readPositionFromFile(std::string suffix) {

    cv::Point2f speed_usk, speed_inertial;
    cv::Point2f dimension_pixel, sensor_fov, sensor_clip;
    cv::Point3f offset, position_inertial, position_usk, position_pixel, dimension_realworld;
    cv::Point3f position_inertial_pre, position_usk_pre, position_pixel_pre;
    float totalDistanceTravelled;
    float sensor_to_obj;

    cv::Point3f orientation_inertial, orientation_usk;


    cv::FileNode file_node;
    cv::FileNodeIterator file_node_iterator_begin, file_node_iterator_end, file_node_iterator;

    char sensor_index_folder_suffix[50];
    sprintf(sensor_index_folder_suffix, "%02d", m_sensorGroupCount);

    ushort  objectCount = 0;
    ushort  sensorCount = 0;
    cv::FileStorage fs;
    fs.open("../position_" + suffix + sensor_index_folder_suffix + ".yml", cv::FileStorage::READ);

    assert(fs.isOpened());

    //std::string temp_str = "_sensor_count_" + sensor_index;
    std::cout << "read yaml file for sensor_index " << sensor_index_folder_suffix << std::endl;
    //unsigned long FRAME_COUNT = m_list_gt_objects.at(0).get_object_shape_point_displacement().size();
    unsigned long FRAME_COUNT;
    FRAME_COUNT = Dataset::ITERATION_END_POINT;


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

                    if (m_mapObjectNameToObjectMetaData.count((*file_node_iterator)["name"].string()) == 0 ) {
                        m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(m_objectCount));
                        m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()] = m_ptr_customObjectMetaDataList.at(objectCount);
                        m_ptr_customObjectMetaDataList.at(objectCount)->setObjectName((*file_node_iterator)["name"].string());
                        std::unique_ptr<Noise> noNoise = std::make_unique<NoNoise>(NoNoise());
                        Rectangle rectangle((int)(*file_node_iterator)["dim_x_camera"], (int)(*file_node_iterator)["dim_y_camera"], noNoise, (float)(*file_node_iterator)["dist_cam_to_obj_usk"]); // width, height
                        m_ptr_customObjectMetaDataList.at(objectCount)->setObjectShape(rectangle);
                        m_ptr_customObjectMetaDataList.at(objectCount)->setStartPoint(Dataset::ITERATION_START_POINT);
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
                            current_frame_index, (int) (*file_node_iterator)["occ_px"], (float) (*file_node_iterator)["dist_cam_to_obj_px"]);

                    (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberOcclusionUsk(
                            current_frame_index, (int) (*file_node_iterator)["occ_usk"], (float) (*file_node_iterator)["dist_cam_to_obj_usk"]);

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

                    sensor_clip = cv::Point2f((double) (*file_node_iterator)["clip_near"],
                            (double) (*file_node_iterator)["clip_far"]);

                    (m_mapSensorNameToSensorMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberSensorState(
                            current_frame_index, position_inertial, orientation_inertial, orientation_usk, offset,
                            sensor_fov, sensor_clip);

                    (m_mapSensorNameToSensorMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberVisibility(
                            current_frame_index, (int) (*file_node_iterator)["visible"]);
                }
            }
        }
    }
    fs.release();
}



