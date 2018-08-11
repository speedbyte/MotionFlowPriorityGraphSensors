//
// Created by veikas on 21.02.18.
//

#include "ObjectMetaData.h"
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>
#include <iostream>


#include "kitti/log_colormap.h"
#include "datasets.h"


void Achterbahn::process(cv::Size frame_size) {
    std::vector<ushort> theta;
    for ( ushort current_frame_index = 0; current_frame_index < MAX_ITERATION_THETA; current_frame_index+=1) {
        theta.push_back(current_frame_index);
    }
    // Prepare points
    cv::Point2f l_pixel_position;
    ushort current_frame_index = 0;
    for ( ushort i = m_objectMetaData_startPoint; i< MAX_ITERATION_GT_SCENE_GENERATION_DATASET+m_objectMetaData_startPoint; i++) {


        l_pixel_position.x = static_cast<float>((frame_size.width/2) + (100 * cos(theta[i] *CV_PI / 180.0) /
                                                                        (1.0 + std::pow(sin(theta[i] * CV_PI / 180.0), 2))));

        l_pixel_position.y = static_cast<float>((frame_size.height/2) + (55 * (cos(theta[i] * CV_PI / 180.0) *
                                                                               sin(theta[i] * CV_PI / 180.0)) /
                                                                         (0.2 +std::pow(sin(theta[i] * CV_PI / 180.0),2))));

        m_object_gt_all.at(current_frame_index).occluded = 0;

        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.location_x_px = (l_pixel_position.x);
        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.location_y_px = (l_pixel_position.y);


        m_object_gt_all.at(current_frame_index).m_object_location_inertial_m.location_x_m = 1;
        m_object_gt_all.at(current_frame_index).visMask = 7;

        m_object_gt_all.at(current_frame_index).frame_no = current_frame_index;

        current_frame_index++;

    }
}

void CircleTrajectory::process(cv::Size frame_size) {

    std::vector<ushort> theta;
    for ( ushort current_frame_index = 0; current_frame_index < MAX_ITERATION_THETA; current_frame_index++) {
        theta.push_back(current_frame_index);
    }
    // Prepare points
    cv::Point2f l_pixel_position;
    for ( int i = 0; i< MAX_ITERATION_THETA; i++) {

        l_pixel_position.x = static_cast<float>( frame_size.width/2 + 100 * cos(theta[i]));

        l_pixel_position.y = static_cast<float>( frame_size.height/2 + 100 * sin(theta[i]));

        m_object_gt_all.at(i).m_object_location_camera_px.location_x_px  = (l_pixel_position.x);
        m_object_gt_all.at(i).m_object_location_camera_px.location_y_px  = (l_pixel_position.y);
        m_object_gt_all.at(i).occluded;
    }
}

void Ramp::process(cv::Size frame_size) {

    std::vector<ushort> theta;
    for ( ushort current_frame_index = 0; current_frame_index < MAX_ITERATION_THETA; current_frame_index++) {
        theta.push_back(current_frame_index);
    }
    // Prepare points
    cv::Point2f l_pixel_position;
    ushort current_frame_index = 0;
    for ( ushort i = m_objectMetaData_startPoint; i< MAX_ITERATION_GT_SCENE_GENERATION_DATASET+m_objectMetaData_startPoint; i++) {

        //l_pixel_position.x = static_cast<float>((frame_size.width/2) + 10 * cos(theta[i]));

        //l_pixel_position.y = static_cast<float>((frame_size.height/2) + 10 * sin(theta[i]));

        l_pixel_position.x = static_cast<float>(0 + (theta[i]));

        l_pixel_position.y = static_cast<float>(0 + (theta[i]));

        m_object_gt_all.at(current_frame_index).occluded = 0;

        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.location_x_px = (l_pixel_position.x);
        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.location_y_px = (l_pixel_position.y);


        m_object_gt_all.at(current_frame_index).m_object_location_inertial_m.location_x_m = 1;
        m_object_gt_all.at(current_frame_index).visMask = 7;

        m_object_gt_all.at(current_frame_index).frame_no = current_frame_index;

        current_frame_index++;

    }
}

void NegativeRamp::process(cv::Size frame_size) {

    std::vector<ushort> theta;
    for ( ushort current_frame_index = 0; current_frame_index < MAX_ITERATION_THETA; current_frame_index++) {
        theta.push_back(current_frame_index);
    }
    // Prepare points
    cv::Point2f l_pixel_position;
    ushort current_frame_index = 0;
    for ( ushort i = m_objectMetaData_startPoint; i< MAX_ITERATION_GT_SCENE_GENERATION_DATASET+m_objectMetaData_startPoint; i++) {

        //l_pixel_position.x = static_cast<float>((frame_size.width/2) + 10 * cos(theta[i]));

        //l_pixel_position.y = static_cast<float>((frame_size.height/2) + 10 * sin(theta[i]));

        l_pixel_position.x = static_cast<float>(frame_size.width - (theta[i]));

        l_pixel_position.y = static_cast<float>(0  + (theta[i]));

        m_object_gt_all.at(current_frame_index).occluded = 0;

        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.location_x_px = (l_pixel_position.x);
        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.location_y_px = (l_pixel_position.y);

        m_object_gt_all.at(current_frame_index).m_object_location_inertial_m.location_x_m = 1;
        m_object_gt_all.at(current_frame_index).visMask = 7;

        m_object_gt_all.at(current_frame_index).frame_no = current_frame_index;

        current_frame_index++;
    }
}

void NoPosition::process(cv::Size frame_size) {

};

void ObjectMetaData::setCppData() {

    ushort current_frame_index = 0;
    for ( ushort i = m_objectMetaData_startPoint; i< MAX_ITERATION_GT_SCENE_GENERATION_DATASET+m_objectMetaData_startPoint; i++) {

        m_object_gt_all.at(current_frame_index).m_object_dimension_camera_px.width_px = m_objectMetaData_shape.getImage().cols;
        m_object_gt_all.at(current_frame_index).m_object_dimension_camera_px.height_px = m_objectMetaData_shape.getImage().rows;
        m_object_gt_all.at(current_frame_index).m_object_distances.sensor_to_obj_usk = m_objectMetaData_shape.getObjectDepth();


        m_object_gt_all.at(current_frame_index).m_region_of_interest_px.x = m_object_gt_all.at(
                current_frame_index).m_object_location_camera_px.location_x_px;
        m_object_gt_all.at(current_frame_index).m_region_of_interest_px.y = m_object_gt_all.at(
                current_frame_index).m_object_location_camera_px.location_y_px;

        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.cog_px.x = (
                m_object_gt_all.at(current_frame_index).m_region_of_interest_px.x +
                m_object_gt_all.at(current_frame_index).m_object_dimension_camera_px.width_px / 2);
        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.cog_px.y = (
                m_object_gt_all.at(current_frame_index).m_region_of_interest_px.y +
                m_object_gt_all.at(current_frame_index).m_object_dimension_camera_px.height_px / 2);

        m_object_gt_all.at(current_frame_index).m_region_of_interest_px.width_px = m_object_gt_all.at(
                current_frame_index).m_object_dimension_camera_px.width_px;
        m_object_gt_all.at(current_frame_index).m_region_of_interest_px.height_px = m_object_gt_all.at(
                current_frame_index).m_object_dimension_camera_px.height_px;
        current_frame_index++;
    }
}

void Rectangle::process() {

    //m_data_image.create(m_objectHeight, m_objectWidth, CV_8UC3);
    //m_data_depth.create(m_objectHeight, m_objectWidth, CV_8UC1);

}

void Canvas::process() {

    //m_data_image.create(m_objectHeight, m_objectWidth, CV_8UC3);

}

void Circle::construct(ushort radius, std::unique_ptr<Noise> &noise, ushort depth)  {
    m_data_image.create(radius, radius, CV_8UC3);
    m_data_image.setTo(cv::Scalar(255,255,255));
    m_data_depth.create(radius, radius, CV_8UC1);
    cv::circle(m_data_image, cv::Point(m_objectRadius, m_objectRadius), (m_objectRadius-5), cv::Scalar(255,0,0), CV_FILLED);
    applyNoise(noise);
    applyDepth(depth);

}

void Circle::process() {

    //m_data_image.create(m_objectHeight, m_objectWidth, CV_8UC3);
    //m_data_depth.create(m_objectHeight, m_objectWidth, CV_8UC1);

}




