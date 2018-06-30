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

        m_object_gt_all.at(current_frame_index).m_object_dimension_camera_px.width_px = 30;
        m_object_gt_all.at(current_frame_index).m_object_dimension_camera_px.height_px = 70;

        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.location_x_px = (l_pixel_position.x);
        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.location_y_px = (l_pixel_position.y);


        m_object_gt_all.at(current_frame_index).m_object_location_inertial_m.location_x_m = 1;
        m_object_gt_all.at(current_frame_index).visMask = 7;

        m_object_gt_all.at(current_frame_index).frame_no = current_frame_index;

        current_frame_index++;

    }
}

void Circle::process(cv::Size frame_size) {

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

        m_object_gt_all.at(current_frame_index).m_object_dimension_camera_px.width_px = 30;
        m_object_gt_all.at(current_frame_index).m_object_dimension_camera_px.height_px = 70;

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

        m_object_gt_all.at(current_frame_index).m_object_dimension_camera_px.width_px = 30;
        m_object_gt_all.at(current_frame_index).m_object_dimension_camera_px.height_px = 70;

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



