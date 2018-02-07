//
// Created by veikas on 25.01.18.
//

#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>
#include <iostream>


#include "kitti/log_colormap.h"
#include "ObjectTrajectory.h"
#include "datasets.h"



void Achterbahn::process(cv::Size frame_size) {
    std::vector<ushort> theta;
    for ( ushort frame_count = 0; frame_count < MAX_ITERATION_THETA; frame_count++) {
        theta.push_back(frame_count);
    }
    // Prepare points
    cv::Point2f l_pixel_position;
    for ( int i = 0; i< MAX_ITERATION_THETA; i++) {

        l_pixel_position.x = static_cast<float>((frame_size.width/2) + (500 * cos(theta[i] *CV_PI / 180.0) /
                                                                            (1.0 + std::pow(sin(theta[i] * CV_PI / 180.0), 2))));

        l_pixel_position.y = static_cast<float>((frame_size.height/2) + (55 * (cos(theta[i] * CV_PI / 180.0) *
                                                                                   sin(theta[i] * CV_PI / 180.0)) /
                (0.2 +std::pow(sin(theta[i] * CV_PI / 180.0),2))));

        m_trajectory.push_back(l_pixel_position);
    }
}

void Circle::process(cv::Size frame_size) {

    std::vector<ushort> theta;
    for ( ushort frame_count = 0; frame_count < MAX_ITERATION_THETA; frame_count++) {
        theta.push_back(frame_count);
    }
    // Prepare points
    cv::Point2f l_pixel_position;
    for ( int i = 0; i< MAX_ITERATION_THETA; i++) {

        l_pixel_position.x = static_cast<float>( frame_size.width/2 + 100 * cos(theta[i]));

        l_pixel_position.y = static_cast<float>( frame_size.height/2 + 100 * sin(theta[i]));

        m_trajectory.push_back(l_pixel_position);
    }
}

void Ramp::process(cv::Size frame_size) {

    std::vector<ushort> theta;
    for ( ushort frame_count = 0; frame_count < MAX_ITERATION_THETA; frame_count++) {
        theta.push_back(frame_count);
    }
    // Prepare points
    cv::Point2f l_pixel_position;
    for ( int i = 0; i< MAX_ITERATION_THETA; i++) {

        //l_pixel_position.x = static_cast<float>((frame_size.width/2) + 10 * cos(theta[i]));

        //l_pixel_position.y = static_cast<float>((frame_size.height/2) + 10 * sin(theta[i]));

        l_pixel_position.x = static_cast<float>(0 + (theta[i]));

        l_pixel_position.y = static_cast<float>(0 + (theta[i]));

        m_trajectory.push_back(l_pixel_position);
    }
}

void NegativeRamp::process(cv::Size frame_size) {

    std::vector<ushort> theta;
    for ( ushort frame_count = 0; frame_count < MAX_ITERATION_THETA; frame_count++) {
        theta.push_back(frame_count);
    }
    // Prepare points
    cv::Point2f l_pixel_position;
    for ( int i = 0; i< MAX_ITERATION_THETA; i++) {

        //l_pixel_position.x = static_cast<float>((frame_size.width/2) + 10 * cos(theta[i]));

        //l_pixel_position.y = static_cast<float>((frame_size.height/2) + 10 * sin(theta[i]));

        l_pixel_position.x = static_cast<float>(120 - (theta[i]));

        l_pixel_position.y = static_cast<float>(0  + (theta[i]));

        m_trajectory.push_back(l_pixel_position);
    }
}

void NoTrajectory::process(cv::Size frame_size) {

    for ( ushort frame_count = 0; frame_count < MAX_ITERATION_THETA; frame_count++) {
        m_trajectory.push_back(cv::Point2f(0, 0));
    }

};


