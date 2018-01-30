//
// Created by veikas on 25.01.18.
//

#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>


#include "kitti/log_colormap.h"
#include <kitti/io_flow.h>
#include "ObjectTrajectory.h"
#include "datasets.h"



void Achterbahn::process(cv::Size frame_size) {
    std::vector<ushort> theta;
    for ( ushort frame_count = 0; frame_count < MAX_ITERATION_THETA; frame_count++) {
        theta.push_back(frame_count);
    }
    // Prepare points
    cv::Point2i l_pixel_position;
    for ( int i = 0; i< MAX_ITERATION_THETA; i++) {

        l_pixel_position.x = (static_cast<ushort>((frame_size.width/2) + (500 * cos(theta[i] *CV_PI / 180.0) /
                                                                            (1.0 + std::pow(sin(theta[i] * CV_PI / 180.0), 2)))));

        l_pixel_position.y = (static_cast<ushort>((frame_size.height/2) + (55 * (cos(theta[i] * CV_PI / 180.0) *
                                                                                   sin(theta[i] * CV_PI / 180.0)) / (0.2 +std::pow(sin(theta[i] * CV_PI / 180.0),2)))));

        m_trajectory.push_back(l_pixel_position);
    }
}


