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



ObjectTrajectory::ObjectTrajectory() {

};

void ObjectTrajectory::storeData(const std::vector<cv::Point2f> &prev_pts, std::vector<cv::Point2f> &next_pts,
                                 const std::vector<uchar> status) {

    unsigned count = 0;

    for (unsigned i = 0; i < next_pts.size(); i++) {

        int minDist = 1;

        cv::Point2i l_pixel_position, l_pixel_movement;
        cv::Point2f algorithmMovement ((next_pts[i].x - prev_pts[i].x), (next_pts[i].y - prev_pts[i]
                .y));

        // Check if the status vector is good
        if (!status[i])
            continue;


        printf("flow_frame.at<cv::Point2f>(%f, %f).x =  %f\n", next_pts[i].x, next_pts[i].y,
               algorithmMovement.x);
        printf("flow_frame.at<cv::Point2f>(%f, %f).y =  %f\n", next_pts[i].x, next_pts[i].y,
               algorithmMovement.y);

        l_pixel_movement.x = cvRound( algorithmMovement.x + 0.5);
        l_pixel_movement.y = cvRound( algorithmMovement.y + 0.5);

        /* If the new point is within 'minDist' distance from an existing point, it will not be tracked */
        // auto dist = cv::norm(prev_pts[i] - next_pts[i]);
        double dist;
        dist = pow(l_pixel_movement.x,2)+pow(l_pixel_movement.y,2);
        //calculating distance by euclidean formula
        dist = sqrt(dist);

        if ( dist <= minDist ) {
            printf("minimum distance for %i is %f\n", i, dist);
            continue;
        }

        if ( l_pixel_movement.x == 0 && l_pixel_movement.y == 0) {
            continue;
        }

        next_pts[count++] = next_pts[i];

        // l_pixel_position is the new pixel position !
        l_pixel_position.x = std::abs(cvRound(next_pts[i].x));
        l_pixel_position.y = std::abs(cvRound(next_pts[i].y));

        printf("(iteration %u, coordinates x y (%i,%i) ->  Vx, Vy (%d,%d) \n", i,
               l_pixel_position.x, l_pixel_position.y, l_pixel_movement.x, l_pixel_movement.y);
        // Lines to indicate the motion vectors
        m_flow_matrix_result.push_back(std::make_pair(l_pixel_position, l_pixel_movement));
    }
    next_pts.resize(count);
}

void ObjectTrajectory::store_in_png(cv::FileStorage &fs, ushort &frame_count, cv::Size frame_size, std::string
temp_result_flow_path) {
    //Create png Matrix with 3 channels: x displacement. y displacment and Validation bit
    //kitti uses col, row specification
    FlowImage F_result_write(frame_size.width, frame_size.height);
    std::vector<std::pair<cv::Point2i, cv::Point2i> >::iterator it ;

    fs << "frame_count" << frame_count;

    for ( it = m_flow_matrix_result.begin(); it != m_flow_matrix_result.end(); it++ )
    {
        F_result_write.setFlowU((*it).first.x,(*it).first.y,(*it).second.x);
        F_result_write.setFlowV((*it).first.x,(*it).first.y,(*it).second.y);
        F_result_write.setValid((*it).first.x,(*it).first.y,(bool)1.0f);
        store_in_yaml(fs, (*it).first, (*it).second ); // coordinate - > movement y(row),x(col) ; x,y
    }
    F_result_write.write(temp_result_flow_path);
}

void ObjectTrajectory::store_in_yaml(cv::FileStorage &fs, const cv::Point2i &l_pixelposition, const cv::Point2i
    &l_pixelmovement )  {

    fs << "gt flow png file read" << "[";
    fs << "{:" << "row" <<  l_pixelposition.y << "col" << l_pixelposition.x << "displacement" << "[:";
    fs << l_pixelmovement.x;
    fs << l_pixelmovement.y;
    fs << 1;
    fs << "]" << "}";
    fs << "]";
}

