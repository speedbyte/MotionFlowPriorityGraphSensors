//
// Created by veikas on 28.01.18.
//

#include "ObjectFlow.h"

#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>


#include "kitti/log_colormap.h"
#include "ObjectTrajectory.h"
#include "datasets.h"
#include "PlotFlow.h"


void ObjectFlow::storeData(const std::vector<cv::Point2f> &prev_pts, std::vector<cv::Point2f> &next_pts,
                                 const std::vector<uchar> status) {

    unsigned count = 0;

    for (unsigned i = 0; i < next_pts.size(); i++) {

        int minDist = 1;

        cv::Point2i gt_next_pts, gt_displacement;
        cv::Point2f algorithmMovement ((next_pts[i].x - prev_pts[i].x), (next_pts[i].y - prev_pts[i]
                .y));

        // Check if the status vector is good
        if (!status[i])
            continue;


        printf("flow_frame.at<cv::Point2f>(%f, %f).x =  %f\n", next_pts[i].x, next_pts[i].y,
               algorithmMovement.x);
        printf("flow_frame.at<cv::Point2f>(%f, %f).y =  %f\n", next_pts[i].x, next_pts[i].y,
               algorithmMovement.y);

        gt_displacement.x = cvRound( algorithmMovement.x + 0.5);
        gt_displacement.y = cvRound( algorithmMovement.y + 0.5);

        /* If the new point is within 'minDist' distance from an existing point, it will not be tracked */
        // auto dist = cv::norm(prev_pts[i] - next_pts[i]);
        double dist;
        dist = pow(gt_displacement.x,2)+pow(gt_displacement.y,2);
        //calculating distance by euclidean formula
        dist = sqrt(dist);

        if ( dist <= minDist ) {
            printf("minimum distance for %i is %f\n", i, dist);
            continue;
        }

        if ( gt_displacement.x == 0 && gt_displacement.y == 0) {
            continue;
        }

        next_pts[count++] = next_pts[i];

        // gt_next_pts is the new pixel position !
        gt_next_pts.x = std::abs(cvRound(next_pts[i].x));
        gt_next_pts.y = std::abs(cvRound(next_pts[i].y));

        printf("(iteration %u, coordinates x y (%i,%i) ->  Vx, Vy (%d,%d) \n", i,
               gt_next_pts.x, gt_next_pts.y, gt_displacement.x, gt_displacement.y);
        // Lines to indicate the motion vectors
        m_obj_flow_vector_baseframe.push_back(std::make_pair(gt_next_pts, gt_displacement));
    }
    next_pts.resize(count);
}

void ObjectFlow::store_in_png(cv::FileStorage &fs, ushort &frame_count, cv::Size frame_size,
                              std::string temp_result_flow_path) {
    //Create png Matrix with 3 channels: x displacement. y displacment and Validation bit
    //kitti uses col, row specification
    FlowImageExtended F_gt_write(Dataset::getFrameSize().width, Dataset::getFrameSize().height);
    std::vector<std::pair<cv::Point2i, cv::Point2i> >::iterator it ;

    fs << "frame_count" << frame_count;

    for ( it = m_obj_flow_vector_baseframe.begin(); it != m_obj_flow_vector_baseframe.end(); it++ )
    {
        //F_result_write.setFlowU((*it).first.x,(*it).first.y,(*it).second.x);
        //F_result_write.setFlowV((*it).first.x,(*it).first.y,(*it).second.y);
        //F_result_write.setValid((*it).first.x,(*it).first.y,(bool)1.0f);
        store_in_yaml(fs, (*it).first, (*it).second ); // coordinate - > movement y(row),x(col) ; x,y
    }
    //F_result_write.write(temp_result_flow_path);
}

void ObjectFlow::store_in_yaml(cv::FileStorage &fs, const cv::Point2i &l_pixelposition, const cv::Point2i
&l_pixelmovement )  {

    fs << "gt flow png file read" << "[";
    fs << "{:" << "row" <<  l_pixelposition.y << "col" << l_pixelposition.x << "displacement" << "[:";
    fs << l_pixelmovement.x;
    fs << l_pixelmovement.y;
    fs << 1;
    fs << "]" << "}";
    fs << "]";
}



void ObjectFlow::generate_baseframe_flow_vector(const ushort &start_point, const
std::vector<cv::Point2i> &trajectory_points) {

    //Initialization

    ushort current_index = start_point;

    std::cout << "start point "<< current_index << std::endl;

    for (ushort frame_count=0; frame_count < MAX_ITERATION_GT; frame_count++) {
        // The first frame is the reference frame, hence it is skipped

        if ( frame_count > 0 ) {

            cv::Point2i gt_next_pts= {0,0}, gt_displacement = {0,0};

            //If we are at the end of the path vector, we need to reset our iterators
            if (current_index >= trajectory_points.size()) {
                current_index = 0;
                gt_displacement.x = trajectory_points.at(current_index).x - trajectory_points.at
                        (trajectory_points.size() - 1).x;
                gt_displacement.y = trajectory_points.at(current_index).y - trajectory_points.at
                        (trajectory_points.size() - 1).y;
                gt_next_pts = trajectory_points.at(current_index);
            } else {
                gt_displacement.x = trajectory_points.at(current_index).x - trajectory_points.at
                        (current_index - (ushort) 1).x;
                gt_displacement.y = trajectory_points.at(current_index).y - trajectory_points.at
                        (current_index - (ushort) 1).y;
                gt_next_pts = trajectory_points.at(current_index);
            }

            printf("%u, %u , %u, %u, %d, %d\n", frame_count, current_index, gt_next_pts.x,
                   gt_next_pts.y,
                   gt_displacement.x, gt_displacement.y);

            // make m_flowvector_with_coordinate_gt with smallest resolution.
            m_obj_flow_vector_baseframe.push_back(std::make_pair(gt_next_pts, gt_displacement));
        }
        else {
            m_obj_flow_vector_baseframe.push_back(std::make_pair(cv::Point2i(0,0), cv::Point2i(0,0)));
        }
        current_index++;
    }
}

void ObjectFlow::generate_multiframe_flow_vector(const int &max_skips ) {

    int temp_flow_x(0);
    int temp_flow_y(0);

    for ( int frame_skip = 1; frame_skip < max_skips ; frame_skip++ ) {

        std::vector<std::pair<cv::Point2i, cv::Point2i> > multiframe_flowvector;

        std::cout << "creating flow files for frame_skip " << frame_skip << std::endl;

        for (ushort frame_count = 1; frame_count < MAX_ITERATION_GT; frame_count++) {

            // The first frame is the reference frame.
            // frame skip 1 means no skips
            // The below code has to go through consecutive frames
            if ((frame_count % frame_skip != 0)) {
                temp_flow_x += m_obj_flow_vector_baseframe.at(frame_count).second.x;
                temp_flow_y += m_obj_flow_vector_baseframe.at(frame_count).second.y;
            }
            else {
                temp_flow_x += m_obj_flow_vector_baseframe.at(frame_count).second.x;
                temp_flow_y += m_obj_flow_vector_baseframe.at(frame_count).second.y;

                multiframe_flowvector.push_back
                        (std::make_pair(m_obj_flow_vector_baseframe.at
                        (frame_count).first, cv::Point2i(temp_flow_x, temp_flow_y)));
                temp_flow_x = 0, temp_flow_y = 0;
            }
        }
        m_obj_flow_vector_multiframe.push_back(multiframe_flowvector);
    }
}

