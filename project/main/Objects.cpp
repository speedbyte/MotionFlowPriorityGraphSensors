//
// Created by veikas on 28.01.18.
//

#include <iostream>
#include "Objects.h"


unsigned Objects::objectCurrentCount = 0;



void Objects::generate_baseframe_flow_vector(const ushort &start_point, const
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
            m_obj_flow_vector_base_movement.push_back(std::make_pair(gt_next_pts, gt_displacement));
        }
        else {
            m_obj_flow_vector_base_movement.push_back(std::make_pair(cv::Point2i(0,0), cv::Point2i(0,0)));
        }
        current_index++;
    }
}

void Objects::generate_multiframe_flow_vector(const int &max_skips ) {

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
                temp_flow_x += m_obj_flow_vector_base_movement.at(frame_count).second.x;
                temp_flow_y += m_obj_flow_vector_base_movement.at(frame_count).second.y;
            }
            else {
                temp_flow_x += m_obj_flow_vector_base_movement.at(frame_count).second.x;
                temp_flow_y += m_obj_flow_vector_base_movement.at(frame_count).second.y;

                multiframe_flowvector.push_back
                        (std::make_pair(m_obj_flow_vector_base_movement.at
                                (frame_count).first, cv::Point2i(temp_flow_x, temp_flow_y)));
                temp_flow_x = 0, temp_flow_y = 0;
            }
        }
        m_obj_flow_vector_fast_movement.push_back(multiframe_flowvector);
    }
}

