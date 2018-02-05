//
// Created by veikas on 28.01.18.
//

#include <iostream>
#include "Objects.h"


unsigned Objects::objectCurrentCount = 0;



void Objects::generate_obj_base_pixel_point_pixel_displacement(const ushort &start_point, const
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
            m_obj_base_pixel_point_pixel_displacement.push_back(std::make_pair(gt_next_pts, gt_displacement));
        }
        else {
            m_obj_base_pixel_point_pixel_displacement.push_back(std::make_pair(cv::Point2i(0,0), cv::Point2i(0,0)));
        }
        current_index++;
    }
}

void Objects::generate_obj_extrapolated_pixel_point_pixel_displacement(const unsigned &max_skips) {

    int temp_flow_x(0);
    int temp_flow_y(0);

    for ( unsigned frame_skip = 1; frame_skip < max_skips ; frame_skip++ ) {

        std::vector<std::pair<cv::Point2i, cv::Point2i> > multiframe_flowvector;

        std::cout << "creating flow files for frame_skip " << frame_skip << std::endl;
        unsigned long FRAME_COUNT = m_obj_base_pixel_point_pixel_displacement.size();

        for (ushort frame_count = 1; frame_count < FRAME_COUNT; frame_count++) {

            // The first frame is the reference frame.
            // frame skip 1 means no skips
            // The below code has to go through consecutive frames
            if ((frame_count % frame_skip != 0)) {
                temp_flow_x += m_obj_base_pixel_point_pixel_displacement.at(frame_count).second.x;
                temp_flow_y += m_obj_base_pixel_point_pixel_displacement.at(frame_count).second.y;
            }
            else {
                temp_flow_x += m_obj_base_pixel_point_pixel_displacement.at(frame_count).second.x;
                temp_flow_y += m_obj_base_pixel_point_pixel_displacement.at(frame_count).second.y;

                multiframe_flowvector.push_back
                        (std::make_pair(m_obj_base_pixel_point_pixel_displacement.at
                                (frame_count).first, cv::Point2i(temp_flow_x, temp_flow_y)));
                temp_flow_x = 0, temp_flow_y = 0;
            }
        }
        m_obj_extrapolated_pixel_point_pixel_displacement.push_back(multiframe_flowvector);
    }
}


void Objects::generate_obj_extrapolated_shape_pixel_point_pixel_displacement(const unsigned &max_skips ) {

// object image_data_and_shape
    int width = m_image_data_and_shape.get().cols;
    int height = m_image_data_and_shape.get().rows;

    for (unsigned frame_skip = 1; frame_skip < max_skips; frame_skip++) {
        std::vector<std::vector<std::pair<cv::Point2i, cv::Point2i> > > outer_base_movement;
        unsigned long FRAME_COUNT = m_obj_extrapolated_pixel_point_pixel_displacement.at(frame_skip - 1).size();
        for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement
            cv::Point2i gt_next_pts = m_obj_extrapolated_pixel_point_pixel_displacement.at(frame_skip - 1).at(frame_count).first;
            cv::Point2f gt_displacement = m_obj_extrapolated_pixel_point_pixel_displacement.at(frame_skip - 1).at(frame_count).second;

            std::vector<std::pair<cv::Point2i, cv::Point2i> > base_movement;

            for (unsigned j = 0; j < width; j++) {
                for (unsigned k = 0; k < height; k++) {
                    base_movement.push_back(std::make_pair(cv::Point2i(gt_next_pts.x + j, gt_next_pts.y +
                                                                                          k), gt_displacement));
                }
            }
            outer_base_movement.push_back(base_movement);
        }
        m_obj_extrapolated_shape_pixel_point_pixel_displacement.push_back(outer_base_movement);
    }
}


void Objects::generate_obj_extrapolated_pixel_centroid_pixel_displacement_mean( const unsigned &max_skips) {


    for (unsigned frame_skip = 1; frame_skip < max_skips; frame_skip++) {
        std::vector<std::pair<cv::Point2i, cv::Point2i> > multiframe_flowvector;
        unsigned long FRAME_COUNT = m_obj_extrapolated_pixel_point_pixel_displacement.at(frame_skip - 1).size();
        for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement
            int prev_pts_x = 0;
            int prev_pts_y = 0;
            int next_pts_x = 0;
            int next_pts_y = 0;
            int displacement_vector_x = 0;
            int displacement_vector_y = 0;

            const unsigned CLUSTER_SIZE = (unsigned)m_obj_extrapolated_shape_pixel_point_pixel_displacement.at
                            (frame_skip - 1)
                    .at(frame_count).size();
            for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {
                cv::Point2i pts = m_obj_extrapolated_shape_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).first;
                cv::Point2f gt_displacement = m_obj_extrapolated_shape_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).second;
                next_pts_x += pts.x ;
                next_pts_y += pts.y ;
                displacement_vector_x += gt_displacement.x ;
                displacement_vector_y += gt_displacement.y ;
            }
            next_pts_x /= CLUSTER_SIZE;
            next_pts_y /= CLUSTER_SIZE;
            displacement_vector_x /= (int)CLUSTER_SIZE;
            displacement_vector_y /= (int)CLUSTER_SIZE;
            //prev_pts_x = next_pts_x - displacement_vector_x;
            //prev_pts_y = next_pts_y - displacement_vector_y;

            // I should return the vector instead of points and then normalize it.
            multiframe_flowvector.push_back(std::make_pair(cv::Point2i(int(std::round(next_pts_x)), int(
                    (std::round(next_pts_y)))), cv::Point2d
                    (int(std::round(displacement_vector_x)), int(std::round(displacement_vector_y)))));


        }

        m_obj_extrapolated_pixel_centroid_pixel_displacement_mean.push_back(multiframe_flowvector);
    }


    //TODO - clean up this section.
    for (unsigned frame_skip = 1; frame_skip < max_skips; frame_skip++) {
        std::vector<std::pair<cv::Point2f, cv::Point2i> > line_parameters;
        const unsigned long FRAME_COUNT = m_obj_extrapolated_pixel_centroid_pixel_displacement_mean.at(frame_skip - 1)
                .size();
        for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement
            cv::Point2i next_pts = m_obj_extrapolated_pixel_centroid_pixel_displacement_mean.at(frame_skip - 1)
                    .at(frame_count).first;
            cv::Point2i  displacement_vector = m_obj_extrapolated_pixel_centroid_pixel_displacement_mean.at
                            (frame_skip - 1).at(frame_count).second;

            cv::Vec<float, 4> line_in_cartesian_coordinates = {displacement_vector.x, -displacement_vector.y,
                                                               next_pts.x, -next_pts.y};

            float m, c;
            m = line_in_cartesian_coordinates[1] / line_in_cartesian_coordinates[0];
            c = line_in_cartesian_coordinates[3] - line_in_cartesian_coordinates[2] * m;

            float d = (float) sqrt((double) line_in_cartesian_coordinates[0] * line_in_cartesian_coordinates[0] +
                                   (double) line_in_cartesian_coordinates[1] * line_in_cartesian_coordinates[1]);
            line_in_cartesian_coordinates[0] /= d; // normalized vector in x
            line_in_cartesian_coordinates[1] /= d; // normalized vector in y

            cv::Point pt1, pt2;
            pt1.x = cvRound(line_in_cartesian_coordinates[2]); // 700
            pt1.y = cvRound(-line_in_cartesian_coordinates[3]); // again change to pixel coordinates

            if (std::isinf(m)) {
                if (line_in_cartesian_coordinates[1] >
                    0.0f) { //  if going up ( displacement.y in cartesian > 0 ) then, find x point on y = height
                    pt2.x = pt1.x; //
                    pt2.y = 0;
                    pt2.y = -pt2.y; // again change to pixel coordinates
                    //std::cout << temp_gt_flow_image_path << " " << pt1 <<  " " << m << " " << pt2<< std::endl;
                } else {
                    pt2.x = pt1.x; //
                    pt2.y = -Dataset::getFrameSize().height;
                    pt2.y = -pt2.y; // again change to pixel coordinates
                    //std::cout << temp_gt_flow_image_path << " " << pt1 <<  " " << m << " " << pt2<< std::endl;
                }
            } else if (m == 0) {
                if (std::signbit(m)) { //  if going up ( displacement.y in cartesian > 0 ) then, find x point
                    pt2.x = 0; //
                    pt2.y = pt1.y;
                    //pt2.y = -pt2.y; // again change to pixel coordinates
                    //std::cout << temp_gt_flow_image_path << " " << pt1 << " " << m << " " << pt2 << std::endl;
                } else {
                    pt2.x = Dataset::getFrameSize().width; //
                    pt2.y = pt1.y;
                    //pt2.y = -pt2.y; // again change to pixel coordinates
                    //std::cout << temp_gt_flow_image_path << " " << pt1 << " " << m << " " << pt2 << std::endl;
                }
            } else {

                if (line_in_cartesian_coordinates[1] >
                    0.0f) { //  if going up ( displacement.y in cartesian > 0 ) then, find x point on y = height
                    pt2.x = cvRound((Dataset::getFrameSize().height - c) / m); //
                    pt2.y = Dataset::getFrameSize().height;
                    pt2.y = -pt2.y; // again change to pixel coordinates
                    //std::cout << temp_gt_flow_image_path << " " << pt1 <<  " " << m << " " << pt2<< std::endl;
                } else if (line_in_cartesian_coordinates[1] <
                           0.0f) { //  if going down ( displacement.y in cartesian < 0 ) then, find x point on
                    // y = 0
                    pt2.x = int((-(float) Dataset::getFrameSize().height - c) / m); //
                    pt2.y = -Dataset::getFrameSize().height; // again change to pixel coordinates
                    pt2.y = -pt2.y; // again change to pixel coordinates
                    //std::cout << temp_gt_flow_image_path << " " << line <<  " " << m << " " << pt2.x << std::endl;
                }
            }

            line_parameters.push_back(std::make_pair(cv::Point2f(m, c), pt2));
        }
        m_obj_line_parameters.push_back(line_parameters);

    }
}

