//
// Created by veikas on 28.01.18.
//

#include <iostream>
#include "Objects.h"


unsigned Objects::objectCurrentCount = 0;



void Objects::generate_obj_base_pixel_point_pixel_displacement() {

    //Initialization

    ushort current_index = m_startPoint;

    std::cout << "start point "<< current_index << std::endl;

    for (ushort frame_count=0; frame_count < MAX_ITERATION_GT; frame_count++) {
        // The first frame is the reference frame, hence it is skipped

        if ( frame_count > 0 ) {

            cv::Point2f gt_next_pts = {0,0}, gt_displacement = {0,0};

            //If we are at the end of the path vector, we need to reset our iterators
            if (current_index >= m_obj_trajectory.getTrajectory().size()) {
                current_index = 0;
                gt_displacement.x = m_obj_trajectory.getTrajectory().at(current_index).x - m_obj_trajectory.getTrajectory().at
                        (m_obj_trajectory.getTrajectory().size() - 1).x;
                gt_displacement.y = m_obj_trajectory.getTrajectory().at(current_index).y - m_obj_trajectory.getTrajectory().at
                        (m_obj_trajectory.getTrajectory().size() - 1).y;
                gt_next_pts = m_obj_trajectory.getTrajectory().at(current_index);

            } else {

                gt_displacement.x = m_obj_trajectory.getTrajectory().at(current_index).x - m_obj_trajectory.getTrajectory().at
                        (current_index - (ushort) 1).x;
                gt_displacement.y = m_obj_trajectory.getTrajectory().at(current_index).y - m_obj_trajectory.getTrajectory().at
                        (current_index - (ushort) 1).y;
                gt_next_pts = m_obj_trajectory.getTrajectory().at(current_index);

            }

            printf("%s, %u, %u , %f, %f, %f, %f\n", (m_obj_trajectory.getVisibility().at(current_index)?"true":"false"),
                    frame_count,
                   current_index, gt_next_pts.x, gt_next_pts.y,
                   gt_displacement.x, gt_displacement.y);

            // make m_flowvector_with_coordinate_gt with smallest resolution.
            m_obj_base_pixel_point_pixel_displacement.push_back(std::make_pair(gt_next_pts, gt_displacement));
            m_obj_base_visibility.push_back(true);
        }
        else {
            m_obj_base_visibility.push_back(false);
            m_obj_base_pixel_point_pixel_displacement.push_back(std::make_pair(cv::Point2f(0,0), cv::Point2f(0,0)));
        }
        current_index++;
    }
}

void Objects::generate_obj_extrapolated_pixel_point_pixel_displacement(const unsigned &max_skips) {

    float temp_flow_x(0);
    float temp_flow_y(0);

    for ( unsigned frame_skip = 1; frame_skip < max_skips ; frame_skip++ ) {

        std::vector<std::pair<cv::Point2f, cv::Point2f> > multiframe_flowvector;
        std::vector<bool>  multiframe_visibility;

        std::cout << "creating flow files for frame_skip " << frame_skip << std::endl;
        unsigned long FRAME_COUNT = m_obj_base_pixel_point_pixel_displacement.size();
        assert(FRAME_COUNT>0);

        for (ushort frame_count = 1; frame_count < FRAME_COUNT; frame_count++) {

            // The first frame is the reference frame. frame skip 1 means no skips
            // The below code has to go through consecutive frames
            if ((frame_count % frame_skip != 0)) {
                temp_flow_x += m_obj_base_pixel_point_pixel_displacement.at(frame_count).second.x;
                temp_flow_y += m_obj_base_pixel_point_pixel_displacement.at(frame_count).second.y;
            }
            else {
                temp_flow_x += m_obj_base_pixel_point_pixel_displacement.at(frame_count).second.x;
                temp_flow_y += m_obj_base_pixel_point_pixel_displacement.at(frame_count).second.y;

                if ( m_obj_base_visibility.at(frame_count) == false) {
                    // Make all 0
                    multiframe_flowvector.push_back
                            (std::make_pair(m_obj_base_pixel_point_pixel_displacement.at
                                    (frame_count).first, cv::Point2f(0,0)));
                    multiframe_visibility.push_back(false);

                } else {
                    multiframe_flowvector.push_back
                            (std::make_pair(m_obj_base_pixel_point_pixel_displacement.at
                                    (frame_count).first, cv::Point2f(temp_flow_x, temp_flow_y)));
                    multiframe_visibility.push_back(true);
                }
                temp_flow_x = 0, temp_flow_y = 0;
            }
        }
        m_obj_extrapolated_pixel_point_pixel_displacement.push_back(multiframe_flowvector);
        m_obj_extrapolated_visibility.push_back(multiframe_visibility);
    }
}


void Objects::generate_obj_extrapolated_shape_pixel_point_pixel_displacement(const unsigned &max_skips ) {

// object image_data_and_shape
    int width = m_image_data_and_shape.get().cols;
    int height = m_image_data_and_shape.get().rows;

    for (unsigned frame_skip = 1; frame_skip < max_skips; frame_skip++) {

        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_base_movement;
        std::vector<std::vector<bool>  > outer_base_visiblity;

        unsigned long FRAME_COUNT = m_obj_extrapolated_pixel_point_pixel_displacement.at(frame_skip - 1).size();
        assert(FRAME_COUNT>0);
        for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement
            cv::Point2f gt_next_pts = m_obj_extrapolated_pixel_point_pixel_displacement.at(frame_skip - 1).at(frame_count).first;
            cv::Point2f gt_displacement = m_obj_extrapolated_pixel_point_pixel_displacement.at(frame_skip - 1).at(frame_count).second;

            std::vector<std::pair<cv::Point2f, cv::Point2f> > base_movement;
            std::vector<bool> base_visibility;

            for (unsigned j = 0; j < width; j++) {
                for (unsigned k = 0; k < height; k++) {
                    base_movement.push_back(std::make_pair(cv::Point2f(gt_next_pts.x + j, gt_next_pts.y +
                                                                                              k), gt_displacement));
                    base_visibility.push_back(true);
                }
            }
            outer_base_movement.push_back(base_movement);
            outer_base_visiblity.push_back(base_visibility);
        }
        m_obj_extrapolated_shape_pixel_point_pixel_displacement.push_back(outer_base_movement);
        m_obj_extrapolated_shape_visibility.push_back(outer_base_visiblity);
    }
}


void Objects::generate_obj_extrapolated_pixel_centroid_pixel_displacement_mean( const unsigned &max_skips) {


    for (unsigned frame_skip = 1; frame_skip < max_skips; frame_skip++) {
        std::vector<std::pair<cv::Point2f, cv::Point2f> > multiframe_flowvector;
        std::vector<bool> multiframe_visibility;
        unsigned long FRAME_COUNT = m_obj_extrapolated_pixel_point_pixel_displacement.at(frame_skip - 1).size();
        assert(FRAME_COUNT>0);
        for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement
            float next_pts_x = 0.0f;
            float next_pts_y = 0.0f;
            float displacement_vector_x = 0.0f;
            float displacement_vector_y = 0.0f;

            const unsigned CLUSTER_SIZE = (unsigned)m_obj_extrapolated_shape_pixel_point_pixel_displacement.at
                            (frame_skip - 1).at(frame_count).size();

            for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {
                cv::Point2f pts = m_obj_extrapolated_shape_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).first;
                cv::Point2f gt_displacement = m_obj_extrapolated_shape_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).second;
                next_pts_x += pts.x ;
                next_pts_y += pts.y ;
                displacement_vector_x += gt_displacement.x ;
                displacement_vector_y += gt_displacement.y ;
            }
            next_pts_x /= (float)CLUSTER_SIZE;
            next_pts_y /= (float)CLUSTER_SIZE;
            displacement_vector_x =  displacement_vector_x / (float) CLUSTER_SIZE;
            displacement_vector_y = displacement_vector_y / (float) CLUSTER_SIZE;

            // I should return the vector instead of points and then normalize it.
            multiframe_flowvector.push_back(std::make_pair(cv::Point2f(next_pts_x, next_pts_y), cv::Point2f
                    (displacement_vector_x, displacement_vector_y)));
            multiframe_visibility.push_back(true);
        }
        m_obj_extrapolated_pixel_centroid_pixel_displacement_mean.push_back(multiframe_flowvector);
        m_obj_extrapolated_mean_visibility.push_back(multiframe_visibility);
    }


    //TODO - clean up this section.
    for (unsigned frame_skip = 1; frame_skip < max_skips; frame_skip++) {
        std::vector<std::pair<cv::Point2f, cv::Point2f> > line_parameters;
        const unsigned long FRAME_COUNT = m_obj_extrapolated_pixel_centroid_pixel_displacement_mean.at(frame_skip - 1)
                .size();
        assert(FRAME_COUNT>0);
        for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement

            if ( m_obj_extrapolated_mean_visibility.at(frame_skip-1).at(frame_count) == true ) {
                cv::Point2f next_pts = m_obj_extrapolated_pixel_centroid_pixel_displacement_mean.at(frame_skip - 1)
                        .at(frame_count).first;
                cv::Point2f  displacement_vector = m_obj_extrapolated_pixel_centroid_pixel_displacement_mean.at
                        (frame_skip - 1).at(frame_count).second;

                float m, c;
                m = displacement_vector.y / displacement_vector.x;
                c = next_pts.y - m * next_pts.x;  // c = y - mx

                //float d = (float) sqrt((double) displacement_vector.x * displacement_vector.x +
                //                       (double) displacement_vector.y * displacement_vector.y);
                //displacement_vector.x /= d; // normalized vector in x
                //displacement_vector.y /= d; // normalized vector in y

                cv::Point2f pt2;

                if (std::isinf(m)) {
                    if (displacement_vector.y >  0.0f) {  // going up
                        pt2.x = next_pts.x;
                        pt2.y = Dataset::getFrameSize().height;
                    } else {  // going down
                        pt2.x = next_pts.x;
                        pt2.y = 0;
                    }
                } else if (m == 0) {
                    //std::cout << frame_count << " " << next_pts<<  " " << m << " " << displacement_vector << std::endl;
                    if (std::signbit(m)) { //  going left
                        pt2.x = 0;
                        pt2.y = next_pts.y;
                    } else {  // going right
                        pt2.x = Dataset::getFrameSize().width;
                        pt2.y = next_pts.y;
                    }
                } else {
                    //std::cout << frame_count << " " << next_pts <<  " " << m << " " << pt2<< std::endl;
                    if (displacement_vector.y >  0.0f) {
                        pt2.x = (Dataset::getFrameSize().height - c) / m; //
                        pt2.y = Dataset::getFrameSize().height;
                    } else if (displacement_vector.y < 0.0f) {
                        pt2.x = (-c/m); //
                        pt2.y = 0;
                    }
                }

                line_parameters.push_back(std::make_pair(cv::Point2f(m, c), pt2));
            }
            else {
                line_parameters.push_back(std::make_pair(cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f)));
            }

        }
        m_obj_line_parameters.push_back(line_parameters);

    }
}

