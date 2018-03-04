//
// Created by veikas on 28.01.18.
//

#include <iostream>
#include "GroundTruthObjects.h"


unsigned GroundTruthObjects::objectCurrentCount = 0;


void GroundTruthObjects::generate_obj_base_pixel_point_pixel_displacement() {

    //Initialization

    ushort current_index = m_startPoint;

    std::cout << "generate_obj_base_pixel_point_pixel_displacement with start_point " << m_startPoint << std::endl;

    for (ushort frame_count=0; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_VECTOR; frame_count++) {
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
            m_obj_base_visibility.push_back(m_obj_trajectory.getVisibility().at(current_index));
        }
        else {

            printf("%s, %u, %u , %f, %f, %f, %f\n", (m_obj_trajectory.getVisibility().at(current_index)?"true":"false"),
                   frame_count,
                   current_index, m_obj_trajectory.getTrajectory().at(current_index).x, m_obj_trajectory.getTrajectory().at(current_index).y,
                   (float)0, (float)0);
            m_obj_base_pixel_point_pixel_displacement.push_back(std::make_pair(m_obj_trajectory.getTrajectory().at(current_index) , cv::Point2f(0,0)));
            m_obj_base_visibility.push_back(m_obj_trajectory.getVisibility().at(current_index));
        }
        current_index++;
    }
}

void GroundTruthObjects::generate_obj_base_shape_dimensions() {

    //Initialization

    ushort current_index = m_startPoint;

    std::cout << "generate_obj_base_shape_dimensions with start_point " << m_startPoint << std::endl;

    for (ushort frame_count=0; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_VECTOR; frame_count++) {
        // The first frame is the reference frame, hence it is skipped

        cv::Point2f gt_dimensions = {0,0};

        //If we are at the end of the path vector, we need to reset our iterators
        if (current_index >= m_obj_dimensions.getTrajectoryDimensions().size()) {
            current_index = 0;
            gt_dimensions.x = m_obj_dimensions.getTrajectoryDimensions().at(current_index).x ;
            gt_dimensions.y = m_obj_dimensions.getTrajectoryDimensions().at(current_index).y ;

        } else {

            gt_dimensions.x = m_obj_dimensions.getTrajectoryDimensions().at(current_index).x ;
            gt_dimensions.y = m_obj_dimensions.getTrajectoryDimensions().at(current_index).y ;

        }

        printf("%s, %u, %u , %f, %f\n", (m_obj_trajectory.getVisibility().at(current_index)?"true":"false"),
               frame_count,
               current_index, gt_dimensions.x, gt_dimensions.y);

        // make m_flowvector_with_coordinate_gt with smallest resolution.
        m_obj_base_shape_dimensions.push_back(gt_dimensions);
    current_index++;
    }
}

void GroundTruthObjects::generate_obj_extrapolated_pixel_point_pixel_displacement(const unsigned &max_skips) {

    float temp_flow_x(0);
    float temp_flow_y(0);

    for ( unsigned frame_skip = 1; frame_skip < max_skips ; frame_skip++ ) {

        std::vector<std::pair<cv::Point2f, cv::Point2f> > multiframe_flowvector;
        std::vector<bool>  multiframe_visibility;

        std::cout << "generate_obj_extrapolated_pixel_point_pixel_displacement for frame_skip " << frame_skip << std::endl;
        unsigned long FRAME_COUNT = m_obj_base_pixel_point_pixel_displacement.size();
        assert(FRAME_COUNT>0);

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

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


void GroundTruthObjects::generate_obj_extrapolated_shape_pixel_point_pixel_displacement_pixel_visibility(const unsigned &max_skips ) {

// object image_data_and_shape

    for (unsigned frame_skip = 1; frame_skip < max_skips; frame_skip++) {

        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_base_movement;
        std::vector<std::vector<bool>  > outer_base_visiblity;

        std::cout << "generate_obj_extrapolated_shape_pixel_point_pixel_displacement_pixel_visibility for frame_skip " << frame_skip << std::endl;
        unsigned long FRAME_COUNT = m_obj_extrapolated_pixel_point_pixel_displacement.at(frame_skip - 1).size();
        assert(FRAME_COUNT>0);
        for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement
            cv::Point2f gt_next_pts = m_obj_extrapolated_pixel_point_pixel_displacement.at(frame_skip - 1).at(frame_count).first;
            cv::Point2f gt_displacement = m_obj_extrapolated_pixel_point_pixel_displacement.at(frame_skip - 1).at(frame_count).second;

            bool visibility = m_obj_extrapolated_visibility.at(frame_skip - 1).at(frame_count);

            std::vector<std::pair<cv::Point2f, cv::Point2f> > base_movement;
            std::vector<bool> base_visibility;

            for (unsigned j = 0; j < m_ObjectWidth; j++) {
                for (unsigned k = 0; k < m_ObjectHeight; k++) {
                    base_movement.push_back(std::make_pair(cv::Point2f(gt_next_pts.x + j, gt_next_pts.y +
                                                                                              k), gt_displacement));
                    base_visibility.push_back(visibility);
                }
            }
            outer_base_movement.push_back(base_movement);
            outer_base_visiblity.push_back(base_visibility);
        }
        m_obj_extrapolated_shape_pixel_point_pixel_displacement.push_back(outer_base_movement);
        m_obj_extrapolated_shape_visibility.push_back(outer_base_visiblity);
        generate_obj_extrapolated_stencil_pixel_point_pixel_displacement(outer_base_movement);
    }
}

void GroundTruthObjects::generate_obj_extrapolated_stencil_pixel_point_pixel_displacement(std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_stencil_movement  ) {

    // object image_data_and_shape
    m_obj_extrapolated_stencil_pixel_point_pixel_displacement = m_obj_extrapolated_shape_pixel_point_pixel_displacement;

    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

        std::cout << "generate_obj_extrapolated_stencil_pixel_point_pixel_displacement for frame_skip " << frame_skip << std::endl;
        unsigned long FRAME_COUNT = m_obj_extrapolated_pixel_point_pixel_displacement.at(frame_skip - 1).size();
        assert(FRAME_COUNT>0);
        m_obj_extrapolated_stencil_pixel_point_pixel_displacement.push_back(outer_stencil_movement);
    }
}
