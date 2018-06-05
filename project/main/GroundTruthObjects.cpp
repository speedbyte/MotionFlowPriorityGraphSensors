//
// Created by veikas on 28.01.18.
//

#include <iostream>
#include "GroundTruthObjects.h"


unsigned GroundTruthObjects::objectCurrentCount = 0;


void GroundTruthObjects::generate_object_base_point_displacement(ObjectMetaData gt_data) {

    //Initialization

    std::vector<STRUCT_GT_OBJECTS_ALL>  multiframe_object_base_all;
    std::vector<bool>  multiframe_object_base_visibility;
    m_object_base_point_displacement.clear();

    assert(gt_data.getAll().size() >= MAX_ITERATION_GT_SCENE_GENERATION_VECTOR);

    ushort frame_number = m_startPoint;

    std::cout << "generate_object_base_point_displacement with start_point " << m_startPoint << std::endl;

    for (ushort current_frame_index=0; current_frame_index < MAX_ITERATION_GT_SCENE_GENERATION_VECTOR; current_frame_index++) {
        // The first frame is the reference frame, hence it is skipped


        cv::Point2f gt_displacement = {0,0}, gt_displacement_inertial = {0,0}, gt_displacement_usk = {0,0};
        cv::Point2f gt_dimensions = {0,0};

        gt_dimensions.x = gt_data.getAll().at(frame_number).m_region_of_interest_px.width_px;
        gt_dimensions.y = gt_data.getAll().at(frame_number).m_region_of_interest_px.height_px;

        if (gt_data.getAll().at(frame_number).visMask && gt_data.getAll().at(frame_number).m_object_location_inertial_m.location_x_m != 0 ) {

            /*if ( gt_data.getAll().at(frame_number).m_object_occlusion.occlusion_inertial == 127 || gt_data.getAll().at(frame_number).visMask == 0
                || gt_data.getAll().at(frame_number).m_object_location_inertial_m.location_x_m == 0 || gt_data.getAll().at(frame_number).m_region_of_interest_px.x <= 0) { */
            multiframe_object_base_visibility.push_back((true));
        }
        else{
            multiframe_object_base_visibility.push_back((false));
        }


        if ( current_frame_index == 0 ) {

            m_object_base_point_displacement.push_back(std::make_pair(cv::Point2f(gt_data.getAll().at(frame_number).m_object_location_px.location_x_m, gt_data.getAll().at(frame_number).m_object_location_px.location_y_m) , cv::Point2f(0,0)));

        }
        else {

            //If we are at the end of the path vector, we need to reset our iterators
            if (frame_number >= MAX_ITERATION_RESULTS) {

                frame_number = 0;
                gt_displacement.x = gt_data.getAll().at(frame_number).m_object_location_px.cog_px.x - gt_data.getAll().at(MAX_ITERATION_RESULTS - 1).m_object_location_px.cog_px.x;
                gt_displacement.y = gt_data.getAll().at(frame_number).m_object_location_px.cog_px.y - gt_data.getAll().at(MAX_ITERATION_RESULTS - 1).m_object_location_px.cog_px.y;

                gt_displacement_inertial.x = gt_data.getAll().at(frame_number).m_object_location_inertial_m.location_x_m - gt_data.getAll().at(MAX_ITERATION_RESULTS - 1).m_object_location_inertial_m.location_x_m;
                gt_displacement_inertial.y = gt_data.getAll().at(frame_number).m_object_location_inertial_m.location_y_m - gt_data.getAll().at(MAX_ITERATION_RESULTS - 1).m_object_location_inertial_m.location_y_m;

                gt_displacement_usk.x = gt_data.getAll().at(frame_number).m_object_location_m.location_x_m - gt_data.getAll().at(MAX_ITERATION_RESULTS-(ushort)1).m_object_location_m.location_x_m;
                gt_displacement_usk.y = gt_data.getAll().at(frame_number).m_object_location_m.location_y_m - gt_data.getAll().at(MAX_ITERATION_RESULTS-(ushort)1).m_object_location_m.location_y_m;

            } else {

                gt_displacement.x = gt_data.getAll().at(frame_number).m_object_location_px.cog_px.x - gt_data.getAll().at(frame_number-(ushort)1).m_object_location_px.cog_px.x;
                gt_displacement.y = gt_data.getAll().at(frame_number).m_object_location_px.cog_px.y - gt_data.getAll().at(frame_number-(ushort)1).m_object_location_px.cog_px.y;

                gt_displacement_inertial.x = gt_data.getAll().at(frame_number).m_object_location_inertial_m.location_x_m - gt_data.getAll().at(frame_number-(ushort)1).m_object_location_inertial_m.location_x_m;
                gt_displacement_inertial.y = gt_data.getAll().at(frame_number).m_object_location_inertial_m.location_y_m - gt_data.getAll().at(frame_number-(ushort)1).m_object_location_inertial_m.location_y_m;

                gt_displacement_usk.x = gt_data.getAll().at(frame_number).m_object_location_m.location_x_m - gt_data.getAll().at(frame_number-(ushort)1).m_object_location_m.location_x_m;
                gt_displacement_usk.y = gt_data.getAll().at(frame_number).m_object_location_m.location_y_m - gt_data.getAll().at(frame_number-(ushort)1).m_object_location_m.location_y_m;


            }

            if ( frame_number != MAX_ITERATION_RESULTS && frame_number != 0 ) {
                if (gt_data.getAll().at(frame_number - (ushort) 1).m_object_location_inertial_m.location_x_m != 0) {
                    auto dist_inertial = cv::norm(gt_displacement_inertial);
                    auto dist_usk = cv::norm(gt_displacement_usk);
                    assert(std::round(dist_inertial + 0.5) / 1000 == std::round(dist_usk + 0.5) / 1000);
                }
            }

            // make m_centroid_displacement_with_coordinate_gt with smallest resolution.
            m_object_base_point_displacement.push_back(std::make_pair(gt_data.getAll().at(frame_number).m_object_location_px.cog_px, gt_displacement));

        }

        printf("%s, %u, %u , points %f, %f, displacement %f, %f dimension - %f %f\n", ((bool)multiframe_object_base_visibility.at(current_frame_index)?"true":"false"), current_frame_index, frame_number, gt_data.getAll().at(frame_number).m_object_location_px.cog_px.x, gt_data.getAll().at(frame_number).m_object_location_px.cog_px.y, gt_displacement.x, gt_displacement.y, gt_dimensions.x, gt_dimensions.y
        );

        multiframe_object_base_all.push_back(gt_data.getAll().at(frame_number));
        frame_number++;
    }
    m_object_extrapolated_all.push_back(multiframe_object_base_all);
    m_object_extrapolated_point_displacement.push_back(m_object_base_point_displacement);
    m_object_extrapolated_visibility.push_back(multiframe_object_base_visibility);

}


void GroundTruthObjects::generate_combined_sensor_data() {


    std::vector<std::pair<cv::Point2f, cv::Point2f> > sensor_combined_point_displacement;
    std::vector<bool> sensor_combined_visibility;

    for (ushort current_frame_index = 0; current_frame_index < MAX_ITERATION_GT_SCENE_GENERATION_VECTOR; current_frame_index++) {


        cv::Point2f pts;
        cv::Point2f displacement;
        bool visibility_1, visibility_2;

        pts = m_object_extrapolated_point_displacement.at(0).at(current_frame_index).first;
        visibility_1 = m_object_extrapolated_visibility.at(0).at(current_frame_index);
        visibility_2 = m_object_extrapolated_visibility.at(1).at(current_frame_index);

        displacement = m_object_extrapolated_point_displacement.at(0).at(current_frame_index).second;
        displacement += m_object_extrapolated_point_displacement.at(1).at(current_frame_index).second;
        displacement /= 2;
        sensor_combined_point_displacement.push_back(std::make_pair(pts, displacement));

        sensor_combined_visibility.push_back(visibility_1 | visibility_2);


    }

    m_object_extrapolated_point_displacement.push_back(sensor_combined_point_displacement);
    m_object_extrapolated_visibility.push_back(sensor_combined_visibility);
    std::vector<STRUCT_GT_OBJECTS_ALL> sensor_combined_object_extrapolated_all;

    std::copy(m_object_extrapolated_all.at(0).begin(), m_object_extrapolated_all.at(0).end(), std::back_inserter(sensor_combined_object_extrapolated_all));

    m_object_extrapolated_all.push_back(sensor_combined_object_extrapolated_all);

}