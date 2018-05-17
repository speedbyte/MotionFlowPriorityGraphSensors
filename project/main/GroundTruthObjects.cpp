//
// Created by veikas on 28.01.18.
//

#include <iostream>
#include "GroundTruthObjects.h"


unsigned GroundTruthObjects::objectCurrentCount = 0;


void GroundTruthObjects::generate_object_base_point_displacement(ObjectMetaData gt_data) {

    //Initialization

    m_object_base_all.clear();
    m_object_base_point_displacement.clear();
    m_object_base_visibility.clear();


    assert(gt_data.getAll().size() >= MAX_ITERATION_GT_SCENE_GENERATION_VECTOR);

    ushort current_index = m_startPoint;

    std::cout << "generate_object_base_point_displacement with start_point " << m_startPoint << std::endl;

    for (ushort frame_count=0; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_VECTOR; frame_count++) {
        // The first frame is the reference frame, hence it is skipped


        cv::Point2f gt_displacement = {0,0}, gt_displacement_inertial = {0,0}, gt_displacement_usk = {0,0};
        cv::Point2f gt_dimensions = {0,0};

        gt_dimensions.x = gt_data.getAll().at(current_index).m_object_dimensions_px.dim_width_m;
        gt_dimensions.y = gt_data.getAll().at(current_index).m_object_dimensions_px.dim_height_m;

        if ( frame_count == 0 ) {

            printf("%s, %u, %u , points %f, %f, displacement %f, %f dimension - %f %f\n", ((bool)gt_data.getAll().at(current_index).m_object_occlusion.occlusion_inertial?"false":"true"),
                   frame_count,
                   current_index, gt_data.getAll().at(current_index).m_object_location_px.cog_px.x, gt_data.getAll().at(current_index).m_object_location_px.cog_px.y, gt_displacement.x, gt_displacement.y, gt_dimensions.x, gt_dimensions.y
            );

            m_object_base_point_displacement.push_back(std::make_pair(cv::Point2f(gt_data.getAll().at(current_index).m_object_location_px.location_x_m, gt_data.getAll().at(current_index).m_object_location_px.location_y_m) , cv::Point2f(0,0)));

        }
        else {

            //If we are at the end of the path vector, we need to reset our iterators
            if (current_index >= gt_data.getAll().size()) {

                current_index = 0;
                gt_displacement.x = gt_data.getAll().at(current_index).m_object_location_px.cog_px.x - gt_data.getAll().at(gt_data.getAll().size() - 1).m_object_location_px.cog_px.x;
                gt_displacement.y = gt_data.getAll().at(current_index).m_object_location_px.cog_px.y - gt_data.getAll().at(gt_data.getAll().size() - 1).m_object_location_px.cog_px.y;

                gt_displacement_inertial.x = gt_data.getAll().at(current_index).m_object_location_inertial_m.location_x_m - gt_data.getAll().at(gt_data.getAll().size() - 1).m_object_location_inertial_m.location_x_m;
                gt_displacement_inertial.y = gt_data.getAll().at(current_index).m_object_location_inertial_m.location_y_m - gt_data.getAll().at(gt_data.getAll().size() - 1).m_object_location_inertial_m.location_y_m;

                gt_displacement_usk.x = gt_data.getAll().at(current_index).m_object_location_m.location_x_m - gt_data.getAll().at(current_index-(ushort)1).m_object_location_m.location_x_m;
                gt_displacement_usk.y = gt_data.getAll().at(current_index).m_object_location_m.location_y_m - gt_data.getAll().at(current_index-(ushort)1).m_object_location_m.location_y_m;

            } else {

                gt_displacement.x = gt_data.getAll().at(current_index).m_object_location_px.cog_px.x - gt_data.getAll().at(current_index-(ushort)1).m_object_location_px.cog_px.x;
                gt_displacement.y = gt_data.getAll().at(current_index).m_object_location_px.cog_px.y - gt_data.getAll().at(current_index-(ushort)1).m_object_location_px.cog_px.y;

                gt_displacement_inertial.x = gt_data.getAll().at(current_index).m_object_location_inertial_m.location_x_m - gt_data.getAll().at(current_index-(ushort)1).m_object_location_inertial_m.location_x_m;
                gt_displacement_inertial.y = gt_data.getAll().at(current_index).m_object_location_inertial_m.location_y_m - gt_data.getAll().at(current_index-(ushort)1).m_object_location_inertial_m.location_y_m;

                gt_displacement_usk.x = gt_data.getAll().at(current_index).m_object_location_m.location_x_m - gt_data.getAll().at(current_index-(ushort)1).m_object_location_m.location_x_m;
                gt_displacement_usk.y = gt_data.getAll().at(current_index).m_object_location_m.location_y_m - gt_data.getAll().at(current_index-(ushort)1).m_object_location_m.location_y_m;


            }

            if ( gt_data.getAll().at(current_index-(ushort)1).m_object_location_inertial_m.location_x_m != 0 ) {
                auto dist_inertial = cv::norm(gt_displacement_inertial);
                auto dist_usk = cv::norm(gt_displacement_usk);
                assert(std::round(dist_inertial+0.5)/1000 == std::round(dist_usk+0.5)/1000);
            }


            printf("%s, %u, %u , points %f, %f, displacement %f, %f dimension - %f %f\n", ((bool)gt_data.getAll().at(current_index).m_object_occlusion.occlusion_inertial?"false":"true"),
                   frame_count,
                   current_index, gt_data.getAll().at(current_index).m_object_location_px.cog_px.x, gt_data.getAll().at(current_index).m_object_location_px.cog_px.y,
                   gt_displacement.x, gt_displacement.y, gt_dimensions.x, gt_dimensions.y);

            // make m_flowvector_with_coordinate_gt with smallest resolution.
            m_object_base_point_displacement.push_back(std::make_pair(gt_data.getAll().at(current_index).m_object_location_px.cog_px, gt_displacement));

        }
        if ( gt_data.getAll().at(current_index).m_object_occlusion.occlusion_inertial == 127 || gt_data.getAll().at(current_index).visMask == 0
                || gt_data.getAll().at(current_index).m_object_location_inertial_m.location_x_m == 0 || gt_data.getAll().at(current_index).m_region_of_interest_px.x <= 0) {
            m_object_base_visibility.push_back((false));
        }
        else{
            m_object_base_visibility.push_back((true));
        }
        m_object_base_all.push_back(gt_data.getAll().at(current_index));
        current_index++;
    }
    m_object_all.push_back(m_object_base_all);
    m_object_pixel_position_pixel_displacement.push_back(m_object_base_point_displacement);
    m_object_visibility.push_back(m_object_base_visibility);

}
