//
// Created by veikas on 28.01.18.
//

#include <iostream>
#include "GroundTruthObjects.h"


unsigned GroundTruthObjects::objectCurrentCount = 0;


void GroundTruthObjects::generate_obj_base_pixel_position_pixel_displacement(ObjectMetaData gt_data) {

    //Initialization

    m_obj_base_all.clear();
    m_obj_base_pixel_position_pixel_displacement.clear();
    m_obj_base_visibility.clear();


    assert(gt_data.getAll().size() >= MAX_ITERATION_GT_SCENE_GENERATION_VECTOR);

    ushort current_index = m_startPoint;

    std::cout << "generate_obj_base_pixel_position_pixel_displacement with start_point " << m_startPoint << std::endl;

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

            m_obj_base_pixel_position_pixel_displacement.push_back(std::make_pair(cv::Point2f(gt_data.getAll().at(current_index).m_object_location_px.location_x_m, gt_data.getAll().at(current_index).m_object_location_px.location_y_m) , cv::Point2f(0,0)));

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

            auto dist_inertial = cv::norm(gt_displacement_inertial);
            auto dist_usk = cv::norm(gt_displacement_usk);
            assert(std::round(dist_inertial+0.5)/1000 == std::round(dist_usk+0.5)/1000);

            printf("%s, %u, %u , points %f, %f, displacement %f, %f dimension - %f %f\n", ((bool)gt_data.getAll().at(current_index).m_object_occlusion.occlusion_inertial?"false":"true"),
                   frame_count,
                   current_index, gt_data.getAll().at(current_index).m_object_location_px.cog_px.x, gt_data.getAll().at(current_index).m_object_location_px.cog_px.y,
                   gt_displacement.x, gt_displacement.y, gt_dimensions.x, gt_dimensions.y);

            // make m_flowvector_with_coordinate_gt with smallest resolution.
            m_obj_base_pixel_position_pixel_displacement.push_back(std::make_pair(gt_data.getAll().at(current_index).m_object_location_px.cog_px, gt_displacement));

        }
        if ( gt_data.getAll().at(current_index).m_object_occlusion.occlusion_inertial > 80) {
            m_obj_base_visibility.push_back((false));
        }
        else{
            m_obj_base_visibility.push_back((true));
        }
        m_obj_base_all.push_back(gt_data.getAll().at(current_index));
        current_index++;
    }
    m_obj_extrapolated_all.push_back(m_obj_base_all);
    m_obj_extrapolated_pixel_position_pixel_displacement.push_back(m_obj_base_pixel_position_pixel_displacement);
    m_obj_extrapolated_visibility.push_back(m_obj_base_visibility);

}

/*
void GroundTruthObjects::generate_obj_extrapolated_pixel_position_pixel_displacement_old() {

    float temp_flow_x(0);
    float temp_flow_y(0);

    for ( unsigned sensor_index = 1; sensor_index < MAX_SKIPS ; sensor_index++ ) {

        std::vector<std::pair<cv::Point2f, cv::Point2f> > multiframe_flowvector;
        std::vector<bool>  multiframe_visibility;
        std::vector<STRUCT_GT_OBJECTS_ALL> multiframe_all;


        std::cout << "generate_obj_extrapolated_pixel_position_pixel_displacement for sensor_index " << sensor_index << std::endl;
        unsigned long FRAME_COUNT = m_obj_base_pixel_position_pixel_displacement.size();
        assert(FRAME_COUNT>0);

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

            // The first frame is the reference frame. frame skip 1 means no skips
            // The below code has to go through consecutive frames
            if ((frame_count % sensor_index != 0)) {
                temp_flow_x += m_obj_base_pixel_position_pixel_displacement.at(frame_count).second.x;
                temp_flow_y += m_obj_base_pixel_position_pixel_displacement.at(frame_count).second.y;
            }
            else {
                temp_flow_x += m_obj_base_pixel_position_pixel_displacement.at(frame_count).second.x;
                temp_flow_y += m_obj_base_pixel_position_pixel_displacement.at(frame_count).second.y;

                if ( m_obj_base_visibility.at(frame_count) == false) {
                    // Make all 0
                    multiframe_flowvector.push_back
                            (std::make_pair(m_obj_base_pixel_position_pixel_displacement.at
                                    (frame_count).first, cv::Point2f(0,0)));
                    multiframe_visibility.push_back(false);
                    multiframe_all.push_back(m_obj_base_all.at(frame_count));

                } else {
                    multiframe_flowvector.push_back
                            (std::make_pair(m_obj_base_pixel_position_pixel_displacement.at
                                    (frame_count).first, cv::Point2f(temp_flow_x, temp_flow_y)));
                    multiframe_visibility.push_back(true);
                    multiframe_all.push_back(m_obj_base_all.at(frame_count));

                }
                temp_flow_x = 0, temp_flow_y = 0;
            }
        }

        m_obj_extrapolated_pixel_position_pixel_displacement.push_back(multiframe_flowvector);
        m_obj_extrapolated_visibility.push_back(multiframe_visibility);
        m_obj_extrapolated_all.push_back(multiframe_all);

    }
}
*/

void GroundTruthObjects::generate_obj_extrapolated_shape_pixel_point_pixel_displacement_pixel_visibility( ) {

// object image_data_and_shape
    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > dummy;

    for (unsigned sensor_index = 1; sensor_index < MAX_SKIPS; sensor_index++) {

        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_base_movement;
        std::vector<std::vector<bool>  > outer_base_visiblity;

        std::cout << "generate_obj_extrapolated_shape_pixel_point_pixel_displacement_pixel_visibility for sensor_index " << sensor_index << std::endl;
        unsigned long FRAME_COUNT = m_obj_extrapolated_pixel_position_pixel_displacement.at(sensor_index - 1).size();
        assert(FRAME_COUNT>0);
        for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement

            cv::Point2f gt_roi_pts = cv::Point2f(m_obj_extrapolated_all.at(sensor_index-1).at(frame_count).m_region_of_interest_px.x, m_obj_extrapolated_all.at(sensor_index-1).at(frame_count).m_region_of_interest_px.y);

            cv::Point2f gt_displacement = m_obj_extrapolated_pixel_position_pixel_displacement.at(sensor_index - 1).at(frame_count).second;

            bool visibility = m_obj_extrapolated_visibility.at(sensor_index - 1).at(frame_count);

            std::vector<std::pair<cv::Point2f, cv::Point2f> > base_movement;
            std::vector<bool> base_visibility;

            int ObjectWidth = cvRound(m_obj_extrapolated_all.at(sensor_index-1).at(frame_count).m_object_dimensions_px.dim_width_m);
            int ObjectHeight = cvRound(m_obj_extrapolated_all.at(sensor_index-1).at(frame_count).m_object_dimensions_px.dim_height_m);

            for (unsigned j = 0; j < ObjectWidth; j+=1) {
                for (unsigned k = 0; k < ObjectHeight; k+=1) {
                    if ( j%STENCIL_GRID_COMPRESSOR == 0 && k%STENCIL_GRID_COMPRESSOR == 0 ) { // only entertain multiple of x pixels to reduce data
                        base_movement.push_back(std::make_pair(cv::Point2f(gt_roi_pts.x + j, gt_roi_pts.y +
                                                                                              k), gt_displacement));
                        base_visibility.push_back(visibility);
                    }
                }
            }
            if ( !visibility ) {
                base_movement.push_back(std::make_pair(cv::Point2f(0,0), cv::Point2f(0,0)));
                base_visibility.push_back(visibility);
            }
            outer_base_movement.push_back(base_movement);
            outer_base_visiblity.push_back(base_visibility);
        }
        m_obj_extrapolated_shape_pixel_point_pixel_displacement.push_back(outer_base_movement);
        m_obj_extrapolated_shape_visibility.push_back(outer_base_visiblity);
    }
    generate_obj_extrapolated_stencil_pixel_point_pixel_displacement(dummy);

}

void GroundTruthObjects::generate_obj_extrapolated_stencil_pixel_point_pixel_displacement(std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_stencil_movement  ) {

    // object image_data_and_shape
    //m_obj_extrapolated_stencil_pixel_point_pixel_displacement = m_obj_extrapolated_shape_pixel_point_pixel_displacement;


        std::cout << "generate_obj_extrapolated_stencil_pixel_point_pixel_displacement for sensor_index "  << std::endl;
        m_obj_extrapolated_stencil_pixel_point_pixel_displacement = m_obj_extrapolated_shape_pixel_point_pixel_displacement; //.push_back(outer_stencil_movement);
}
