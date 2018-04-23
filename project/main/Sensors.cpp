//
// Created by veikas on 08.02.18.
//

#include <iostream>
#include <map>
#include "Sensors.h"
#include <chrono>

using namespace std::chrono;

void Sensors::generate_sen_base_pixel_position_pixel_displacement(SensorMetaData gt_data) {

    //Initialization

    ushort current_index = m_startPoint;

    std::cout << "generate_sen_base_pixel_position_pixel_displacement with start_point " << m_startPoint << std::endl;

    for (ushort frame_count=0; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_VECTOR; frame_count++) {
        // The first frame is the reference frame, hence it is skipped

        printf("%s, %u, %u , points %f, %f, displacement %f, %f dimension - %f %f\n", (gt_data.getAll().at(current_index).visMask?"true":"false"),
                frame_count,
                current_index, gt_data.getAll().at(current_index).m_sensor_location_px.location_x_m, gt_data.getAll().at(current_index).m_sensor_location_px.location_y_m,
                (float)0, (float)0, gt_data.getAll().at(current_index).m_sensor_location_carrier_m.location_x_m, gt_data.getAll().at(current_index).m_sensor_location_carrier_m.location_y_m
        );


        //If we are at the end of the path vector, we need to reset our iterators
        if (current_index >= gt_data.getAll().size()) {

            current_index = 0;
            m_sen_base_visibility.push_back(gt_data.getAll().at(current_index).visMask);

        } else {
            m_sen_base_visibility.push_back(gt_data.getAll().at(current_index).visMask);

        }

        m_sen_base_all.push_back(gt_data.getAll().at(current_index));
        current_index++;
    }
}

void Sensors::generate_sen_extrapolated_pixel_position_pixel_displacement() {


    for ( unsigned frame_skip = 1; frame_skip < MAX_SKIPS ; frame_skip++ ) {

        std::vector<bool>  multiframe_visibility;
        std::vector<STRUCT_GT_SENSORS_ALL> multiframe_all;


        std::cout << "generate_sen_extrapolated_pixel_position_pixel_displacement for frame_skip " << frame_skip << std::endl;
        unsigned long FRAME_COUNT = m_sen_base_all.size();
        assert(FRAME_COUNT>0);

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

            // The first frame is the reference frame. frame skip 1 means no skips
            // The below code has to go through consecutive frames
            if ((frame_count % frame_skip != 0)) {

            }
            else {

                if ( m_sen_base_visibility.at(frame_count) == false) {
                    // Make all 0

                    multiframe_visibility.push_back(false);
                    multiframe_all.push_back(m_sen_base_all.at(frame_count));

                } else {

                    multiframe_all.push_back(m_sen_base_all.at(frame_count));
                }
            }
        }
        m_sen_extrapolated_visibility.push_back(multiframe_visibility);
        m_sen_extrapolated_all.push_back(multiframe_all);

    }
}

