//
// Created by veikas on 08.02.18.
//

#include <iostream>
#include <map>
#include "Sensors.h"
#include <chrono>

using namespace std::chrono;

void Sensors::generate_sen_base_point_displacement(SensorMetaData gt_data) {

    //Initialization

    m_sen_base_all.clear();
    m_sen_base_visibility.clear();

    ushort frame_number;;
    if ( Dataset::GENERATE) {
        frame_number = 0;
    } else {
        frame_number = m_startPoint;
    }

    ushort FRAME_COUNT;
    if ( Dataset::GENERATE) {
        FRAME_COUNT = Dataset::MAX_GENERATION_DATASET;
    } else {
        FRAME_COUNT = Dataset::MAX_ITERATION_RESULTS;
    }

    std::cout << "generate_sen_base_point_displacement with start_point " << m_startPoint << std::endl;

    for (ushort current_frame_index=0; current_frame_index < FRAME_COUNT; current_frame_index++) {
        // The first frame is the reference frame, hence it is skipped

        /*
        printf("%s, %u, %u , points %f, %f, displacement %f, %f dimension - %f %f\n", (gt_data.getAll().at(frame_number).visMask?"true":"false"),
                current_frame_index,
                frame_number, gt_data.getAll().at(frame_number).m_sensor_location_px.location_x_m, gt_data.getAll().at(frame_number).m_sensor_location_px.location_y_m,
                (float)0, (float)0, gt_data.getAll().at(frame_number).m_sensor_location_carrier_m.location_x_m, gt_data.getAll().at(frame_number).m_sensor_location_carrier_m.location_y_m
        );*/


        //If we are at the end of the path vector, we need to reset our iterators
        if (frame_number >= gt_data.getAll().size()) {

            frame_number = 0;
            m_sen_base_visibility.push_back(gt_data.getAll().at(frame_number).visMask);

        } else {
            m_sen_base_visibility.push_back(gt_data.getAll().at(frame_number).visMask);

        }

        m_sen_base_all.push_back(gt_data.getAll().at(frame_number));
        frame_number++;
    }

    m_sen__all.push_back(m_sen_base_all);
    m_sen__visibility.push_back(m_sen_base_visibility);

}

