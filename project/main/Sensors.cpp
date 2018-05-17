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
    ushort current_index = m_startPoint;

    std::cout << "generate_sen_base_point_displacement with start_point " << m_startPoint << std::endl;

    for (ushort frame_count=0; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_VECTOR; frame_count++) {
        // The first frame is the reference frame, hence it is skipped

        /*
        printf("%s, %u, %u , points %f, %f, displacement %f, %f dimension - %f %f\n", (gt_data.getAll().at(current_index).visMask?"true":"false"),
                frame_count,
                current_index, gt_data.getAll().at(current_index).m_sensor_location_px.location_x_m, gt_data.getAll().at(current_index).m_sensor_location_px.location_y_m,
                (float)0, (float)0, gt_data.getAll().at(current_index).m_sensor_location_carrier_m.location_x_m, gt_data.getAll().at(current_index).m_sensor_location_carrier_m.location_y_m
        );*/


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

    m_sen__all.push_back(m_sen_base_all);
    m_sen__visibility.push_back(m_sen_base_visibility);

}

