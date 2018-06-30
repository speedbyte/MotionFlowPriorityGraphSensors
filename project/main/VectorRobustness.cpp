//
// Created by veikas on 30.06.18.
//

#include <opencv/cv.hpp>
#include <iostream>
#include <map>
#include "RobustnessIndex.h"
#include "Utils.h"


void VectorRobustness::generateVectorRobustness(ushort SENSOR_COUNT, const OpticalFlow &opticalFlow_base, const OpticalFlow &opticalFlow) {

    auto position = opticalFlow.getResultOrdner().find('/');
    std::string suffix = opticalFlow.getResultOrdner().replace(position, 1, "_");

    unsigned COUNT;
    if ( suffix == "_ground_truth") {
        COUNT = 1;
    }
    else {
        COUNT = DATAFILTER_COUNT;
    }

    m_list_collision_data_multiframe = opticalFlow.getCollisionPoints();
    writeToYaml(SENSOR_COUNT, opticalFlow);
}


void VectorRobustness::writeToYaml(ushort SENSOR_COUNT, const OpticalFlow &opticalFlow) {

    auto position = opticalFlow.getResultOrdner().find('/');
    std::string suffix = opticalFlow.getResultOrdner().replace(position, 1, "_");

    unsigned COUNT;
    if ( suffix == "_ground_truth") {
        COUNT = 1;
    }
    else {
        COUNT = DATAFILTER_COUNT;
    }

    for ( unsigned datafilter_index = 0; datafilter_index < COUNT; datafilter_index++ ) {

        for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

            std::cout << "generating vector robustness in RobustnessIndex.cpp for " << suffix << " " << sensor_index
                      << " for datafilter " << datafilter_index << std::endl;

            std::vector<cv::Point2f>  xsamples, ysamples;
            std::vector<cv::Point2f>  xsamples_dimension, ysamples_displacement;

            unsigned long FRAME_COUNT = m_list_collision_data_multiframe.at(datafilter_index).at(sensor_index).size();

            // Send to plotter
            if ( suffix == "_ground_truth") {
                m_fs << (std::string("collisionpoints") + suffix + std::string("_sensor_index_") + std::to_string(sensor_index)) << "[";
            }
            else {
                m_fs << (std::string("collisionpoints_") + suffix + std::string("datafilter_") + std::to_string(datafilter_index) + "_" + std::string("sensor_index_") + std::to_string(sensor_index)) << "[";
            }


            for (unsigned current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

                unsigned long TOTAL_OBJECTS = m_list_collision_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).size();

                for (unsigned objIndex = 0; objIndex < TOTAL_OBJECTS; objIndex++) {

                    m_fs << "{:" << "current_frame_index" <<
                         m_list_collision_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).first
                         << "collisionpoints" <<
                         (m_list_collision_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).second)

                         << "}";
                }
            }

            m_fs << "]";

        }
    }

}
