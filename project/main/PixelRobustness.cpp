//
// Created by veikas on 30.06.18.
//

#include <opencv/cv.hpp>
#include <iostream>
#include <map>
#include "RobustnessIndex.h"
#include "Utils.h"


void PixelRobustness::generatePixelRobustness(ushort SENSOR_COUNT, const OpticalFlow &opticalFlow_base, const OpticalFlow &opticalFlow) {

    auto position = opticalFlow.getResultOrdner().find('/');
    std::string suffix = opticalFlow.getResultOrdner().replace(position, 1, "_");

    unsigned COUNT;
    if ( suffix == "_ground_truth") {
        COUNT = 1;
    }
    else {
        COUNT = DATAFILTER_COUNT;
    }

    // shape of algorithhm, with shape of ground truth
    for ( unsigned datafilter_index = 0; datafilter_index < COUNT; datafilter_index++ ) {

        std::vector<std::vector<std::vector<double> > > sensor_evaluation_data_multiframe;

        for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

            std::vector<std::vector<double> > evaluation_data_multiframe;

            std::cout << "generating Mahalanobis distance in RobustnessIndex.cpp for " << suffix << " " << sensor_index
                      << " for datafilter " << datafilter_index << std::endl;

            std::vector<cv::Point2f> xsamples, ysamples;
            std::vector<cv::Point2f> xsamples_dimension, ysamples_displacement;

            unsigned long FRAME_COUNT = opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(
                    sensor_index).size();
            for (unsigned current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

                 std::vector<double> evaluation_data_frame;

                evaluation_data_multiframe.push_back(evaluation_data_frame);
            }
            sensor_evaluation_data_multiframe.push_back(evaluation_data_multiframe);
        }
        m_list_evaluation_data_multiframe.push_back(sensor_evaluation_data_multiframe);
    }
    writeToYaml(SENSOR_COUNT, opticalFlow);
}

