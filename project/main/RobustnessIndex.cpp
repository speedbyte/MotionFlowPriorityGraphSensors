//
// Created by veikas on 02.02.18.
//

#include <opencv/cv.hpp>
#include <iostream>
#include <map>
#include "RobustnessIndex.h"
#include "Utils.h"


void PixelRobustness::writeToYaml(ushort SENSOR_COUNT, const OpticalFlow &opticalFlow) {

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

        for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

            std::cout << "generating pixel robustness in RobustnessIndex.cpp for " << suffix << " " << sensor_index
                      << " for datafilter " << datafilter_index << std::endl;

            std::vector<cv::Point2f>  xsamples, ysamples;
            std::vector<cv::Point2f>  xsamples_dimension, ysamples_displacement;

            unsigned long FRAME_COUNT = opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).size();

            // Send to plotter

            if ( suffix == "_ground_truth") {
                m_fs << (std::string("evaluation_data") + suffix + std::string("_sensor_index_") + std::to_string(sensor_index)) << "[";
            }
            else {
                m_fs << (std::string("evaluation_data_") + suffix + std::string("datafilter_") + std::to_string(datafilter_index) + "_" + std::string("sensor_index_") + std::to_string(sensor_index)) << "[";
            }

            for (unsigned current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

                unsigned long TOTAL_OBJECTS = opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(current_frame_index).size();

                for (unsigned objIndex = 0; objIndex < TOTAL_OBJECTS; objIndex++) {

                    std::pair<cv::Point2f, cv::Point2f> displacementPoints_base = std::make_pair(
                            opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).object_dimension, opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).mean_displacement);


                    m_fs << "{:" << "current_frame_index" <<
                         opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).current_frame_index
                         << "obj_index" <<
                         opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).obj_index
                         << "good_pixels_l2" <<
                         opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).goodPixels_l2
                         << "good_pixels_maha" <<
                         opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).goodPixels_maha
                         << "visible_pixels" <<
                         opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).visiblePixels
                         << "ground_truth_pixels" <<
                         opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).object_dimension.x * opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).object_dimension.y
                         << "stddev" <<
                         (opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).stddev_displacement)
                         << "ma_distance" <<
                         (opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).mahalanobisDistance)
                         << "l1_distance" <<
                         (opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).l1)
                         << "l2_distance" <<
                         (opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).l2)

                         << "}";
                    //xsamples.push_back(shapepoints.first);
                    //ysamples.push_back(shapepoints.second);
                    xsamples_dimension.push_back(displacementPoints_base.first);
                    ysamples_displacement.push_back(displacementPoints_base.second);

                }

            }

            m_fs << "]";

            // Send to plotter
            if ( datafilter_index < 0 ) {

                if (suffix == "_ground_truth") {
                    m_fs << (std::string("obj_displacement") + suffix) + std::string("_sensor_index_") +
                            std::to_string(sensor_index) << "[";
                } else {
                    m_fs << (std::string("obj_displacement") +
                             std::string("_datafilter_") + std::to_string(datafilter_index) + suffix +
                             std::string("sensor_index_") + std::to_string(sensor_index)) << "[";
                }

                for (unsigned i = 0; i < xsamples_dimension.size(); i++) {
                    m_fs << "{:" << "objDim" << xsamples_dimension[i] << "objDisp" << ysamples_displacement[i] << "}";
                }
                m_fs << "]";


                std::map<std::pair<float, float>, int> scenario_displacement_occurence;

                scenario_displacement_occurence = opticalFlow.getScenarioDisplacementOccurence().at(sensor_index);

                m_fs << (std::string("scenario_displacement_occurence") + std::string("sensor_index_") +
                         std::to_string(sensor_index) +
                         std::string("_datafilter_") + std::to_string(datafilter_index) + suffix) << "[";

                for (auto it = scenario_displacement_occurence.begin();
                     it != scenario_displacement_occurence.end(); it++) {

                    if (it->second > 1) {
                        std::cout << cv::Point2f(it->first.first, it->first.second) << " " << it->second << std::endl;
                        m_fs << "{:" << "x" << it->first.first << "y" << it->first.second << "occurence" << it->second
                             << "}";
                    }
                }

                m_fs << "]";

            }
        }
    }

}


