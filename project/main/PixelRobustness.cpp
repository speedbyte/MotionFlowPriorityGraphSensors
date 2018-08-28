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


    m_list_evaluation_data_multiframe = opticalFlow.get_sensor_multiframe_evaluation_data();
    writeToYaml(SENSOR_COUNT, opticalFlow);
}


void PixelRobustness::writeToYaml(ushort SENSOR_COUNT, const OpticalFlow &opticalFlow) {

    auto position = opticalFlow.getResultOrdner().find('/');
    std::string suffix = opticalFlow.getResultOrdner().replace(position, 1, "_");

    unsigned COUNT;
    if ( suffix == "_ground_truth") {
        COUNT = 1;
    }
    else {
        COUNT = (unsigned)m_list_evaluation_data_multiframe.size();
    }

    // shape of algorithhm, with shape of ground truth
    for ( unsigned datafilter_index = 0; datafilter_index < COUNT; datafilter_index++ ) {

        for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

            std::cout << "generating pixel robustness in RobustnessIndex.cpp for " << suffix << " " << sensor_index
                      << " for datafilter " << datafilter_index << std::endl;

            std::vector<cv::Point2f>  xsamples, ysamples;
            std::vector<cv::Point2f>  xsamples_dimension, ysamples_displacement;

            unsigned long FRAME_COUNT = m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).size();

            // Send to plotter

            if ( suffix == "_ground_truth") {
                m_fs << (std::string("evaluation_data") + suffix + std::string("_sensor_index_") + std::to_string(sensor_index)) << "[";
            }
            else {
                m_fs << (std::string("evaluation_data_") + suffix + std::string("datafilter_") + std::to_string(datafilter_index) + "_" + std::string("sensor_index_") + std::to_string(sensor_index)) << "[";
            }

            for (unsigned current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

                unsigned long TOTAL_OBJECTS = m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).size();

                for (unsigned objIndex = 0; objIndex < TOTAL_OBJECTS; objIndex++) {

                    std::pair<cv::Point2f, cv::Point2f> displacementPoints_base = std::make_pair(
                            m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).object_dimension, m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).mean_displacement);


                    m_fs << "{:"

                         << "current_frame_index" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).current_frame_index
                         << "obj_index" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).obj_index

                         << "good_pixels_l1_error" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).goodPixels_l1_error
                         << "good_pixels_l2_error" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).goodPixels_l2_error
                         << "good_pixels_ma_error" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).goodPixels_ma_error

                         << "l1_error_all" <<
                         (m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).allPixels_l1_error)
                         << "l2_error_all" <<
                         (m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).allPixels_l2_error)
                         << "ma_error_all" <<
                         (m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).allPixels_ma_error)

                         << "visible_pixels" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).visiblePixels
                         << "ground_truth_pixels" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).groundTruthPixels
                         << "stddev" <<
                         (m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).stddev_displacement)


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

