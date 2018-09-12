//
// Created by veikas on 30.06.18.
//

#include <opencv/cv.hpp>
#include <iostream>
#include <map>
#include "RobustnessIndex.h"
#include "Utils.h"
#include "OpticalFlow.h"


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

            unsigned long FRAME_COUNT = m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).size();

            // Send to plotter

            if ( suffix == "_ground_truth") {
                m_fs << (std::string("evaluation_data") + suffix + std::string("_sensor_index_") + std::to_string(sensor_index)) << "[";
            }
            else {
                m_fs << (std::string("evaluation_data_") + suffix + std::string("datafilter_") + std::to_string(datafilter_index) + "_" + std::string("sensor_index_") + std::to_string(sensor_index)) << "[";
            }

            for (unsigned current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

                m_fs <<  "{:" << "current_frame_index" <<                              m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(0).current_frame_index
                        << "}";
                m_fs << "[";

                unsigned long TOTAL_OBJECTS = m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).size();

                for (unsigned objIndex = 0; objIndex < TOTAL_OBJECTS; objIndex++) {

                    m_fs << "{"
                         << "obj_index" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).obj_index

                         << "visibility" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).visiblity

                         << "ground_truth_pixels_count" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).ground_truth_pixels_count

                         << "ground_truth_sroi_pixels_count" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).ground_truth_sroi_pixels_count

                         << "algorithm_pixels_count" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).algorithm_pixels_count

                         << "algorithm_interpolated_pixels_count" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).algorithm_interpolated_pixels_count

                         << "algorithm_sroi_pixels_count" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).algorithm_sroi_pixels_count

                         << "algorithm_sroi_interpolated_pixels_count" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).algorithm_sroi_interpolated_pixels_count

                         << "l1_total_count_good_pixels" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).l1_total_count_good_pixels
                         << "l2_total_count_good_pixels" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).l2_total_count_good_pixels
                         << "ma_total_count_good_pixels" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).ma_total_count_good_pixels

                         << "l1_cumulative_distance_all_pixels" <<
                         (m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).l1_cumulative_distance_all_pixels)
                         << "l2_cumulative_distance_all_pixels" <<
                         (m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).l2_cumulative_distance_all_pixels)
                         << "ma_cumulative_distance_all_pixels" <<
                         (m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).ma_cumulative_distance_all_pixels)

                         << "l1_cumulative_distance_good_pixels" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).l1_cumulative_distance_good_pixels
                         << "l2_cumulative_distance_good_pixels" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).l2_cumulative_distance_good_pixels
                         << "ma_cumulative_distance_good_pixels" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).ma_cumulative_distance_good_pixels

                         << "stddev_displacement" <<
                         (m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).stddev_displacement)

                         << "}";


                }
                m_fs << "]";
            }
            m_fs << "]";
        }
    }

}

