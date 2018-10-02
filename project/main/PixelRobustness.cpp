//
// Created by veikas on 30.06.18.
//

#include <opencv/cv.hpp>
#include <iostream>
#include <map>
#include "RobustnessIndex.h"
#include "Utils.h"
#include "OpticalFlow.h"


void PixelRobustness::generatePixelRobustness(const OpticalFlow &opticalFlow_base, const OpticalFlow &opticalFlow, cv::FileStorage &m_fs) {

    auto position = opticalFlow.getResultOrdner().find('/');
    std::string suffix = opticalFlow.getResultOrdner().replace(position, 1, "_");


    m_list_evaluation_data_multiframe = opticalFlow.get_sensor_multiframe_evaluation_data();
    writeToYaml(opticalFlow, m_fs);
}


void PixelRobustness::writeToYaml(const OpticalFlow &opticalFlow, cv::FileStorage &m_fs) {

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

        for (unsigned sensor_index = 0; sensor_index < Dataset::SENSOR_COUNT; sensor_index++) {

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

                m_fs <<  "{:" << "frame_number" << m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).at(0).frame_number
                        << "}";
                m_fs << "[";

                std::cout << "current_frame_index " << current_frame_index << " for pixel robustness"
                        << std::endl;

                unsigned long TOTAL_OBJECTS = m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(current_frame_index).size();

                for (unsigned objIndex = 0; objIndex < TOTAL_OBJECTS; objIndex++) {

                    ushort count = 0;
                    const std::vector<std::string> algorithm_metrics_strings = {

                            "all_pixels",
                            "l1_good_pixels",
                            "l2_good_pixels",
                            "ma_good_pixels",

                            "l1_cumulative_error_all_pixels",
                            "l2_cumulative_error_all_pixels",
                            "ma_cumulative_error_all_pixels",

                            "l1_cumulative_error_good_pixels",
                            "l2_cumulative_error_good_pixels",
                            "ma_cumulative_error_good_pixels",
                    };

                    // ------------------------------
                    std::string suffix_algorithm_metrics = "eroi_";
                    ushort *algorithm_metrics_pointer_metrics_ushort = (ushort *) (&m_list_evaluation_data_multiframe.at(
                            datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).entire_metrics);
                    float *algorithm_metrics_pointer_metrics_float = (float *) (
                            algorithm_metrics_pointer_metrics_ushort + 4);
                    char *algorithm_metrics_pointer_metrics_sync_point = (char *) (
                            algorithm_metrics_pointer_metrics_float + 6);
                    // ------------------------------
                    std::string suffix_algorithm_interpolated_metrics = "eroi_interpolated_";
                    ushort *algorithm_interpolated_metrics_pointer_metrics_ushort = (ushort *) (&m_list_evaluation_data_multiframe.at(
                            datafilter_index).at(sensor_index).at(current_frame_index).at(
                            objIndex).entire_interpolated_metrics);
                    float *algorithm_interpolated_metrics_pointer_metrics_float = (float *) (
                            algorithm_interpolated_metrics_pointer_metrics_ushort + 4);
                    char *algorithm_interpolated_metrics_pointer_metrics_sync_point = (char *) (
                            algorithm_interpolated_metrics_pointer_metrics_float + 6);
                    // ------------------------------
                    std::string suffix_algorithm_sroi_metrics = "sroi_";
                    ushort *algorithm_sroi_metrics_pointer_metrics_ushort = (ushort *) (&m_list_evaluation_data_multiframe.at(
                            datafilter_index).at(sensor_index).at(current_frame_index).at(objIndex).sroi_metrics);
                    float *algorithm_sroi_metrics_pointer_metrics_float = (float *) (
                            algorithm_sroi_metrics_pointer_metrics_ushort + 4);
                    char *algorithm_sroi_metrics_pointer_metrics_sync_point = (char *) (
                            algorithm_sroi_metrics_pointer_metrics_float + 6);
                    // ------------------------------
                    std::string suffix_algorithm_sroi_interpolated_metrics = "sroi_interpolated_";
                    ushort *algorithm_sroi_interpolated_metrics_pointer_metrics_ushort = (ushort *) (&m_list_evaluation_data_multiframe.at(
                            datafilter_index).at(sensor_index).at(current_frame_index).at(
                            objIndex).sroi_interpolated_metrics);
                    float *algorithm_sroi_interpolated_metrics_pointer_metrics_float = (float *) (
                            algorithm_sroi_interpolated_metrics_pointer_metrics_ushort + 4);
                    char *algorithm_sroi_interpolated_metrics_pointer_metrics_sync_point = (char *) (
                            algorithm_sroi_interpolated_metrics_pointer_metrics_float + 6);

                    m_fs << "{"

                         // ------------------------------

                         << "obj_index" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(
                                 current_frame_index).at(objIndex).obj_index

                         << "visibility" <<
                         m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(
                                 current_frame_index).at(objIndex).visiblity

                         << "stddev_displacement" <<
                         (m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(
                                 current_frame_index).at(objIndex).stddev_displacement)

                         << "intersection_angle_deg" << (m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(
                            current_frame_index).at(objIndex).intersection_angle_deg);

                         cv::Mat_<float> ellipse = m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(
                            current_frame_index).at(objIndex).ellipse;

                         if ( ellipse.data != NULL ) {
                             cv::Vec3f flatten_ellipse = { ellipse(0), ellipse(1), ellipse(2) };
                             m_fs << "ellipse" << "[:";
                             for ( ushort element = 0; element < flatten_ellipse.rows; element++) {
                                 m_fs << flatten_ellipse(element);
                             }
                             m_fs << "]";
                         } else {
                             cv::Vec3f flatten_ellipse = { 0, 0, 0 };
                             m_fs << "ellipse" << "[:";
                             for ( ushort element = 0; element < flatten_ellipse.rows; element++) {
                                 m_fs << flatten_ellipse(element);
                             }
                             m_fs << "]";
                         }

                         cv::Mat_<float> covar = (m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(
                                         current_frame_index).at(objIndex).covar_displacement);

                         if ( covar.data != NULL ) {
                             cv::Vec4f flatten_covar = { covar(0,0), covar(0,1), covar(1,0), covar(1,1) };
                             m_fs << "covar_displacement" << "[:";
                             for ( ushort row = 0; row < flatten_covar.rows; row++) {
                                 m_fs << flatten_covar(row);
                             }
                         } else {
                             cv::Vec4f flatten_covar = { 0, 0, 0, 0 };
                             m_fs << "covar_displacement" << "[:";
                             for ( ushort row = 0; row < flatten_covar.rows; row++) {
                                 m_fs << flatten_covar(row);
                             }
                         }
                         m_fs << "]";
                         m_fs << "correlation" <<
                                m_list_evaluation_data_multiframe.at(datafilter_index).at(sensor_index).at(
                                        current_frame_index).at(objIndex).correlation;
                   // ------------------------------
                    // the ushort part
                    count = 0;
                    for (auto x = 0; x < 4; x++) {
                        m_fs << suffix_algorithm_metrics + algorithm_metrics_strings.at(x)
                             << *(algorithm_metrics_pointer_metrics_ushort + count);
                        count++;
                    }

                    // the float part
                    count = 0;
                    for (auto x = 4; x < 10; x++) {
                        m_fs << suffix_algorithm_metrics + algorithm_metrics_strings.at(x)
                             << *(algorithm_metrics_pointer_metrics_float + count);
                        count++;
                    }
                    assert(*(algorithm_metrics_pointer_metrics_sync_point + 0) == '$');
                    // ------------------------------
                    // the ushort part
                    count = 0;
                    for (auto x = 0; x < 4; x++) {
                        m_fs << suffix_algorithm_interpolated_metrics + algorithm_metrics_strings.at(x)
                             << *(algorithm_interpolated_metrics_pointer_metrics_ushort + count);
                        count++;
                    }

                    // the float part
                    count = 0;
                    for (auto x = 4; x < 10; x++) {
                        m_fs << suffix_algorithm_interpolated_metrics + algorithm_metrics_strings.at(x)
                             << *(algorithm_interpolated_metrics_pointer_metrics_float + count);
                        count++;
                    }
                    assert(*(algorithm_interpolated_metrics_pointer_metrics_sync_point + 0) == '$');
                    // ------------------------------
                    // the ushort part
                    count = 0;
                    for (auto x = 0; x < 4; x++) {
                        m_fs << suffix_algorithm_sroi_metrics + algorithm_metrics_strings.at(x)
                             << *(algorithm_sroi_metrics_pointer_metrics_ushort + count);
                        count++;
                    }
                    // the float part
                    count = 0;
                    for (auto x = 4; x < 10; x++) {
                        m_fs << suffix_algorithm_sroi_metrics + algorithm_metrics_strings.at(x)
                             << *(algorithm_sroi_metrics_pointer_metrics_float + count);
                        count++;
                    }
                    assert(*(algorithm_sroi_metrics_pointer_metrics_sync_point + 0) == '$');
                    // ------------------------------
                    // the ushort part
                    count = 0;
                    for (auto x = 0; x < 4; x++) {
                        m_fs << suffix_algorithm_sroi_interpolated_metrics + algorithm_metrics_strings.at(x)
                             << *(algorithm_sroi_interpolated_metrics_pointer_metrics_ushort + count);
                        count++;
                    }
                    // the float part
                    count = 0;
                    for (auto x = 4; x < 10; x++) {
                        m_fs << suffix_algorithm_sroi_interpolated_metrics + algorithm_metrics_strings.at(x)
                             << *(algorithm_sroi_interpolated_metrics_pointer_metrics_float + count);
                        count++;
                    }
                    assert(*(algorithm_sroi_interpolated_metrics_pointer_metrics_sync_point + 0) == '$');


                    cv::Mat distribution_matrix_yaml = m_list_evaluation_data_multiframe.at(datafilter_index).at(
                            sensor_index).at(
                            current_frame_index).at(objIndex).distribution_matrix.clone();

                    if (suffix == "_ground_truth"){
                        distribution_matrix_yaml.setTo(1);
                    }
                    //m_fs << "[";
                    m_fs << "distribution_matrix" << "[:";
                    for ( ushort row = 0; row < distribution_matrix_yaml.rows; row++) {
                        m_fs << "[:";
                        for ( ushort col = 0; col < distribution_matrix_yaml.cols; col++) {
                                m_fs <<
                                        (float)distribution_matrix_yaml.at<float>(row, col);
                            }
                        m_fs << "]";
                    }
                    m_fs << "]";

                    m_fs
                         << "}";

                }
                m_fs << "]";
            }
            m_fs << "]";
        }
    }

}

