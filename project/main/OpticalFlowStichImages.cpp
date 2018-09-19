


#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include "OpticalFlow.h"

void OpticalFlow::stich_gnuplots() {

    std::vector<Objects *> list_of_current_objects;
    std::vector<Objects *> ptr_list_of_derived_objects;

    for (auto i = 0; i < m_ptr_list_gt_objects.size(); i++) {
        ptr_list_of_derived_objects.push_back(static_cast<Objects *>(m_ptr_list_gt_objects.at(i)));
    }


    unsigned COUNT;
    if (m_opticalFlowName == "ground_truth") {
        COUNT = 1;
        list_of_current_objects = ptr_list_of_derived_objects;
    } else {
        list_of_current_objects = m_ptr_list_simulated_objects;
        COUNT = (unsigned) list_of_current_objects.at(0)->
                get_list_object_dataprocessing_mean_centroid_displacement().size();
    }

    for (unsigned datafilter_index = 0; datafilter_index < COUNT; datafilter_index++) {

        for (unsigned sensor_index = 0; sensor_index < Dataset::SENSOR_COUNT; sensor_index++) {

            std::cout << "stiching gnuplots in OpticalFlow.cpp for sensor index " << sensor_index
                      << " for opticalflow  " << m_opticalFlowName << std::endl;

            unsigned FRAME_COUNT = (unsigned) Dataset::MAX_ITERATION_RESULTS;

            assert(FRAME_COUNT > 0);

            std::vector<std::vector<OPTICAL_FLOW_EVALUATION_METRICS> > multiframe_evaluation_data;

            for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

                cv::Mat stich_plots;

                stich_plots.create(800, 800, CV_8UC3); // make space for 4 objects
                stich_plots = cv::Scalar::all(0);

                char file_name_image_output_stiched[50], stiched_sensor_index_folder_suffix[10];
                sprintf(stiched_sensor_index_folder_suffix, "%02d", (Dataset::SENSOR_COUNT - 1));
                std::string gnuplot_image_file_with_path_stiched;
                sprintf(file_name_image_output_stiched, "stiched_000%03d_10.png",
                        m_sensor_multiframe_evaluation_data.at(datafilter_index).at(sensor_index).at(
                                current_frame_index).at(0).frame_number);
                gnuplot_image_file_with_path_stiched =
                        m_gnuplots_path.string() + stiched_sensor_index_folder_suffix + "/" +
                        file_name_image_output_stiched;

                std::string gnuplotname_prefix;
                for (ushort x = 0; x < 4; x++) {

                    char file_name_image_input[50], sensor_index_folder_suffix[10];
                    std::string gnuplot_image_file_with_path;
                    sprintf(sensor_index_folder_suffix, "%02d", sensor_index);

                    //stich_plots = cv::imread(gnuplot_image_file_with_path_stiched, CV_LOAD_IMAGE_ANYCOLOR);

                    cv::Mat roi_stich;
                    switch (x) {
                        case 0:
                            gnuplotname_prefix = "entire";
                            roi_stich = stich_plots.rowRange(0, 400).colRange(0, 400);
                            break;
                        case 1:
                            gnuplotname_prefix = "entire_interpolated";
                            roi_stich = stich_plots.rowRange(400, 800).colRange(0, 400);
                            break;
                        case 2:
                            gnuplotname_prefix = "special";
                            roi_stich = stich_plots.rowRange(0, 400).colRange(400, 800);
                            break;
                        case 3:
                            gnuplotname_prefix = "special_interpolated";
                            roi_stich = stich_plots.rowRange(400, 800).colRange(400, 800);
                            break;
                    }

                    sprintf(file_name_image_input, "%s_000%03d_10.png", gnuplotname_prefix.c_str(),
                            m_sensor_multiframe_evaluation_data.at(datafilter_index).at(sensor_index).at(
                                    current_frame_index).at(0).frame_number);
                    gnuplot_image_file_with_path =
                            m_gnuplots_path.string() + sensor_index_folder_suffix + "/" + file_name_image_input;

                    cv::Mat gnuplot_index = cv::imread(gnuplot_image_file_with_path, CV_LOAD_IMAGE_ANYCOLOR);
                    if (gnuplot_index.empty()) {
                        throw "no image is found error";
                    }
                    //cv::imshow("plot", gnuplot_index);
                    //cv::waitKey(0);

                    gnuplot_index.copyTo(roi_stich);
                    boost::filesystem::remove(gnuplot_image_file_with_path);
                }

                cv::imwrite(gnuplot_image_file_with_path_stiched, stich_plots);

            }
        }
    }
}
