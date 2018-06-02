//
// Created by veikas on 26.01.18.
//

#include <iostream>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/highgui.h>
#include <chrono>
#include <opencv/cv.hpp>
#include <map>
#include <gnuplot-iostream/gnuplot-iostream.h>

#include "AlgorithmFlow.h"


using namespace std::chrono;


void AlgorithmFlow::prepare_directories(std::string noise, ushort fps, ushort stepSize) {

    m_GroundTruthImageLocation = Dataset::getGroundTruthPath().string() + "/" + noise;

    m_resultordner = "results_";

    switch ( mAlgo ) {
        case lk: {
            m_resultordner += "LK_";
            break;
        }
        case fb: {
            m_resultordner += "FB_";
            break;
        }
        default: {
            throw("algorithm not yet supported");
        }
    }

    m_resultordner += noise + "_" + std::to_string(fps) + "_" + std::to_string(stepSize) + "/";

    m_generatepath = Dataset::getResultPath().string() + "/" +  m_resultordner;

    if (!Dataset::getDatasetPath().compare(CPP_DATASET_PATH) || !Dataset::getDatasetPath().compare(VIRES_DATASET_PATH)) {

        prepare_directories_common();

    }
}

void AlgorithmFlow::run_optical_flow_algorithm(FRAME_TYPES frame_types, ushort fps ) {

    char sensor_index_folder_suffix[50];

    for ( ushort sensor_index = 0; sensor_index < MAX_ALLOWED_SENSOR_GROUPS; sensor_index++ ) {

        unsigned FRAME_COUNT = (unsigned)m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(sensor_index).size();
        assert(FRAME_COUNT>0);
        cv::Mat image_02_frame = cv::Mat::zeros(Dataset::getFrameSize(), CV_32FC3);
        sprintf(sensor_index_folder_suffix, "%02d", sensor_index);
        std::cout << "saving algorithm flow files in flow/ for sensor_index  " << sensor_index << std::endl;

        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > multiframe_stencil_displacement(m_ptr_list_simulated_objects.size());
        std::vector<std::vector<std::vector<bool> >  > multiframe_visibility(m_ptr_list_simulated_objects.size());

        bool needToInit = true;

        cv::Mat curGray, prevGray;

        std::vector<cv::Point2f> frame_prev_pts_array;

        std::cout << "results will be stored in " << m_resultordner << std::endl;
        std::cout << "creating flow files for sensor_index " << sensor_index << std::endl;

        for (ushort frame_count=0; frame_count < MAX_ITERATION_RESULTS; frame_count++) {

            // Break out of the loop if the user presses the Esc key
            char c = (char) cv::waitKey(10);
            switch (c) {
                case 27:
                    break;
                case 'r':
                    needToInit = true;
                    break;
                case 'c':
                    frame_prev_pts_array.clear();
                    break;
                default:
                    break;
            }

            char file_name_input_image[50];
            sprintf(file_name_input_image, "000%03d_10.png", frame_count);
            std::string input_image_path = m_GroundTruthImageLocation.string() + "_" + std::to_string(sensor_index) + "/" + file_name_input_image;
            image_02_frame = cv::imread(input_image_path, CV_LOAD_IMAGE_COLOR);
            if ( image_02_frame.data == NULL ) {
                std::cerr << input_image_path << " not found" << std::endl;
                throw ("No image file found error");
            }

            std::string position_path = m_position_occ_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;

            std::vector<cv::Point2f> frame_next_pts_array, displacement_array;

            // Convert to grayscale
            cv::cvtColor(image_02_frame, curGray, cv::COLOR_BGR2GRAY);

            // Calculate optical generate_flow_frame map using LK algorithm
            if (prevGray.data) {  // Calculate only on second or subsequent images.

                std::cout << "frame_count " << frame_count << std::endl;

                needToInit = false;
                execute(prevGray, curGray, frame_prev_pts_array, frame_next_pts_array, displacement_array, needToInit);

                for (unsigned i = 0; i < frame_next_pts_array.size(); i++) {
                    //cv::circle(image_02_frame, frame_next_pts_array[i], 1, cv::Scalar(0, 255, 0), 1, 8);
                    cv::arrowedLine(image_02_frame, frame_prev_pts_array[i], frame_next_pts_array[i], cv::Scalar(0,255,0), 1, 8, 0, 0.5);
                }

                common_flow_frame(sensor_index, frame_count, frame_next_pts_array, displacement_array, multiframe_stencil_displacement, multiframe_visibility);

            }

            else {

                std::cout << "skipping first frame frame count " << frame_count << std::endl;

                for ( ushort obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++ ) {
                    multiframe_stencil_displacement.at(obj_index).push_back({{std::make_pair(cv::Point2f(0, 0),cv::Point2f(0, 0))}});
                    multiframe_visibility.at(obj_index).push_back({{false}});
                }

                needToInit = true;
                execute(prevGray, curGray, frame_prev_pts_array, frame_next_pts_array, displacement_array, needToInit);

            }


            // Display the output image
            //cv::namedWindow(m_resultordner+"_" + std::to_string(frame_count), CV_WINDOW_AUTOSIZE);
            //cv::imshow(m_resultordner+"_"+std::to_string(frame_count), image_02_frame);
            //cv::waitKey(0);
            cv::destroyAllWindows();
            cv::imwrite(position_path, image_02_frame);
            prevGray = curGray.clone();

        }

        for ( ushort obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++) {
            m_ptr_list_simulated_objects.at(obj_index)->push_back_object_stencil_point_displacement_pixel_visibility(multiframe_stencil_displacement.at(obj_index), multiframe_visibility.at(obj_index));
        }

        cv::destroyAllWindows();
    }

    std::vector<std::vector<bool> > combined_sensor_stencil_visibility(3);

    for ( ushort obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++) {


        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >  combined_stencil_sensor;
        std::vector<std::vector<bool> > combined_sensor_stencil_visibility;

        const std::vector<std::pair<cv::Point2f, cv::Point2f> > elements_combined_stencil_sensor;

        const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > stencil_sensor_1 =   m_ptr_list_simulated_objects.at(obj_index)->get_object_stencil_point_displacement();

        std::copy(stencil_sensor_1.at(0).begin(), stencil_sensor_1.at(0).end(), std::back_inserter(combined_stencil_sensor));

        for (ushort frame_count=0; frame_count < MAX_ITERATION_RESULTS; frame_count++) {

            std::copy(stencil_sensor_1.at(1).at(frame_count).begin(), stencil_sensor_1.at(1).at(frame_count).end(), std::back_inserter(combined_stencil_sensor.at(frame_count)));

        }


        m_ptr_list_simulated_objects.at(obj_index)->push_back_object_stencil_point_displacement_pixel_visibility(combined_stencil_sensor, combined_sensor_stencil_visibility);

    }


}
