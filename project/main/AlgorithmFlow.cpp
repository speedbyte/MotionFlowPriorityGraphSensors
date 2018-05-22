//
// Created by veikas on 26.01.18.
//

#include <iostream>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <opencv2/core/types.hpp>
#include <boost/tuple/tuple.hpp>
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

void AlgorithmFlow::run_optical_flow_algorithm(FRAME_TYPES frame_types, std::string noise, ushort fps ) {

    char sensor_index_folder_suffix[50];

    for ( ushort sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++ ) {

        unsigned FRAME_COUNT = (unsigned)m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(sensor_index).size();
        assert(FRAME_COUNT>0);
        cv::Mat image_02_frame = cv::Mat::zeros(Dataset::getFrameSize(), CV_32FC3);
        sprintf(sensor_index_folder_suffix, "%02d", sensor_index);
        std::cout << "saving algorithm flow files in flow/ for sensor_index  " << sensor_index << std::endl;

        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > multiframe_stencil_displacement(m_ptr_list_simulated_objects.size());
        std::vector<std::vector<std::vector<bool> >  > multiframe_visibility(m_ptr_list_simulated_objects.size());

        bool needToInit = true;

        cv::Mat curGray, prevGray;

        std::vector<cv::Point2f> next_pts_healthy;

        std::cout << "results will be stored in " << m_resultordner << std::endl;
        std::cout << "creating flow files for sensor_index " << sensor_index << std::endl;
        std::vector<cv::Point2f> frame_prev_pts;

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
                    frame_prev_pts.clear();
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

            std::vector<cv::Point2f> frame_next_pts, displacement_array;

            // Convert to grayscale
            cv::cvtColor(image_02_frame, curGray, cv::COLOR_BGR2GRAY);

            // Calculate optical generate_flow_frame map using LK algorithm
            if (prevGray.data) {  // Calculate only on second or subsequent images.

                std::cout << "frame_count " << frame_count << std::endl;

                execute(prevGray, curGray, frame_prev_pts, frame_next_pts, needToInit);

                unsigned count_good_points = 0;
                for (unsigned i = 0; i < frame_next_pts.size(); i++) {

                    float minDist = 0.5;
                    // Check if the status vector is good
                    /*
                    if (!status[i])
                        continue;
                    */
                    cv::Point2f displacement;

                    displacement.x = frame_next_pts[i].x - frame_prev_pts[i].x;
                    displacement.y = frame_next_pts[i].y - frame_prev_pts[i].y;

                    /* If the new point is within 'minDist' distance from an existing point, it will not be tracked */
                    auto dist = cv::norm(displacement);
                    if ( dist <= minDist ) {
                        continue;
                    }

                    if ( displacement.x == 0 && displacement.y == 0) {
                        continue;
                    }

                    frame_next_pts[count_good_points++] = frame_next_pts[i];
                    displacement_array.push_back(displacement);
                }

                // flow frame and displacement in the form of displacement_array is created here.

                frame_next_pts.resize(count_good_points);
                assert(displacement_array.size() == count_good_points);

                if ( frame_next_pts.size() == 0 ) {
                    // pick up the last healthy points
                    frame_next_pts = next_pts_healthy;
                }

                for (unsigned i = 0; i < frame_next_pts.size(); i++) {
                    //cv::circle(image_02_frame, frame_next_pts[i], 1, cv::Scalar(0, 255, 0), 1, 8);
                    cv::arrowedLine(image_02_frame, frame_prev_pts[i], frame_next_pts[i], cv::Scalar(0,255,0), 1, 8, 0, 0.5);
                }

                common_flow_frame(sensor_index, frame_count, frame_next_pts, displacement_array, multiframe_stencil_displacement, multiframe_visibility);

            }

            else {
                
                std::cout << "skipping first frame frame count " << frame_count << std::endl;

                for ( ushort obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++ ) {
                    multiframe_stencil_displacement.at(obj_index).push_back({{std::make_pair(cv::Point2f(0, 0),cv::Point2f(0, 0))}});
                    multiframe_visibility.at(obj_index).push_back({{false}});
                }

                needToInit = true;
                execute(prevGray, curGray, frame_prev_pts, frame_next_pts, needToInit);

            }



            frame_prev_pts = frame_next_pts;
            next_pts_healthy = frame_prev_pts;
            frame_next_pts.clear();

            // Display the output image
            cv::namedWindow(m_resultordner+"_" + std::to_string(frame_count), CV_WINDOW_AUTOSIZE);
            cv::imshow(m_resultordner+"_"+std::to_string(frame_count), image_02_frame);
            cv::waitKey(0);
            cv::destroyAllWindows();
            cv::imwrite(position_path, image_02_frame);
            prevGray = curGray.clone();

        }

        for ( ushort obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++) {
            m_ptr_list_simulated_objects.at(obj_index)->set_object_stencil_point_displacement_pixel_visibility(multiframe_stencil_displacement.at(obj_index), multiframe_visibility.at(obj_index));
        }

        cv::destroyAllWindows();
    }

}


