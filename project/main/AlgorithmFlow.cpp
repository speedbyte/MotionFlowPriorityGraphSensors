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


void AlgorithmFlow::prepare_directories(ALGO_TYPES algo, std::string noise, ushort fps, ushort stepSize) {

    m_resultordner = "results_";

    switch ( algo ) {
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

    m_GroundTruthImageLocation = Dataset::getGroundTruthPath().string() + "/" + noise;

    m_resultordner += noise + "_" + std::to_string(fps) + "_" + std::to_string(stepSize) + "/";

    m_generatepath = Dataset::getResultPath().string() + "/" +  m_resultordner;

    if (!Dataset::getDatasetPath().compare(CPP_DATASET_PATH) || !Dataset::getDatasetPath().compare(VIRES_DATASET_PATH)) {

        std::cout << "Creating Flow directories " << m_resultordner << std::endl;

        prepare_directories_common();

        std::cout << "Ending Flow directories " << m_resultordner << std::endl;
    }
}

void AlgorithmFlow::run_optical_flow_algorithm(ALGO_TYPES algo, FRAME_TYPES frame_types, std::string noise, ushort fps ) {

    char sensor_index_folder_suffix[50];
    for ( ushort sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++ ) {

        unsigned FRAME_COUNT = (unsigned)m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(sensor_index).size();
        assert(FRAME_COUNT>0);
        cv::Mat image_02_frame = cv::Mat::zeros(Dataset::getFrameSize(), CV_32FC3);
        sprintf(sensor_index_folder_suffix, "%02d", sensor_index);
        std::cout << "saving algorithm flow files in flow/ for sensor_index  " << sensor_index << std::endl;



        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > object_stencil_movement(m_ptr_list_simulated_objects.size());
        std::vector<std::vector<std::vector<bool> >  > object_extrapolated_visibility(m_ptr_list_simulated_objects.size());

        bool needToInit = true;

        cv::Mat curGray, prevGray;
        cv::Size subPixWinSize(10, 10), winSize(21, 21);
        const int MAX_COUNT = 5000;

        ushort collision = 0, iterator = 0, sIterator = 0;
        std::vector<ushort> xPos, yPos;

        bool plotTime = 1;
        std::vector<bool> error(2);
        error.at(0) = 0;
        error.at(1) = 0;

        std::vector<cv::Point2f> next_pts_healthy;

        std::cout << "results will be stored in " << m_resultordner << std::endl;
        std::cout << "creating flow files for sensor_index " << sensor_index << std::endl;
        std::vector<cv::Point2f> prev_pts_array;

        cv::Mat flowFrame( Dataset::getFrameSize(), CV_32FC2 );
        flowFrame = cv::Scalar_<float>(0,0); //  the flow frame consists of next iterations


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
                    prev_pts_array.clear();
                    break;
                default:
                    break;
            }

            char file_name_input_image[50];
            std::cout << "frame_count " << frame_count << std::endl;
            sprintf(file_name_input_image, "000%03d_10.png", frame_count);
            std::string input_image_path = m_GroundTruthImageLocation.string() + "_" + std::to_string(sensor_index) + "/" + file_name_input_image;
            image_02_frame = cv::imread(input_image_path, CV_LOAD_IMAGE_COLOR);
            if ( image_02_frame.data == NULL ) {
                std::cerr << input_image_path << " not found" << std::endl;
                throw ("No image file found error");
            }

            std::string flow_path = m_flow_occ_path.string() + sensor_index_folder_suffix + "/" +
                                    file_name_input_image;
            std::string kitti_path = m_plots_path.string() + sensor_index_folder_suffix + "/" +
                                    file_name_input_image;
            std::string position_path = m_position_occ_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
            FlowImageExtended F_png_write(Dataset::getFrameSize().width, Dataset::getFrameSize().height);
            float max_magnitude = 0;

            std::vector<cv::Point2f> next_pts_array, displacement_array;


            // Convert to grayscale
            cv::cvtColor(image_02_frame, curGray, cv::COLOR_BGR2GRAY);

            // Calculate optical generate_flow_frame map using LK algorithm
            if (prevGray.data) {  // Calculate only on second or subsequent images.

                std::cout << "frame_count " << frame_count << std::endl;

                std::vector<uchar> status;
                // Initialize parameters for the optical generate_flow_frame algorithm
                float pyrScale = 0.5;
                int numLevels = 1;
                int windowSize = 5;
                int numIterations = 1;
                int neighborhoodSize = 2; // polyN
                float stdDeviation = 1.1; // polySigma

                std::vector<float> err;

                // Calculate optical generate_flow_frame map using Farneback algorithm
                // Farnback returns displacement frame and LK returns points.
                cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
                if ( lk == algo ) {
                    cv::calcOpticalFlowPyrLK(prevGray, curGray, prev_pts_array, next_pts_array, status,
                                             err, winSize, 5, termcrit, 0, 0.001);
                }
                else if ( fb == algo ) {
                    cv::calcOpticalFlowFarneback(prevGray, curGray, flowFrame, pyrScale, numLevels, windowSize,
                                                 numIterations, neighborhoodSize, stdDeviation,
                                                 cv::OPTFLOW_USE_INITIAL_FLOW);
                    // OPTFLOW_USE_INITIAL_FLOW didnt work and gave NaNs
                }

                // Draw the optical generate_flow_frame map
                if ( fb == algo ) {
                    prev_pts_array.clear();
                    next_pts_array.clear();
                    for (int32_t row = 0; row < Dataset::getFrameSize().height; row += mStepSize) { // rows
                        for (int32_t col = 0; col < Dataset::getFrameSize().width; col += 1) {  // cols

                            cv::Point2f algorithmMovement ( flowFrame.at<cv::Point2f>(row, col).x, flowFrame
                                    .at<cv::Point2f>(row, col).y );

                            if (( cvFloor(std::abs(algorithmMovement.x)) == 0 && cvFloor(std::abs(algorithmMovement
                                                                                                          .y)) == 0 )) {
                                continue;
                            }

                            next_pts_array.push_back(cv::Point2f(col, row));
                            prev_pts_array.push_back(cv::Point2f((col - algorithmMovement.x), (row - algorithmMovement.y)));

                            status.push_back(1);
                        }
                    }
                }

                unsigned count_good_points = 0;

                for (unsigned i = 0; i < next_pts_array.size(); i++) {

                    float minDist = 0.5;

                    // Check if the status vector is good
                    if (!status[i])
                        continue;

                    cv::Point2f next_pts, displacement;

                    displacement.x = next_pts_array[i].x - prev_pts_array[i].x;
                    displacement.y = next_pts_array[i].y - prev_pts_array[i].y;

                    /* If the new point is within 'minDist' distance from an existing point, it will not be tracked */
                    auto dist_ = cv::norm(displacement);
                    double dist;
                    dist = pow(displacement.x,2)+pow(displacement.y,2); //calculating distance by euclidean formula
                    dist = sqrt(dist);
                    assert(dist==dist_);

                    if ( dist <= minDist ) {
                        //printf("minimum distance for %i is %f\n", i, dist);
                        continue;
                    }

                    if ( displacement.x == 0 && displacement.y == 0) {
                        continue;
                    }

                    // next_pts is the new pixel position !
                    next_pts.x = next_pts_array[i].x;
                    next_pts.y = next_pts_array[i].y;

                    // std::cout << "next valid points " << next_pts << " displacement " << displacement << std::endl;
                    next_pts_array[count_good_points++] = next_pts_array[i];
                    displacement_array.push_back(displacement);
                }

                // flow frame and displacement in the form of displacement_array is created here.

                next_pts_array.resize(count_good_points); // this is required for LK. For FB, anyways the frame will
                assert(displacement_array.size() == count_good_points);
                // be completely calculated every time.

                if ( next_pts_array.size() == 0 ) {
                    // pick up the last healthy points
                    next_pts_array = next_pts_healthy;
                }

                for (unsigned i = 0; i < next_pts_array.size(); i++) {
                    //cv::circle(image_02_frame, next_pts_array[i], 1, cv::Scalar(0, 255, 0), 1, 8);
                    cv::arrowedLine(image_02_frame, prev_pts_array[i], next_pts_array[i], cv::Scalar(0,255,0), 1, 8, 0, 0.5);
                }


                for ( ushort obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++ ) {

                    float columnBegin = m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (sensor_index).at(frame_count).m_region_of_interest_px.x;
                    float rowBegin = m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (sensor_index).at(frame_count).m_region_of_interest_px.y;
                    int width = cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(frame_count).m_region_of_interest_px.width_px);
                    int height = cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(frame_count).m_region_of_interest_px.height_px);
                    bool visibility = m_ptr_list_simulated_objects.at(obj_index)->get_object_extrapolated_visibility().at(sensor_index).at(frame_count);
                    if ( visibility ) {
                        // gt_displacement
                        cv::Point2f gt_displacement = m_ptr_list_gt_objects.at(obj_index)->get_object_extrapolated_point_displacement().at(sensor_index).at(frame_count).second;

                        //cv::rectangle(image_02_frame, cv::Rect(columnBegin, rowBegin, width, height), cv::Scalar(0,0,255), 4, 8, 0 );

                        cv::Mat roi = flowFrame.
                                rowRange(cvRound(rowBegin-(DO_STENCIL_GRID_EXTENSION*STENCIL_GRID_EXTENDER)),
                                         (cvRound(rowBegin+height+(DO_STENCIL_GRID_EXTENSION*STENCIL_GRID_EXTENDER)))).
                                colRange(cvRound(columnBegin-(DO_STENCIL_GRID_EXTENSION*STENCIL_GRID_EXTENDER)),
                                         (cvRound(columnBegin+width+(DO_STENCIL_GRID_EXTENSION*STENCIL_GRID_EXTENDER))));

                        cv::Size roi_size;
                        cv::Point roi_offset;
                        roi.locateROI(roi_size, roi_offset);

                        // TODO scratch : This is for the base model

                        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> >  > stencil_movement(m_ptr_list_simulated_objects.size());
                        std::vector<std::vector<bool>  > base_visibility(m_ptr_list_simulated_objects.size());

                        for (unsigned row_index = 0; row_index < roi.rows; row_index++) {
                            for (unsigned col_index = 0; col_index < roi.cols; col_index++) {

                                if ( col_index%STENCIL_GRID_COMPRESSOR == 0 && row_index%STENCIL_GRID_COMPRESSOR == 0 ) { // only entertain multiple of col_index pixels to reduce data

                                    cv::Point2f algo_displacement = roi.at<cv::Vec2f>(row_index, col_index);
                                    auto dist_algo = cv::norm(algo_displacement);
                                    if ( dist_algo < 0.1 ) {
                                        continue;
                                    }

                                    for ( auto next_pts_index = 0; next_pts_index < next_pts_array.size(); next_pts_index++ ) {
                                        if ( (( roi_offset.x + col_index ) == next_pts_array.at(next_pts_index).x) &&
                                             (( roi_offset.y + row_index ) == next_pts_array.at(next_pts_index).y)) {

                                            stencil_movement.at(obj_index).push_back(
                                                    std::make_pair(cv::Point2f(roi_offset.x + col_index, roi_offset.y + row_index),
                                                                   algo_displacement));
                                            base_visibility.at(obj_index).push_back(visibility);

                                        }
                                    }
                                }
                            }
                        }

                        auto new_stencil_size = stencil_movement.at(obj_index).size();
                        std::cout << new_stencil_size << " " << next_pts_array.size() << std::endl;
                        assert(new_stencil_size != 0);

                        // TODO scratch : if stencil size does not work

                        object_stencil_movement.at(obj_index).push_back(stencil_movement.at(obj_index));
                        object_extrapolated_visibility.at(obj_index).push_back(base_visibility.at(obj_index));

                    }
                    else {

                        object_stencil_movement.at(obj_index).push_back({{std::make_pair(cv::Point2f(0, 0),cv::Point2f(0, 0))}});
                        object_extrapolated_visibility.at(obj_index).push_back({{false}});

                    }
                }


                std::vector<cv::Point2f>::iterator it, it2 ;

                for ( it = next_pts_array.begin(), it2 = displacement_array.begin(); it !=next_pts_array.end(); it++, it2++ )
                {

                    F_png_write.setFlowU((*it).x,(*it).y,(*it2).x);
                    F_png_write.setFlowV((*it).x,(*it).y,(*it2).y);
                    F_png_write.setValid((*it).x,(*it).y,true);
                }

                F_png_write.writeExtended(flow_path);
                F_png_write.writeColor(kitti_path, 5);
            }

            else {
                std::cout << "skipping first frame frame count " << frame_count << std::endl;
                // But still write the data for completion
                F_png_write.writeExtended(flow_path);
                F_png_write.writeColor(kitti_path, 5);

                for ( ushort obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++ ) {
                    object_extrapolated_visibility.at(obj_index).push_back({{false}});
                    object_stencil_movement.at(obj_index).push_back({{std::make_pair(cv::Point2f(0, 0),cv::Point2f(0, 0))}});
                }

                needToInit = true;
            }

            if ( needToInit && algo == lk) { //|| ( frame_count%4 == 0) ) {
                //|| next_pts_array.size() == 0) { // the init should be also when there is no next_pts_array.
                // automatic initialization
                cv::goodFeaturesToTrack(curGray, next_pts_array, MAX_COUNT, 0.01, 10, cv::Mat(), 3, false, 0.04);
                // Refining the location of the feature points
                assert(next_pts_array.size() <= MAX_COUNT );
                std::cout << next_pts_array.size() << std::endl;
                std::vector<cv::Point2f> currentPoint;
                std::swap(currentPoint, next_pts_array);
                next_pts_array.clear();
                for (unsigned i = 0; i < currentPoint.size(); i++) {
                    std::vector<cv::Point2f> tempPoints;
                    tempPoints.push_back(currentPoint[i]);
                    // Function to refine the location of the corners to subpixel accuracy.
                    // Here, 'pixel' refers to the image patch of size 'windowSize' and not the actual image pixel
                    cv::TermCriteria termcrit_subpixel(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
                    cv::cornerSubPix(curGray, tempPoints, subPixWinSize, cv::Size(-1, -1), termcrit_subpixel);
                    next_pts_array.push_back(tempPoints[0]);
                }
                printf("old next_pts_array size is %ld and new next_pts_array size is %ld\n", currentPoint.size(), next_pts_array.size());
            }

            needToInit = false;
            prev_pts_array = next_pts_array;
            next_pts_healthy = prev_pts_array;
            next_pts_array.clear();

            // Display the output image
            //cv::namedWindow(m_resultordner+"_" + std::to_string(frame_count), CV_WINDOW_AUTOSIZE);
            //cv::imshow(m_resultordner+"_"+std::to_string(frame_count), image_02_frame);
            //cv::waitKey(0);
            //cv::destroyAllWindows();
            cv::imwrite(position_path, image_02_frame);
            prevGray = curGray.clone();

        }

        for ( ushort obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++) {
            m_ptr_list_simulated_objects.at(obj_index)->set_object_stencil_point_displacement_pixel_visibility("alorithm", object_stencil_movement.at(obj_index), object_extrapolated_visibility.at(obj_index));
        }

        cv::destroyAllWindows();
    }
}


