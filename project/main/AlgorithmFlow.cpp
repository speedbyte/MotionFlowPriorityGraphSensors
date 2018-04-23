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


/**
 * This function means that the directories would be deleted and then created.
 * Hence only those datasets that has been synthetically produced on the test bench by us, should be deleted.
 * The results directory can be generally deleted, because they are all created by us.
 *
 * dataset/results/flow_occ_<algorithm>_<source>_<click_speed>_<noise_type>
 *
 * @param dataset_path
 * @param result_sha
 * @return
 */


void AlgorithmFlow::prepare_directories(ALGO_TYPES algo, FRAME_TYPES frame_types, std::string noise) {

    mImageabholOrt = Dataset::getGroundTruthPath().string() + "/" + noise + "/";
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

    m_resultordner += noise + "/";

    m_generatepath = Dataset::getResultPath().string() + "/" +  m_resultordner;

    if (!Dataset::getDatasetPath().compare(CPP_DATASET_PATH) || !Dataset::getDatasetPath().compare(VIRES_DATASET_PATH)) {

        std::cout << "Creating Algorithm Flow directories" << std::endl;

        OpticalFlow::prepare_directories();

        std::cout << "Ending Algorithm Flow directories" << std::endl;
    }
}

void AlgorithmFlow::generate_flow_frame(ALGO_TYPES algo, FRAME_TYPES frame_types, std::string noise ) {

    prepare_directories(algo, frame_types, noise);

    for ( int frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++ ) {

        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > outer_base_movement(m_list_simulated_objects.size());
        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > outer_stencil_movement(m_list_simulated_objects.size());
        std::vector<std::vector<std::vector<bool> >  > outer_base_visiblity(m_list_simulated_objects.size());

        char frame_skip_folder_suffix[50];
        char file_name_input_image[50], file_name_input_image_edge[50];

        std::vector<unsigned> x_pts;
        std::vector<double> y_pts;
        std::vector<unsigned> z_pts;
        std::vector<float> time;
        double sum_time = 0;

        std::vector<boost::tuple<std::vector<unsigned>, std::vector<double>> > pts_exectime;

        bool needToInit = true;

        std::cout << "results will be stored in " << m_resultordner << std::endl;

        /*
        if ( frame_types == video_frames) {
            cv::VideoCapture cap;
            cap.open(Dataset::getGroundTruthPath().string() + "image_02/movement.avi");
            if (!cap.isOpened()) {
                std::cout << "Could not initialize capturing...\n";
                return;
            }
        }*/
        cv::Mat curGray, prevGray;
        sprintf(frame_skip_folder_suffix, "%02d", frame_skip);
        std::string results_flow_matrix_str = m_flow_occ_path.string() + "/" +
                                              frame_skip_folder_suffix + "/" + "result_flow.yaml";
        cv::VideoWriter video_out;

        if ( frame_types == video_frames)
        {
            boost::filesystem::path video_out_path = m_flow_occ_path.string() + frame_skip_folder_suffix + "/" + "movement.avi" ;
            assert(boost::filesystem::exists(video_out_path.parent_path()) != 0);
            //frame_size.height =	(unsigned) cap.get(CV_CAP_PROP_FRAME_HEIGHT );
            //frame_size.width =	(unsigned) cap.get(CV_CAP_PROP_FRAME_WIDTH );
            video_out.open(video_out_path.string(),CV_FOURCC('D','I','V','X'), 5, Dataset::getFrameSize());
            printf("Writer eingerichtet\n");
        }

        cv::Mat image_02_frame = cv::Mat::zeros(Dataset::getFrameSize(), CV_32FC3);

        cv::Size subPixWinSize(10, 10), winSize(21, 21);

        const int MAX_COUNT = 5000;

        auto tic = steady_clock::now();
        auto toc = steady_clock::now();

        ushort collision = 0, iterator = 0, sIterator = 0;
        std::vector<ushort> xPos, yPos;

        std::map<std::string, double> time_map = {{"generate",0},
                                                  {"ground truth", 0},
                                                  {"FB", 0},
                                                  {"LK", 0},
                                                  {"movement", 0},
                                                  {"collision", 0},
        };

        std::map<ALGO_TYPES, std::string> algo_map = {{fb, "FB"},
                                                      {lk, "LK"},
        };

        bool plotTime = 1;
        std::vector<bool> error(2);
        error.at(0) = 0;
        error.at(1) = 0;

        std::string temp_result_flow_path, temp_result_position_path, temp_result_edge_path;
        std::vector<cv::Point2f> next_pts_healthy;

        std::cout << "creating flow files for frame_skip " << frame_skip << std::endl;
        std::vector<cv::Point2f> prev_pts_array;

        for (ushort frame_count=0; frame_count < MAX_ITERATION_RESULTS; frame_count++) {
            //draw new ground truth flow.

            if ( frame_count*frame_skip >= MAX_ITERATION_RESULTS) {
                break;
            }

            /*
            if ( frame_count%frame_skip != 0 ) {
                continue;
            }*/
            sprintf(file_name_input_image, "000%03d_10.png", frame_count*frame_skip);
            sprintf(file_name_input_image_edge, "000%03d_10_edge.png", frame_count*frame_skip);
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

            //cap >> image_02_frame;
            //if (image_02_frame.empty())
            //    break;

            std::string input_image_file_with_path = mImageabholOrt.string() + "/" +
                    file_name_input_image;

            image_02_frame = cv::imread(input_image_file_with_path, CV_LOAD_IMAGE_COLOR);

            if ( image_02_frame.data == NULL ) {
                std::cerr << input_image_file_with_path << " not found" << std::endl;
                throw ("No image file found error");
            }

            temp_result_flow_path = m_flow_occ_path.string() + frame_skip_folder_suffix + "/" + file_name_input_image;
            temp_result_edge_path = m_flow_occ_path.string() + frame_skip_folder_suffix + "/" + file_name_input_image_edge;
            temp_result_position_path = m_position_occ_path.string() + frame_skip_folder_suffix + "/" + file_name_input_image;

            // Convert to grayscale
            cv::cvtColor(image_02_frame, curGray, cv::COLOR_BGR2GRAY);

            cv::Mat flowFrame( Dataset::getFrameSize(), CV_32FC2 );
            flowFrame = cv::Scalar_<float>(0,0);

            std::vector<cv::Point2f> next_pts_array, displacement_array;
            tic = steady_clock::now();

            //Create png Matrix with 3 channels: x displacement. y displacment and Validation bit
            //kitti uses col, row specification
            FlowImageExtended F_png_write(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

            // Calculate optical generate_flow_frame map using LK algorithm
            if (prevGray.data) {  // Calculate only on second or subsequent images.

                std::cout << "frame_count " << frame_count << std::endl;

                std::vector<uchar> status;
                // Initialize parameters for the optical generate_flow_frame algorithm
                float pyrScale = 0.5;
                int numLevels = 3;
                int windowSize = 5;
                int numIterations = 3;
                int neighborhoodSize = 7; // polyN
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
                                                 cv::OPTFLOW_FARNEBACK_GAUSSIAN);
                    // OPTFLOW_USE_INITIAL_FLOW didnt work and gave NaNs
                }

                // Draw the optical generate_flow_frame map
                int stepSize = STEP_SIZE;

                if ( fb == algo ) {
                    // Circles to indicate the uniform grid of points
                    //cv::circle(image_02_frame, cv::Point(x, y), 1, cv::Scalar(0, 0, 0), -1, 8);
                    prev_pts_array.clear();
                    next_pts_array.clear();
                    for (int32_t row = 0; row < Dataset::getFrameSize().height; row += stepSize) { // rows
                        for (int32_t col = 0; col < Dataset::getFrameSize().width; col += stepSize) {  // cols

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

                next_pts_array.resize(count_good_points); // this is required for LK. For FB, anyways the frame will
                assert(displacement_array.size() == count_good_points);
                // be completely calculated every time.

                if ( next_pts_array.size() == 0 ) {
                    // pick up the last healthy points
                    next_pts_array = next_pts_healthy;
                }

                for (unsigned i = 0; i < next_pts_array.size(); i++) {
                    cv::arrowedLine(image_02_frame, prev_pts_array[i], next_pts_array[i], cv::Scalar(0, 255, 0));
                }

                std::vector<cv::Point2f>::iterator it, it2 ;

                for ( it = next_pts_array.begin(), it2 = displacement_array.begin(); it !=next_pts_array.end(); it++, it2++ )
                {

                    //F_png_write.setFlowU((*it).x,(*it).y,65535);
                    //F_png_write.setFlowV((*it).x,(*it).y,65535);
                    //F_png_write.setValid((*it).x,(*it).y,1.0f);

                    F_png_write.setFlowU((*it).x,(*it).y,(*it2).x);
                    F_png_write.setFlowV((*it).x,(*it).y,(*it2).y);
                    F_png_write.setValid((*it).x,(*it).y,true);
                    // TODO - store objectId instead of 1.0. The flow vectors are automatically converted to 2's complement in the writeFlowField
                }

                cv::Mat stencilFrame(Dataset::getFrameSize(), CV_32FC3, cv::Scalar(0,0,0));
                stencilFrame = flowFrame.clone();

                /*
                cv::MatIterator_<cv::Vec3f> it_flowframe = stencilFrame.begin<cv::Vec3f>();
                for (unsigned i = 0; it_flowframe != stencilFrame.end<cv::Vec3f>(); it_flowframe++ ) {
                    for ( unsigned j = 0; j < 3; j++ ) {
                        (*it_flowframe)[j]= *(F_png_write.data_ + i );
                        // std::min(temp*64.0f+32768.0f,65535.0f),0.0f)
                        i++;
                    }
                }*/


                for ( ushort obj_index = 0; obj_index < m_list_simulated_objects.size(); obj_index++ ) {

                    float columnBegin = m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (frame_skip-1).at(frame_count).m_region_of_interest_px.x;
                    float rowBegin = m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (frame_skip-1).at(frame_count).m_region_of_interest_px.y;
                    /*
                    if ( cvRound(rowBegin)%2 != 0 && STEP_SIZE %2 != 0 )   {
                        rowBegin+=1;
                    }
                    if ( cvRound(columnBegin)%2 != 0 && STEP_SIZE%2 != 0 ) {
                        columnBegin+=1;
                    }
                     */

                    int width = cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_width_m);
                    int height = cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_height_m);

                    bool visibility = m_list_simulated_objects.at(obj_index)->get_obj_extrapolated_visibility().at(frame_skip-1).at(frame_count);
                    if ( visibility ) {

                        cv::Mat roi = stencilFrame.
                                rowRange(cvRound(rowBegin-height*(DO_STENCIL_GRID_EXTENSION/STENCIL_GRID_EXTENDER)),
                                         (cvRound(rowBegin+height+height*(DO_STENCIL_GRID_EXTENSION/STENCIL_GRID_EXTENDER)))).
                                colRange(cvRound(columnBegin-width*(DO_STENCIL_GRID_EXTENSION/STENCIL_GRID_EXTENDER)),
                                         (cvRound(columnBegin+width+width*(DO_STENCIL_GRID_EXTENSION/STENCIL_GRID_EXTENDER))));

                        cv::Size roi_size;
                        cv::Point roi_offset;
                        roi.locateROI(roi_size, roi_offset);

                        cv::Point2f gt_displacement = m_list_gt_objects.at(obj_index)->get_obj_extrapolated_pixel_position_pixel_displacement().at
                                (frame_skip-1).at(frame_count).second;
                        // This is for the base model
                        std::vector<std::vector<bool>  > base_visibility(m_list_simulated_objects.size());
                        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> >  > base_movement(m_list_simulated_objects.size());
                        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> >  > stencil_movement(m_list_simulated_objects.size());

                        if ( noise == "none") {

                            assert(m_list_simulated_objects.size() == m_list_gt_objects.size());

                            std::cout << "ground truth value " << rowBegin << " " << columnBegin << " " << gt_displacement << std::endl;

                            std::cout << "making a stencil on the basis of groundtruth object " << m_list_gt_objects.at(obj_index)->getObjectId() << std::endl;

                            auto COUNT = m_list_simulated_objects_base.size();
                            assert(COUNT==0);
                            //std::cout << next_pts_array << std::endl;
                            for (unsigned row_index = 0; row_index < roi.rows; row_index++) {
                                for (unsigned col_index = 0; col_index < roi.cols; col_index++) {

                                    if ( col_index%STENCIL_GRID_COMPRESSOR == 0 && row_index%STENCIL_GRID_COMPRESSOR == 0 ) { // only entertain multiple of col_index pixels to reduce data

                                        cv::Point2f algo_displacement = roi.at<cv::Vec2f>(row_index, col_index);
                                        auto dist_algo = cv::norm(algo_displacement);
                                        if ( dist_algo < 0.1 ) {
                                            continue;
                                        }

                                        //std::cout << roi_offset.x + col_index << std::endl;
                                        for ( auto next_pts_index = 0; next_pts_index < next_pts_array.size(); next_pts_index++ ) {
                                            if ( (( roi_offset.x + col_index ) == next_pts_array.at(next_pts_index).x) &&
                                                    (( roi_offset.y + row_index ) == next_pts_array.at(next_pts_index).y)) {
                                                stencil_movement.at(obj_index).push_back(
                                                        std::make_pair(cv::Point2f(roi_offset.x + col_index, roi_offset.y + row_index),
                                                                       algo_displacement));
                                                base_movement.at(obj_index).push_back(std::make_pair(
                                                        cv::Point2f((roi_offset.x + col_index), (roi_offset.y + row_index)),
                                                        algo_displacement));
                                                base_visibility.at(obj_index).push_back(visibility);

                                            }
                                        }
                                    }
                                }
                            }
                        }
                        else {

                            assert(m_list_simulated_objects.size() == m_list_simulated_objects_base.size());
                            std::cout << "making a stencil on the basis of base algorithm object " << m_list_simulated_objects_base.at(obj_index)->getObjectId() << std::endl;


                            for (unsigned row_index = 0; row_index < roi.rows; row_index++) {
                                for (unsigned col_index = 0; col_index < roi.cols; col_index++) {

                                    if ( col_index%STENCIL_GRID_COMPRESSOR == 0 && row_index%STENCIL_GRID_COMPRESSOR == 0 ) { // only entertain multiple of col_index pixels to reduce data

                                        cv::Point2f algo_displacement = roi.at<cv::Vec2f>(row_index, col_index);
                                        auto dist_algo = cv::norm(algo_displacement);
                                        if ( dist_algo < 0.1 ) {
                                            continue;
                                        }
                                        //std::cout << roi_offset.x + col_index << std::endl;
                                        for ( auto next_pts_index = 0; next_pts_index < next_pts_array.size(); next_pts_index++ ) {
                                            if ( (( roi_offset.x + col_index ) == next_pts_array.at(next_pts_index).x) &&
                                                 (( roi_offset.y + row_index ) == next_pts_array.at(next_pts_index).y)) {
                                                stencil_movement.at(obj_index).push_back(
                                                        std::make_pair(cv::Point2f(roi_offset.x + col_index, roi_offset.y + row_index),
                                                                       algo_displacement));

                                                auto dist_gt = cv::norm(gt_displacement);
                                                auto dist_err = std::abs(dist_gt - dist_algo);
                                                if (dist_err < DISTANCE_ERROR_TOLERANCE) {
                                                    auto angle_err = std::cosh(
                                                            algo_displacement.dot(gt_displacement) / (dist_gt * dist_algo));
                                                    if (((std::abs(angle_err)) < ANGLE_ERROR_TOLERANCE)) {
                                                        // stencil_movement.at(obj_index).push_back(std::make_pair(cv::Point2f(roi_offset.x + x,roi_offset.y + y), algo_displacement));
                                                    }
                                                }
                                                base_movement.at(obj_index).push_back(
                                                        std::make_pair(cv::Point2f((roi_offset.x + col_index), (roi_offset.y + row_index)),
                                                                       algo_displacement));
                                                base_visibility.at(obj_index).push_back(visibility);
                                            }
                                        }
                                    }
                                }
                            }

                            /*

                            auto COUNT = m_list_simulated_objects_base.at(obj_index)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                                    (frame_skip-1).at(frame_count).size();
                            for ( auto count = 0; count < COUNT; count++ ) {

                                float x = m_list_simulated_objects_base.at(
                                        obj_index)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                                        (frame_skip - 1).at(frame_count).at(count).first.x;
                                float y = m_list_simulated_objects_base.at(
                                        obj_index)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                                        (frame_skip - 1).at(frame_count).at(count).first.y;

                                for (auto next_pts_index = 0;
                                     next_pts_index < next_pts_array.size(); next_pts_index++) {
                                    if (((x) == next_pts_array.at(next_pts_index).x) &&
                                        ((y) == next_pts_array.at(next_pts_index).y)) {
                                        cv::Point2f algo_displacement = flowFrame.at<cv::Vec2f>(y, x);
                                        stencil_movement.at(obj_index).push_back(
                                                std::make_pair(cv::Point2f(x, y), algo_displacement));
                                    }
                                }
                            }
                            */
                        }

                        auto new_stencil_size = stencil_movement.at(obj_index).size();
                        std::cout << new_stencil_size << " " << next_pts_array.size() << std::endl;
                        //assert(new_stencil_size != 0);

                        if ( new_stencil_size == 0 ) {
                            for (unsigned row_index = 0; row_index < roi.rows; row_index++) {
                                for (unsigned col_index = 0; col_index < roi.cols; col_index++) {

                                    cv::Point2f algo_displacement = m_list_gt_objects.at(obj_index)->get_obj_extrapolated_pixel_position_pixel_displacement().at(frame_skip-1).at(frame_count).second;
                                    //std::cout << roi_offset.x + col_index << std::endl;
                                    stencil_movement.at(obj_index).push_back(
                                            std::make_pair(cv::Point2f(roi_offset.x + col_index, roi_offset.y + row_index),
                                                    algo_displacement));
                                    base_movement.at(obj_index).push_back(std::make_pair(
                                            cv::Point2f((roi_offset.x + col_index), (roi_offset.y + row_index)),
                                            algo_displacement));
                                    base_visibility.at(obj_index).push_back(visibility);
                                    F_png_write.setFlowU(roi_offset.x + col_index,roi_offset.y + row_index, algo_displacement.x);
                                    F_png_write.setFlowV(roi_offset.x + col_index,roi_offset.y + row_index, algo_displacement.y);
                                    F_png_write.setValid(roi_offset.x + col_index,roi_offset.y + row_index, true);

                                }
                            }
                            std::cout << stencil_movement.at(obj_index).size() << "hacked" << next_pts_array.size() << std::endl;
                        }


                        new_stencil_size = stencil_movement.at(obj_index).size();
                        assert(new_stencil_size != 0);


                        outer_base_movement.at(obj_index).push_back(base_movement.at(obj_index));
                        outer_base_visiblity.at(obj_index).push_back(base_visibility.at(obj_index));
                        outer_stencil_movement.at(obj_index).push_back(stencil_movement.at(obj_index));

                    }
                    else {

                        outer_base_visiblity.at(obj_index).push_back({{false}});
                        outer_base_movement.at(obj_index).push_back({{std::make_pair(cv::Point2f(0, 0),cv::Point2f(0, 0))}});
                        outer_stencil_movement.at(obj_index).push_back({{std::make_pair(cv::Point2f(0, 0),cv::Point2f(0, 0))}});

                    }
                }

                F_png_write.writeExtended(temp_result_flow_path);
            }

            else {
                std::cout << "skipping first frame frame count " << frame_count << std::endl;
                // But still write the data for completion
                F_png_write.writeExtended(temp_result_flow_path);

                for ( ushort obj_index = 0; obj_index < m_list_simulated_objects.size(); obj_index++ ) {
                    outer_base_visiblity.at(obj_index).push_back({{false}});
                    outer_base_movement.at(obj_index).push_back({{std::make_pair(cv::Point2f(0, 0),cv::Point2f(0, 0))}});
                    outer_stencil_movement.at(obj_index).push_back({{std::make_pair(cv::Point2f(0, 0),cv::Point2f(0, 0))}});
                }

                needToInit = true;
            }

            CannyEdgeDetection(input_image_file_with_path, temp_result_edge_path);

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

            toc = steady_clock::now();
            time_map[algo_map[algo]] = duration_cast<milliseconds>(toc - tic).count();
            y_pts.push_back(time_map[algo_map[algo]]);
            time.push_back(duration_cast<milliseconds>(toc - tic).count());

            x_pts.push_back(frame_count);

            if ( frame_types == video_frames) {
                video_out.write(image_02_frame);
            }

            // Display the output image
            //cv::namedWindow(m_resultordner+"_" + std::to_string(frame_count), CV_WINDOW_AUTOSIZE);
            //cv::imshow(m_resultordner+"_"+std::to_string(frame_count), image_02_frame);
            //cv::imwrite(temp_result_position_path, image_02_frame);
            //cv::waitKey(0);
            cv::destroyAllWindows();
            prevGray = curGray.clone();

        }

        for ( ushort obj_index = 0; obj_index < m_list_simulated_objects.size(); obj_index++) {
            m_list_simulated_objects.at(obj_index)->generate_obj_extrapolated_shape_pixel_point_pixel_displacement_pixel_visibility(outer_base_movement.at(obj_index), outer_base_visiblity.at(obj_index));
            m_list_simulated_objects.at(obj_index)->generate_obj_extrapolated_stencil_pixel_point_pixel_displacement(outer_stencil_movement.at(obj_index));
        }

        for(auto &n : time)
            sum_time +=n;

        std::cout << "Noise " << noise  << ", Zeit " << sum_time << std::endl;
        std::cout << "time_map LK " << time_map["LK"] << std::endl;

        auto max = (std::max_element(y_pts.begin(), y_pts.end())).operator*();

        pts_exectime.push_back(boost::make_tuple(x_pts, y_pts));
        if ( frame_types == video_frames) {
            video_out.release();
        }
        cv::destroyAllWindows();

        // gnuplot_2d
        Gnuplot gp2d;
        gp2d << "set xrange [0:" + std::to_string(MAX_ITERATION_RESULTS) + "]\n";
        gp2d << "set yrange [0:" + std::to_string(max*2) + "]\n";
        std::string tmp = std::string(" with points title ") + std::string("'") + Dataset::getGroundTruthPath().string() +
                std::string(" y axis - ms, x axis - image_02_frame\n'");
        //gp2d << "plot" << gp2d.binFile2d(pts_exectime, "record") << tmp;
    }
}



void AlgorithmFlow::store_in_yaml(cv::FileStorage &fs, const cv::Point2f &l_pixelposition, const cv::Point2f
&l_pixelmovement )  {

    fs << "gt flow png file read" << "[";
    fs << "{:" << "row" <<  l_pixelposition.y << "col" << l_pixelposition.x << "displacement" << "[:";
    fs << l_pixelmovement.x;
    fs << l_pixelmovement.y;
    fs << 1;
    fs << "]" << "}";
    fs << "]";
}


