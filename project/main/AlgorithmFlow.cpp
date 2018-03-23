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

    mImageabholOrt = Dataset::getGroundTruthPath().string() + "/" + *m_ptr_environment + "/";
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
        char file_name_input_image[50];

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

        cv::Mat image_02_frame = cv::Mat::zeros(Dataset::getFrameSize(), CV_8UC3);
        cv::Mat flowImage(Dataset::getFrameSize(), CV_32FC3);

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

        std::string temp_result_flow_path, temp_result_position_path;
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
            flowImage = cv::Scalar(0,0,0);
            assert(flowImage.channels() == 3);
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
            temp_result_position_path = m_position_occ_path.string() + frame_skip_folder_suffix + "/" + file_name_input_image;

            // Convert to grayscale
            cv::cvtColor(image_02_frame, curGray, cv::COLOR_BGR2GRAY);

            cv::Mat flowFrame( Dataset::getFrameSize(), CV_32FC2 );

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
                int windowSize = 10;
                int numIterations = 5;
                int neighborhoodSize = 5;
                float stdDeviation = 1.5;

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
                int stepSize = 4;

                if ( fb == algo ) {
                    // Circles to indicate the uniform grid of points
                    //cv::circle(image_02_frame, cv::Point(x, y), 1, cv::Scalar(0, 0, 0), -1, 8);
                    prev_pts_array.clear();
                    next_pts_array.clear();
                    for (int row = 0; row < image_02_frame.rows; row += stepSize) {
                        for (int col = 0; col < image_02_frame.cols; col += stepSize) {

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

                    F_png_write.setFlowU((*it).x,(*it).y,(*it2).x);
                    F_png_write.setFlowV((*it).x,(*it).y,(*it2).y);
                    F_png_write.setValid((*it).x,(*it).y,(bool)1.0f);
                    // TODO - store objectId instead of 1.0. also convert to 2s complement by the below formula.
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

                for ( ushort i = 0; i < m_list_simulated_objects.size(); i++ ) {

                    float columnBegin = m_list_gt_objects.at(i)->get_obj_extrapolated_pixel_position_pixel_displacement().at
                            (frame_skip-1).at(frame_count).first.x;
                    float rowBegin = m_list_gt_objects.at(i)->get_obj_extrapolated_pixel_position_pixel_displacement().at
                            (frame_skip-1).at(frame_count).first.y;

                    int width = cvRound(m_list_gt_objects.at(i)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_width_m);
                    int height = cvRound(m_list_gt_objects.at(i)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_height_m);

                    bool visibility = m_list_simulated_objects.at(i)->get_obj_extrapolated_visibility().at(frame_skip-1).at(frame_count);
                    if ( visibility ) {

                        cv::Mat roi = stencilFrame.rowRange(cvRound(rowBegin-height/STENCIL_GRID_EXTENDER),(cvRound(rowBegin)+height+height/STENCIL_GRID_EXTENDER)).colRange
                                (cvRound(columnBegin-width/STENCIL_GRID_EXTENDER),(cvRound(columnBegin)+width+width/STENCIL_GRID_EXTENDER));

                        cv::Size roi_size;
                        cv::Point roi_offset;
                        roi.locateROI(roi_size, roi_offset);

                        cv::Point2f gt_displacement = m_list_gt_objects.at(i)->get_obj_extrapolated_pixel_position_pixel_displacement().at
                                (frame_skip-1).at(frame_count).second;
                        // This is for the base model
                        std::vector<std::vector<bool>  > base_visibility(m_list_simulated_objects.size());
                        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> >  > base_movement(m_list_simulated_objects.size());
                        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> >  > stencil_movement(m_list_simulated_objects.size());

                        if ( noise == "none") {

                            assert(m_list_simulated_objects.size() == m_list_gt_objects.size());

                            std::cout << "ground truth value " << rowBegin << " " << columnBegin << " " << gt_displacement << std::endl;

                            std::cout << "making a stencil on the basis of groundtruth object " << m_list_gt_objects.at(i)->getObjectId() << std::endl;

                            auto COUNT = m_list_simulated_base_objects.size();
                            assert(COUNT==0);
                            for (unsigned y = 0; y < roi.rows; y++) {
                                for (unsigned x = 0; x < roi.cols; x++) {

                                    if ( x%STENCIL_GRID_COMPRESSOR == 0 && y%STENCIL_GRID_COMPRESSOR == 0 ) { // only entertain multiple of x pixels to reduce data

                                        cv::Point2f algo_displacement = roi.at<cv::Vec2f>(y, x);
                                        auto dist_algo = cv::norm(algo_displacement);
                                        if ( dist_algo < 0.1 ) {
                                            continue;
                                        }
                                        stencil_movement.at(i).push_back(
                                                std::make_pair(cv::Point2f(roi_offset.x + x, roi_offset.y + y),
                                                               algo_displacement));
                                        base_movement.at(i).push_back(std::make_pair(
                                                cv::Point2f((roi_offset.x + x), (roi_offset.y + y)),
                                                algo_displacement));
                                        base_visibility.at(i).push_back(visibility);
                                    }
                                }
                            }
                            auto new_stencil_size = stencil_movement.at(i).size();
                            std::cout << new_stencil_size << std::endl;
                            assert(new_stencil_size != 0);
                        }
                        else {

                            assert(m_list_simulated_objects.size() == m_list_simulated_base_objects.size());
                            std::cout << "making a stencil on the basis of base algorithm object " << m_list_simulated_base_objects.at(i)->getObjectId() << std::endl;

                            auto COUNT = m_list_simulated_base_objects.at(i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                                    (frame_skip-1).at(frame_count).size();
                            for ( auto count = 0; count < COUNT; count++ ) {
                                float x  = m_list_simulated_base_objects.at(i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                                        (frame_skip-1).at(frame_count).at(count).first.x;
                                float y  = m_list_simulated_base_objects.at(i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                                        (frame_skip-1).at(frame_count).at(count).first.y;
                                cv::Point2f algo_displacement = flowFrame.at<cv::Vec2f>(y,x);
                                // If I return the centroid of the ground truth, then the centroid of the simulated object would be the same as the ground truth object
                                stencil_movement.at(i).push_back(std::make_pair(cv::Point2f(x, y), algo_displacement));
                            }

                            auto new_stencil_size = stencil_movement.at(i).size();
                            std::cout << new_stencil_size << std::endl;

                            // This is for the noisy model
                            for (unsigned y = 0; y < roi.rows; y++) {
                                for (unsigned x = 0; x < roi.cols; x++) {

                                    if (x % STENCIL_GRID_COMPRESSOR == 0 && y % STENCIL_GRID_COMPRESSOR ==
                                                                    0) { // only entertain multiple of 5 pixels to reduce data
                                        cv::Point2f algo_displacement = roi.at<cv::Vec2f>(y, x);
                                        auto dist_gt = cv::norm(gt_displacement);
                                        auto dist_algo = cv::norm(algo_displacement);
                                        auto dist_err = std::abs(dist_gt - dist_algo);
                                        if (dist_err < DISTANCE_ERROR_TOLERANCE) {
                                            auto angle_err = std::cosh(
                                                    algo_displacement.dot(gt_displacement) / (dist_gt * dist_algo));
                                            if (((std::abs(angle_err)) < ANGLE_ERROR_TOLERANCE)) {
                                                // If I return the centroid of the ground truth, then the centroid of the simulated object would be the same as the ground truth object
                                                // the stencil is going to be generated as above
                                                // stencil_movement.at(i).push_back(std::make_pair(cv::Point2f(roi_offset.x + x,roi_offset.y + y), algo_displacement));
                                            }
                                        }
                                        base_movement.at(i).push_back(
                                                std::make_pair(cv::Point2f((roi_offset.x + x), (roi_offset.y + y)),
                                                               algo_displacement));
                                        base_visibility.at(i).push_back(visibility);

                                    }
                                }
                            }
                        }

                        outer_base_movement.at(i).push_back(base_movement.at(i));
                        outer_base_visiblity.at(i).push_back(base_visibility.at(i));
                        outer_stencil_movement.at(i).push_back(stencil_movement.at(i));

                    }
                    else {

                        outer_base_visiblity.at(i).push_back({{false}});
                        outer_base_movement.at(i).push_back({{std::make_pair(cv::Point2f(0, 0),cv::Point2f(0, 0))}});
                        outer_stencil_movement.at(i).push_back({{std::make_pair(cv::Point2f(0, 0),cv::Point2f(0, 0))}});

                    }
                }

                F_png_write.write(temp_result_flow_path);
            }

            else {
                std::cout << "skipping first frame frame count " << frame_count << std::endl;
                // But still write the data for completion
                F_png_write.write(temp_result_flow_path);

                for ( ushort i = 0; i < m_list_simulated_objects.size(); i++ ) {
                    outer_base_visiblity.at(i).push_back({{false}});
                    outer_base_movement.at(i).push_back({{std::make_pair(cv::Point2f(0, 0),cv::Point2f(0, 0))}});
                    outer_stencil_movement.at(i).push_back({{std::make_pair(cv::Point2f(0, 0),cv::Point2f(0, 0))}});
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
            cv::imshow(m_resultordner+"_"+std::to_string(frame_count), image_02_frame);
            cv::imwrite(temp_result_position_path, image_02_frame);
            prevGray = curGray.clone();

        }

        for ( ushort i = 0; i < m_list_simulated_objects.size(); i++) {
            m_list_simulated_objects.at(i)->generate_obj_extrapolated_shape_pixel_point_pixel_displacement_pixel_visibility(outer_base_movement.at(i), outer_base_visiblity.at(i));
            m_list_simulated_objects.at(i)->generate_obj_extrapolated_stencil_pixel_point_pixel_displacement(outer_stencil_movement.at(i));
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


void AlgorithmFlow::generate_shape_points() {

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the position in a png file

    std::vector<std::pair<Objects*, Objects*> > list_of_gt_objects_combination;
    std::vector<std::pair<Objects*, Objects*> > list_of_simulated_objects_combination;

    getCombination(m_list_gt_objects, list_of_gt_objects_combination);
    getCombination(m_list_simulated_objects, list_of_simulated_objects_combination);

    std::map<std::string, double> time_map = {{"generate",     0},
                                              {"generate_flow", 0}};

    auto tic = steady_clock::now();
    auto toc = steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();

    char frame_skip_folder_suffix[50];
    cv::FileStorage fs;

    for ( ushort i = 0; i < list_of_simulated_objects_combination.size(); i ++ ) {
        std::cout << "shape between object id " << list_of_simulated_objects_combination.at(i).first->getObjectId() <<
                  " and object id "
                  << list_of_simulated_objects_combination.at(i).second->getObjectId()<< "\n";
    }


    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

        sprintf(frame_skip_folder_suffix, "%02d", frame_skip);

        std::cout << "generating shape points in OpticalFlow.cpp for " << m_resultordner << " " << frame_skip << std::endl;

        std::vector<std::vector<cv::Point2f> >  m_frame_shape_points;

        unsigned FRAME_COUNT = (unsigned)m_list_simulated_objects.at(0)
                ->get_obj_extrapolated_shape_pixel_point_pixel_displacement().at(frame_skip - 1).size();
        assert(FRAME_COUNT>0);

        for (ushort frame_count = 1; frame_count < FRAME_COUNT; frame_count++) {


            std::cout << "frame_count " << frame_count << std::endl;

            fs << "frame_count" << frame_count;

            cv::Point2f shape_average = {0,0};
            std::vector<cv::Point2f> shape_points(m_list_simulated_objects.size());
            std::vector<cv::Point2f> shape_points_average;


            for ( unsigned i = 0; i < m_list_simulated_objects.size(); i++) {


                auto CLUSTER_COUNT_GT = m_list_gt_objects.at(i)->get_obj_extrapolated_shape_pixel_point_pixel_displacement().at
                        (frame_skip - 1).at(frame_count).size();

                auto CLUSTER_COUNT_ALGO = m_list_simulated_objects.at(i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                        (frame_skip - 1).at(frame_count).size();

                if ( ( m_list_simulated_objects.at(i)->get_obj_extrapolated_mean_visibility().at(frame_skip - 1)
                               .at(frame_count) == true ) ) {


                    float rowBegin = m_list_gt_objects.at(i)->get_obj_extrapolated_pixel_position_pixel_displacement().at
                            (frame_skip-1).at(frame_count).first.y;
                    float columnBegin = m_list_gt_objects.at(i)->get_obj_extrapolated_pixel_position_pixel_displacement().at
                            (frame_skip-1).at(frame_count).first.x;


                    // Instances of CLUSTER_COUNT_ALGO in CLUSTER_COUNT_GT

                    float vollTreffer = 0;

                    int width = cvRound(m_list_gt_objects.at(i)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_width_m);
                    int height = cvRound(m_list_gt_objects.at(i)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_height_m);

                    float keinTreffer;

                    if ( m_resultordner == "/generated" ) {  // for ground truth
                        vollTreffer = CLUSTER_COUNT_GT;
                        keinTreffer = (CLUSTER_COUNT_GT - vollTreffer);
                    }
                    else { // for real algorithms
                        for ( auto j = 0; j < CLUSTER_COUNT_ALGO; j++ ) {
                            auto x_coordinates =  m_list_simulated_objects.at(i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                                    (frame_skip - 1).at(frame_count).at(j).first.x;
                            auto y_coordinates = m_list_simulated_objects.at(i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                                    (frame_skip - 1).at(frame_count).at(j).first.y;

                            if ((x_coordinates > (columnBegin - width / STENCIL_GRID_EXTENDER)) &&
                                (x_coordinates < (columnBegin + width + width / STENCIL_GRID_EXTENDER)) &&
                                (y_coordinates > (rowBegin - height / STENCIL_GRID_EXTENDER)) &&
                                (y_coordinates < (rowBegin + height + height / STENCIL_GRID_EXTENDER))
                                    ) {
                                vollTreffer++;
                            }
                        }
                        keinTreffer = ((float)CLUSTER_COUNT_ALGO - vollTreffer);
                    }
                    shape_points.at(i) = (cv::Point2f(vollTreffer, keinTreffer));

                    std::cout << "vollTreffer for object " << m_list_simulated_objects.at(i)->getObjectId() << " = " << vollTreffer << std::endl ;
                    std::cout << "keinTreffer for object " << m_list_simulated_objects.at(i)->getObjectId() << " = " << keinTreffer << std::endl ;

                    shape_average.x += shape_points.at(i).x;
                    shape_average.y += shape_points.at(i).y;

                }
                else {
                    std::cout << "visibility of object " << m_list_simulated_objects.at(i)->getObjectId() << " = " <<
                              m_list_simulated_objects.at(i)->get_obj_extrapolated_mean_visibility().at(frame_skip
                                                                                                        - 1)
                                      .at(frame_count) << " and hence not generating any shape points for this object " <<  std::endl ;

                    shape_points.at(i) = (cv::Point2f(0, CLUSTER_COUNT_ALGO));
                    shape_average.x += shape_points.at(i).x;
                    shape_average.y += shape_points.at(i).y;

                }
            }

            shape_average.x = shape_average.x / m_list_simulated_objects.size();
            shape_average.y = shape_average.y / m_list_simulated_objects.size();

            shape_points_average.push_back( shape_average );

            m_frame_shape_points.push_back(shape_points_average);

        }
        m_frame_skip_shape_points.push_back(m_frame_shape_points);
    }

    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
    toc_all = steady_clock::now();
    time_map["generate_flow"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << m_resultordner + " flow generation time - " << time_map["generate_flow"] << "ms" << std::endl;
}

void AlgorithmFlow::generate_collision_points_mean() {


    std::vector<std::pair<Objects*, Objects*> > list_of_gt_objects_combination;
    std::vector<std::pair<Objects*, Objects* > > list_of_simulated_objects_combination;

    getCombination(m_list_gt_objects, list_of_gt_objects_combination);
    getCombination(m_list_simulated_objects, list_of_simulated_objects_combination);

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the position in a png file

    std::map<std::string, double> time_map = {{"generate",     0},
                                              {"generate_flow", 0}};

    auto tic = steady_clock::now();
    auto toc = steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();

    char frame_skip_folder_suffix[50];
    cv::FileStorage fs;


    for ( ushort i = 0; i < list_of_simulated_objects_combination.size(); i ++ ) {
        std::cout << "collision between object id " << list_of_simulated_objects_combination.at(i).first->getObjectId() <<
                  " and object id "
                  << list_of_simulated_objects_combination.at(i).second->getObjectId()<< "\n";
    }

    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

        sprintf(frame_skip_folder_suffix, "%02d", frame_skip);

        std::cout << "generating collision points in OpticalFlow.cpp for " << m_resultordner << " " << frame_skip << std::endl;

        std::vector<std::vector<cv::Point2f> >  m_frame_collision_points;

        unsigned FRAME_COUNT = (unsigned)m_list_simulated_objects.at(0)
                ->get_obj_extrapolated_mean_pixel_centroid_pixel_displacement().at
                        (frame_skip - 1).size() - 1; // we store the flow image here and hence it starts at 1. Correspondingly the size reduces.
        assert(FRAME_COUNT>0);

        for (ushort frame_count = 1; frame_count < FRAME_COUNT; frame_count++) {
            char file_name_image[50];
            std::cout << "frame_count " << frame_count << std::endl;

            sprintf(file_name_image, "000%03d_10.png", frame_count*frame_skip);
            std::string temp_collision_image_path =
                    m_collision_obj_path.string() + frame_skip_folder_suffix + "/" + file_name_image;

            fs << "frame_count" << frame_count;

            FlowImageExtended F_png_write(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

            cv::Mat tempMatrix;
            tempMatrix.create(Dataset::getFrameSize(), CV_32FC3);
            tempMatrix = cv::Scalar_<unsigned>(255,255,255);
            assert(tempMatrix.channels() == 3);

            for (unsigned i = 0; i < m_list_simulated_objects.size(); i++) {

                // object image_data_and_shape

                if ( m_list_simulated_objects.at(i)->get_obj_extrapolated_mean_visibility().at(frame_skip - 1).at(frame_count)
                     == true ) {

                    cv::Point2f centroid = m_list_simulated_objects.at(i)->
                                    get_obj_extrapolated_mean_pixel_centroid_pixel_displacement().at(frame_skip - 1)
                            .at(frame_count).first;
                    cv::Point2f mean_displacement = m_list_simulated_objects.at(i)->
                                    get_obj_extrapolated_mean_pixel_centroid_pixel_displacement().at(frame_skip- 1)
                            .at(frame_count).second;

                    cv::Point2f gt_line_pts = m_list_simulated_objects.at(i)->get_line_parameters().at(frame_skip - 1)
                            .at(frame_count-1).second;  //line parameters run one less than the others.

                    cv::Mat roi;

                    int width = cvRound(m_list_gt_objects.at(i)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_width_m);
                    int height = cvRound(m_list_gt_objects.at(i)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_height_m);

                    roi = tempMatrix.
                            colRange(cvRound(centroid.x), cvRound(centroid.x + width)).
                            rowRange(cvRound(centroid.y), cvRound(centroid.y + height));
                    //bulk storage
                    roi = cv::Scalar(mean_displacement.x, mean_displacement.y,
                                     static_cast<float>(m_list_simulated_objects.at(i)->getObjectId()));

                    // cv line is intelligent and it can also project to values not within the frame size including negative values.
                    // cv::line(tempMatrix, centroid, gt_line_pts, cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0);
                }
            }

            std::vector<cv::Point2f> collision_points;
            std::vector<cv::Point2f> collision_points_average;
            for ( unsigned i = 0; i < list_of_simulated_objects_combination.size(); i++) {

                if ( ( list_of_simulated_objects_combination.at(i).first->get_obj_extrapolated_mean_visibility().at(frame_skip
                                                                                                                    - 1)
                               .at(frame_count) == true ) && ( list_of_simulated_objects_combination.at(i).second->
                                get_obj_extrapolated_mean_visibility()
                                                                       .at(frame_skip - 1)
                                                                       .at(frame_count) == true )) {

                    // First Freeze lineparamter1 and look for collision points
                    // Then freeze lineparameter2 and find collision point.
                    // Then push_back the two points in the vector

                    for ( auto j = 0; j < 1; j++ ) {

                        cv::Point2f lineparameters1 = list_of_simulated_objects_combination.at(i).first->get_line_parameters().at
                                        (frame_skip - 1)
                                .at(frame_count-1).first;

                        cv::Point2f lineparameters2 = list_of_gt_objects_combination.at(i).second->get_line_parameters
                                        ().at(frame_skip - 1)
                                .at(frame_count-1).first;

                        std::cout << "object " << list_of_simulated_objects_combination.at(i).first->getObjectId() << " = " <<
                                  lineparameters1 << " and object " << list_of_gt_objects_combination.at(i)
                                          .second->getObjectId() << " = " <<lineparameters2 << std::endl ;

                        OpticalFlow::find_collision_points_given_two_line_parameters(lineparameters1, lineparameters2, tempMatrix, collision_points);

                        lineparameters1 = list_of_simulated_objects_combination.at(i).second->get_line_parameters().at
                                        (frame_skip - 1)
                                .at(frame_count-1).first;

                        lineparameters2 = list_of_gt_objects_combination.at(i).first->get_line_parameters
                                        ().at(frame_skip - 1)
                                .at(frame_count-1).first;

                        std::cout << "object " << list_of_simulated_objects_combination.at(i).second->getObjectId() << " = " <<
                                  lineparameters1 << " and object " << list_of_gt_objects_combination.at(i)
                                          .first->getObjectId() << " = " <<lineparameters2 << std::endl ;

                        find_collision_points_given_two_line_parameters(lineparameters1, lineparameters2, tempMatrix, collision_points);

                    }

                }
                else {
                    std::cout << "object " << list_of_simulated_objects_combination.at(i).first->getObjectId() << " visibility = " <<
                              list_of_simulated_objects_combination.at(i).first->get_obj_extrapolated_mean_visibility().at(frame_skip
                                                                                                                           - 1)
                                      .at(frame_count) << " and object " << list_of_gt_objects_combination.at(i)
                                      .second->getObjectId() << " visibility = " << list_of_simulated_objects_combination.at(i).second->get_obj_extrapolated_mean_visibility().at(frame_skip
                                                                                                                                                                                  - 1)
                                      .at(frame_count) << " and hence not generating any collision points for this object combination " <<  std::endl ;
                }
            }


            for ( auto i = 0; i < collision_points.size(); i=i+2 ) {
                if ( collision_points.at(i) != cv::Point2f(-1,-1) && collision_points.at(i+1) != cv::Point2f(-1,-1)) {
                    collision_points_average.push_back(cv::Point2f(((collision_points.at(i).x + collision_points.at(i+1).x ) / 2),
                                                                   ((collision_points.at(i).y + collision_points.at(i+1).y ) / 2)));
                }
            }

            m_frame_collision_points.push_back(collision_points_average);

            //Create png Matrix with 3 channels: x mean_displacement. y displacment and ObjectId
            for (int32_t row = 0; row < Dataset::getFrameSize().height; row++) { // rows
                for (int32_t column = 0; column < Dataset::getFrameSize().width; column++) {  // cols
                    if (tempMatrix.at<cv::Vec3f>(row, column)[2] > 0.5 ) {
                        F_png_write.setFlowU(column, row, tempMatrix.at<cv::Vec3f>(row, column)[1]);
                        F_png_write.setFlowV(column, row, tempMatrix.at<cv::Vec3f>(row, column)[0]);
                        F_png_write.setObjectId(column, row, tempMatrix.at<cv::Vec3f>(row, column)[2]);
                        //position.store_in_yaml(fs, cv::Point2f(row, column), cv::Point2f(xValue, yValue) );
                    }
                }
            }

            F_png_write.writeExtended(temp_collision_image_path);

        }
        m_frame_skip_collision_points.push_back(m_frame_collision_points);
    }

    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
    toc_all = steady_clock::now();
    time_map["generate_flow"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << m_resultordner + " flow generation time - " << time_map["generate_flow"] << "ms" << std::endl;
}




void AlgorithmFlow::visualiseStencil(void) {

    std::cout << "visualise stencil at " << m_generatepath.string() + "stencil/" << std::endl;

    char file_name_image[50], file_name_image_output[50];

    cv::Mat image_data_and_shape;

    const ushort frame_skip = 1; // image is generated only once irrespective of skips.
    cv::Mat tempGroundTruthImage(Dataset::getFrameSize(), CV_8UC3);

    for ( int frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++ ) {

        for (ushort frame_count = 0; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_IMAGES; frame_count++) {

            std::string output_image_file_with_path;
            /*---------------------------------------------------------------------------------*/
            tempGroundTruthImage = cv::Scalar::all(255);

            sprintf(file_name_image, "000%03d_10.png", frame_count*frame_skip);
            std::string input_image_file_with_path = mImageabholOrt.string() + file_name_image;

            sprintf(file_name_image_output, "000%03d_10_bb.png", frame_count*frame_skip);
            output_image_file_with_path = m_generatepath.string() + "stencil/" + file_name_image_output;

            cv::Mat tempGroundTruthImageBase = cv::imread(input_image_file_with_path, CV_LOAD_IMAGE_ANYCOLOR);
            tempGroundTruthImage = cv::Scalar::all(255);
            tempGroundTruthImage = tempGroundTruthImageBase.clone();

            for ( unsigned  i = 0; i < m_list_gt_objects.size(); i++ ) {

                if ( ( m_list_gt_objects.at(i)->get_obj_base_visibility().at(frame_count))
                        ) {

                    //cv::Rect boundingbox =  cv::Rect(cvRound(m_list_objects.at(i).get_obj_base_pixel_position_pixel_displacement().at(frame_count).first.x - (cvRound(m_list_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_count).m_object_dimensions_px.dim_length_m/2))),
                    cv::Rect boundingbox = cv::Rect(
                            cvRound(m_list_gt_objects.at(i)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_location_px.location_x_m),
                            cvRound(m_list_gt_objects.at(i)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_location_px.location_y_m),
                            cvRound(m_list_gt_objects.at(i)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_width_m),
                            cvRound(m_list_gt_objects.at(i)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_height_m));


                    cv::rectangle(tempGroundTruthImage, boundingbox, cv::Scalar(0, 255, 0), 1, 8, 0);

                }
            }
            cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
            /*---------------------------------------------------------------------------------*/
            tempGroundTruthImage = cv::Scalar::all(0);

            sprintf(file_name_image_output, "000%03d_10_stencil_gt.png", frame_count * frame_skip);
            output_image_file_with_path =
                    m_generatepath.string() + "stencil/" + file_name_image_output;

            for (unsigned i = 0; i < m_list_gt_objects.size(); i++) {

                if ((m_list_gt_objects.at(i)->get_obj_base_visibility().at(frame_count))
                        ) {

                    const unsigned CLUSTER_SIZE = (unsigned) m_list_gt_objects.at(
                            i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                            (frame_skip - 1).at(frame_count).size();
                    for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {

                        cv::Point2f pts = m_list_gt_objects.at(
                                        i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_point).first;

                     cv::circle(tempGroundTruthImage, pts, 1.5, cv::Scalar(255, 255, 255), 1, 8);

                    }
                }
            }
            cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
            /*---------------------------------------------------------------------------------*/
            tempGroundTruthImage = cv::Scalar::all(0);

            sprintf(file_name_image_output, "000%03d_10_stencil_algo.png", frame_count * frame_skip);
            output_image_file_with_path =
                    m_generatepath.string() + "stencil/" + file_name_image_output;

            for (unsigned i = 0; i < m_list_simulated_base_objects.size(); i++) {

                if ((m_list_simulated_base_objects.at(i)->get_obj_extrapolated_visibility().at(frame_skip-1).at(frame_count)
                )) {

                    const unsigned CLUSTER_SIZE = (unsigned) m_list_simulated_base_objects.at(
                            i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                            (frame_skip - 1).at(frame_count).size();
                    for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {

                        cv::Point2f pts = m_list_simulated_base_objects.at(
                                        i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_point).first;

                    cv::circle(tempGroundTruthImage, pts, 1.5, cv::Scalar(255, 255, 255), 1, 8);

                    }
                }
            }
            cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
            /*---------------------------------------------------------------------------------*/
            tempGroundTruthImage = cv::Scalar::all(255);

            sprintf(file_name_image_output, "000%03d_10_pixel_gt.png", frame_count * frame_skip);
            output_image_file_with_path =
                    m_generatepath.string() + "stencil/" + file_name_image_output;

            for (unsigned i = 0; i < m_list_gt_objects.size(); i++) {

                if ((m_list_gt_objects.at(i)->get_obj_base_visibility().at(frame_count))
                        ) {

                    const unsigned CLUSTER_SIZE = (unsigned) m_list_gt_objects.at(
                            i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                            (frame_skip - 1).at(frame_count).size();
                    for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {

                        cv::Point2f pts = m_list_gt_objects.at(
                                        i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_point).first;

                        cv::Point2f displacement = m_list_gt_objects.at(
                                        i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_point).second;

                        cv::circle(tempGroundTruthImage, pts, 1.5, cv::Scalar(0, 255, 0), 1, 8);

                    }
                }
            }
            cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
            /*---------------------------------------------------------------------------------*/
            tempGroundTruthImage = cv::Scalar::all(255);

            sprintf(file_name_image_output, "000%03d_10_pixel_base_algo.png", frame_count * frame_skip);
            output_image_file_with_path =
                    m_generatepath.string() + "stencil/" + file_name_image_output;

            for (unsigned i = 0; i < m_list_simulated_base_objects.size(); i++) {

                if ((m_list_simulated_base_objects.at(i)->get_obj_extrapolated_visibility().at(frame_skip-1).at(frame_count)
                )) {

                    const unsigned CLUSTER_SIZE = (unsigned) m_list_simulated_base_objects.at(
                            i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                            (frame_skip - 1).at(frame_count).size();
                    for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {

                        cv::Point2f pts = m_list_simulated_base_objects.at(
                                        i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_point).first;

                        cv::Point2f displacement = m_list_simulated_base_objects.at(
                                        i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_point).second;

                        cv::circle(tempGroundTruthImage, pts, 1.5, cv::Scalar(255, 0, 0), 1, 8);

                    }
                }
            }
            cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
            /*---------------------------------------------------------------------------------*/

            tempGroundTruthImage = cv::Scalar::all(255);

            sprintf(file_name_image_output, "000%03d_10_pixel_algo.png", frame_count * frame_skip);
            output_image_file_with_path =
                    m_generatepath.string() + "stencil/" + file_name_image_output;

            for (unsigned i = 0; i < m_list_simulated_objects.size(); i++) {

                if ((m_list_simulated_objects.at(i)->get_obj_extrapolated_visibility().at(frame_skip-1).at(frame_count)
                        )) {

                    const unsigned CLUSTER_SIZE = (unsigned) m_list_simulated_objects.at(
                            i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                            (frame_skip - 1).at(frame_count).size();
                    for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {

                        cv::Point2f pts = m_list_simulated_objects.at(
                                        i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_point).first;

                        cv::Point2f displacement = m_list_simulated_objects.at(
                                        i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_point).second;

                        cv::circle(tempGroundTruthImage, pts, 1.5, cv::Scalar(0, 0, 255), 1, 8);

                    }
                }
            }
            cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
            /*---------------------------------------------------------------------------------*/

            tempGroundTruthImage = cv::Scalar::all(255);

            sprintf(file_name_image_output, "000%03d_10_flow_gt.png", frame_count * frame_skip);
            output_image_file_with_path =
                    m_generatepath.string() + "stencil/" + file_name_image_output;

            for (unsigned i = 0; i < m_list_gt_objects.size(); i++) {

                if ((m_list_gt_objects.at(i)->get_obj_extrapolated_visibility().at(frame_skip-1).at(frame_count)
                )) {

                    const unsigned CLUSTER_SIZE = (unsigned) m_list_gt_objects.at(
                            i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                            (frame_skip - 1).at(frame_count).size();
                    for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {

                        cv::Point2f pts = m_list_gt_objects.at(
                                        i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_point).first;

                        cv::Point2f displacement = m_list_gt_objects.at(
                                        i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_point).second;

                        cv::Point2f next_pts = cv::Point2f(pts.x+displacement.x, pts.y+displacement.y);

                        cv::arrowedLine(tempGroundTruthImage, pts, next_pts, cv::Scalar(0, 255, 0), 1, 8, 0, 0.25);

                    }
                }
            }
            cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
            /*---------------------------------------------------------------------------------*/

            tempGroundTruthImage = cv::Scalar::all(255);

            sprintf(file_name_image_output, "000%03d_10_flow_base_algo.png", frame_count * frame_skip);
            output_image_file_with_path =
                    m_generatepath.string() + "stencil/" + file_name_image_output;

            for (unsigned i = 0; i < m_list_simulated_base_objects.size(); i++) {

                if ((m_list_simulated_base_objects.at(i)->get_obj_extrapolated_visibility().at(frame_skip-1).at(frame_count)
                )) {

                    const unsigned CLUSTER_SIZE = (unsigned) m_list_simulated_base_objects.at(
                            i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                            (frame_skip - 1).at(frame_count).size();
                    for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {

                        cv::Point2f pts = m_list_simulated_base_objects.at(
                                        i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_point).first;

                        cv::Point2f displacement = m_list_simulated_base_objects.at(
                                        i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_point).second;

                        cv::Point2f next_pts = cv::Point2f(pts.x+displacement.x, pts.y+displacement.y);

                        cv::arrowedLine(tempGroundTruthImage, pts, next_pts, cv::Scalar(255, 0, 0), 1, 8, 0, 0.25);

                    }
                }
            }
            cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
            /*---------------------------------------------------------------------------------*/

            tempGroundTruthImage = cv::Scalar::all(255);

            sprintf(file_name_image_output, "000%03d_10_flow_algo.png", frame_count * frame_skip);
            output_image_file_with_path =
                    m_generatepath.string() + "stencil/" + file_name_image_output;

            for (unsigned i = 0; i < m_list_simulated_objects.size(); i++) {

                if ((m_list_simulated_objects.at(i)->get_obj_extrapolated_visibility().at(frame_skip-1).at(frame_count)
                )) {

                    const unsigned CLUSTER_SIZE = (unsigned) m_list_simulated_objects.at(
                            i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                            (frame_skip - 1).at(frame_count).size();
                    for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {

                        cv::Point2f pts = m_list_simulated_objects.at(
                                        i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_point).first;

                        cv::Point2f displacement = m_list_simulated_objects.at(
                                        i)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_point).second;

                        cv::Point2f next_pts = cv::Point2f(pts.x+displacement.x, pts.y+displacement.y);

                        cv::arrowedLine(tempGroundTruthImage, pts, next_pts, cv::Scalar(0, 0, 255), 1, 8, 0, 0.25);

                    }
                }
            }
            cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
        }
    }
}
