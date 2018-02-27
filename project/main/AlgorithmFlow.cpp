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


void AlgorithmFlow::generate_flow_frame(ALGO_TYPES algo, FRAME_TYPES frame_types, std::string noise,
                                        const std::vector<SimulatedObjects> &base_algo_simulated_object_list) {

    prepare_directories(algo, frame_types, noise);

    for ( int frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++ ) {


        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > outer_base_movement(m_list_simulated_objects.size());
        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > outer_stencil_movement(m_list_simulated_objects.size());
        std::vector<std::vector<bool>  > outer_base_visiblity;

        char frame_skip_folder_suffix[50];
        char file_name_input_image[50];

        std::vector<unsigned> x_pts;
        std::vector<double> y_pts;
        std::vector<unsigned> z_pts;
        std::vector<float> time;
        double sum_time = 0;

        std::vector<boost::tuple<std::vector<unsigned>, std::vector<double>> > pts_exectime;

        FlowImageExtended F_png_write_trajectory(Dataset::getFrameSize().width, Dataset::getFrameSize()
                .height);


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

        cv::namedWindow(m_resultordner, CV_WINDOW_AUTOSIZE);

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

        std::string temp_result_flow_path, temp_result_trajectory_path;
        cv::FileStorage fs;
        fs.open(results_flow_matrix_str, cv::FileStorage::WRITE);
        std::vector<cv::Point2f> next_pts_healthy;

        std::cout << "creating flow files for frame_skip " << frame_skip << std::endl;
        std::vector<cv::Point2f> prev_pts_array;

        for (ushort frame_count=0; frame_count < MAX_ITERATION_RESULTS; frame_count++) {
            //draw new ground truth flow.

            if ( frame_count*frame_skip >= MAX_ITERATION_RESULTS) {
                break;
            }

            std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> >  > base_movement(m_list_simulated_objects.size());
            std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> >  > stencil_movement(m_list_simulated_objects.size());
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
            temp_result_trajectory_path = m_trajectory_occ_path.string() + frame_skip_folder_suffix + "/" + file_name_input_image;

            // Convert to grayscale
            cv::cvtColor(image_02_frame, curGray, cv::COLOR_BGR2GRAY);

            cv::Mat flow_frame( Dataset::getFrameSize(), CV_32FC2 );

            std::vector<cv::Point2f> next_pts_array;
            tic = steady_clock::now();

            //Create png Matrix with 3 channels: x displacement. y displacment and Validation bit
            //kitti uses col, row specification
            FlowImageExtended F_png_write(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

            // Calculate optical generate_flow_frame map using LK algorithm
            if (prevGray.data) {  // Calculate only on second or subsequent images.

                std::cout << "frame_count " << frame_count << std::endl;
                fs << "frame_count" << frame_count;

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
                    cv::calcOpticalFlowFarneback(prevGray, curGray, flow_frame, pyrScale, numLevels, windowSize,
                                                 numIterations, neighborhoodSize, stdDeviation,
                                                 cv::OPTFLOW_FARNEBACK_GAUSSIAN);
                    // OPTFLOW_USE_INITIAL_FLOW didnt work and gave NaNs
                }

                // Draw the optical generate_flow_frame map
                int stepSize = 4;

                if ( fb == algo ) {
                    // Draw the uniform grid of points on the input image along with the motion vectors
                    // Circles to indicate the uniform grid of points
                    //cv::circle(image_02_frame, cv::Point(x, y), 1, cv::Scalar(0, 0, 0), -1, 8);
                    prev_pts_array.clear();
                    next_pts_array.clear();
                    for (int row = 0; row < image_02_frame.rows; row += stepSize) {
                        for (int col = 0; col < image_02_frame.cols; col += stepSize) {

                            cv::Point2f algorithmMovement ( flow_frame.at<cv::Point2f>(row, col).x, flow_frame
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

                std::vector<std::pair<cv::Point2f, cv::Point2f> > frame_points;
                for (unsigned i = 0; i < next_pts_array.size(); i++) {

                    int minDist = 1;

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

                    std::cout << "next valid points " << next_pts << " displacement " << displacement << std::endl;
                    next_pts_array[count_good_points++] = next_pts_array[i];

                    frame_points.push_back(std::make_pair(next_pts, displacement));

                }

                next_pts_array.resize(count_good_points); // this is required for LK. For FB, anyways the frame will
                // be completely calculated every time.

                if ( next_pts_array.size() == 0 ) {
                    // pick up the last healthy points
                    next_pts_array = next_pts_healthy;
                }

                for (unsigned i = 0; i < next_pts_array.size(); i++) {
                    cv::arrowedLine(image_02_frame, prev_pts_array[i], next_pts_array[i], cv::Scalar(0, 255, 0));
                }

                std::vector<std::pair<cv::Point2f, cv::Point2f> >::iterator it ;



                for ( it = frame_points.begin(); it !=frame_points.end(); it++ )
                {

                    F_png_write.setFlowU((*it).first.x,(*it).first.y,(*it).second.x);
                    F_png_write.setFlowV((*it).first.x,(*it).first.y,(*it).second.y);
                    F_png_write.setValid((*it).first.x,(*it).first.y,(bool)1.0f);
                    // TODO - store objectId instead of 1.0. also convert to 2s complement by the below formula.
                    store_in_yaml(fs, (*it).first, (*it).second ); // coordinate - > movement y(row),x(col) ; x,y

                    F_png_write_trajectory.setFlowU((*it).first.x,(*it).first.y,(*it).second.x);
                    F_png_write_trajectory.setFlowV((*it).first.x,(*it).first.y,(*it).second.y);
                    F_png_write_trajectory.setValid((*it).first.x,(*it).first.y,(bool)1.0f);

                }

                cv::Mat flowframe;
                flowframe.create(Dataset::getFrameSize(), CV_32FC3);

                cv::MatIterator_<cv::Vec3f> it_flowframe = flowframe.begin<cv::Vec3f>();
                for (unsigned i = 0; it_flowframe != flowframe.end<cv::Vec3f>(); it_flowframe++ ) {
                    for ( unsigned j = 0; j < 3; j++ ) {
                        float temp = *(F_png_write.data_ + i );
                        if ( temp != 0 ) {
                            (*it_flowframe)[j] = temp;
                        }
                        // std::min(temp*64.0f+32768.0f,65535.0f),0.0f)
                        i++;
                    }
                }

                cv::Mat stencilFrame;
                stencilFrame = flowframe.clone();
                for ( ushort i = 0; i < m_list_simulated_objects.size(); i++ ) {
                    //two objects

                    int width = m_list_simulated_objects.at(i).getWidth();
                    int height = m_list_simulated_objects.at(i).getHeight();

                    // The stencil should be created by comparing the ground truth data and only taking
                    // those points that is within a boundary.
                    // Call ground truth point, call, width and height. Find, matching points in the Algorihtm flow.
                    // Store the matching points as a stencil.
                    // Find similarity between next_pts_array[i] and get_obj_extrapolated_pixel_point_pixel_displacement(). How many valid displacements in a bigger shape?? And if valid displacement, punch this point.

                    float rowBegin = m_list_gt_objects.at(i).get_obj_extrapolated_pixel_point_pixel_displacement().at
                            (frame_skip-1).at(frame_count).first.y;
                    float columnBegin = m_list_gt_objects.at(i).get_obj_extrapolated_pixel_point_pixel_displacement().at
                            (frame_skip-1).at(frame_count).first.x;


                    cv::Mat roi = stencilFrame.rowRange(cvRound(rowBegin-height),(cvRound(rowBegin)+height+height)).colRange
                            (cvRound(columnBegin-width),(cvRound(columnBegin)+width+width));

                    // Here I should write the fact
                    // roi_write = cv::Scalar(i,j,k);
                    //

                    cv::Size roi_size;
                    cv::Point roi_offset;
                    roi.locateROI(roi_size, roi_offset);

                    cv::Point2f gt_displacement = m_list_gt_objects.at(i).get_obj_extrapolated_pixel_point_pixel_displacement().at
                            (frame_skip-1).at(frame_count).second;
                    // This is for the base model
                    if ( noise == "none") {

                        assert(m_list_simulated_objects.size() == m_list_gt_objects.size());

                        std::cout << "expected value " << rowBegin << " " << columnBegin << " " << gt_displacement << std::endl;

                        std::cout << "making a stencil on the basis of groundtruth object " << m_list_gt_objects.at(i).getObjectId() << std::endl;

                        auto COUNT = base_algo_simulated_object_list.size();
                        assert(COUNT==0);
                        for (unsigned y = 0; y < roi.rows; y++) {
                            for (unsigned x = 0; x < roi.cols; x++) {

                                cv::Point2f algo_displacement = roi.at<cv::Vec2f>(y,x);
                                auto dist_gt = cv::norm(gt_displacement);
                                auto dist_algo = cv::norm(algo_displacement);
                                if ((int)dist_algo < 1  ) {
                                    continue;
                                }
                                auto dist_err = std::abs(dist_gt-dist_algo);
                                if ( dist_err < DISTANCE_ERROR_TOLERANCE ) {
                                    auto angle_err = std::cosh(algo_displacement.dot(gt_displacement) / (dist_gt*dist_algo));
                                    if ( ( ( std::abs(angle_err) ) < ANGLE_ERROR_TOLERANCE ) ) {
                                        // If I return the centroid of the ground truth, then the centroid of the simulated object would be the same as the ground truth object
                                        stencil_movement.at(i).push_back(std::make_pair(cv::Point2f(roi_offset.x + x,roi_offset.y + y), algo_displacement));
                                    }
                                    base_movement.at(i).push_back(std::make_pair(cv::Point2f((roi_offset.x + x), (roi_offset.y + y)),
                                                                                 algo_displacement));

                                }
                            }
                        }
                        auto new_stencil_size = stencil_movement.at(i).size();
                        std::cout << new_stencil_size << std::endl;

                        bool hack = true;
                        if ( hack && new_stencil_size == 0 ) {
                            stencil_movement.at(i).push_back(std::make_pair(cv::Point2f(0,0), cv::Point2f(0,0)));
                        }
                        else {
                            if ( new_stencil_size == 0 ) {
                                if ( frame_count == 1 ) {  // Inital frame is quite problematic
                                    stencil_movement.at(i).push_back(std::make_pair(m_list_gt_objects.at(i).get_obj_base_pixel_point_pixel_displacement().at(frame_count).first,
                                            m_list_gt_objects.at(i).get_obj_base_pixel_point_pixel_displacement().at(frame_count).second));
                                }
                                else {
                                    assert(new_stencil_size>0);
                                }
                            }
                        }

                    }
                    else {

                        assert(m_list_simulated_objects.size() == base_algo_simulated_object_list.size());
                        std::cout << "making a stencil on the basis of base algorithm object " << base_algo_simulated_object_list.at(i).getObjectId() << std::endl;

                        auto COUNT = base_algo_simulated_object_list.at(i).get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                                (frame_skip-1).at(frame_count).size();
                        for ( auto count = 0; count < COUNT; count++ ) {
                            float x  = base_algo_simulated_object_list.at(i).get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                                    (frame_skip-1).at(frame_count).at(count).first.x;
                            float y  = base_algo_simulated_object_list.at(i).get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                                    (frame_skip-1).at(frame_count).at(count).first.y;
                            cv::Point2f algo_displacement = flow_frame.at<cv::Vec2f>(y,x);
                            // If I return the centroid of the ground truth, then the centroid of the simulated object would be the same as the ground truth object
                            stencil_movement.at(i).push_back(std::make_pair(cv::Point2f(x, y), algo_displacement));
                        }

                        auto new_stencil_size = stencil_movement.at(i).size();
                        std::cout << new_stencil_size << std::endl;

                        // This is for the noisy model
                        for (unsigned y = 0; y < roi.rows; y++) {
                            for (unsigned x = 0; x < roi.cols; x++) {
                                cv::Point2f algo_displacement = roi.at<cv::Vec2f>(y,x);
                                auto dist_gt = cv::norm(gt_displacement);
                                auto dist_algo = cv::norm(algo_displacement);
                                auto dist_err = std::abs(dist_gt-dist_algo);
                                if ( dist_err < DISTANCE_ERROR_TOLERANCE ) {
                                    auto angle_err = std::cosh(algo_displacement.dot(gt_displacement) / (dist_gt*dist_algo));
                                    if ( ( ( std::abs(angle_err) ) < ANGLE_ERROR_TOLERANCE ) ) {
                                        // If I return the centroid of the ground truth, then the centroid of the simulated object would be the same as the ground truth object
                                        // the stencil is going to be generated as above
                                        // stencil_movement.at(i).push_back(std::make_pair(cv::Point2f(roi_offset.x + x,roi_offset.y + y), algo_displacement));
                                    }
                                }
                                base_movement.at(i).push_back(std::make_pair(cv::Point2f((roi_offset.x + x), (roi_offset.y + y)),
                                                                             algo_displacement));
                            }
                        }
                    }

                    outer_base_movement.at(i).push_back(base_movement.at(i));
                    outer_stencil_movement.at(i).push_back(stencil_movement.at(i));

                }

                //cv::imwrite(temp_result_flow_path, flowframe);
                F_png_write.write(temp_result_flow_path);
                F_png_write_trajectory.write(temp_result_trajectory_path);

            }

            else {
                std::cout << "skipping first frame frame count " << frame_count << std::endl;
                // But still write the data for completion
                F_png_write.write(temp_result_flow_path);
                F_png_write_trajectory.write(temp_result_trajectory_path);

                for ( ushort i = 0; i < m_list_simulated_objects.size(); i++ ) {
                    base_movement.at(i).push_back(std::make_pair(cv::Point2f(0, 0),cv::Point2f(0, 0)));
                    stencil_movement.at(i).push_back(std::make_pair(cv::Point2f(0, 0),cv::Point2f(0, 0)));
                    outer_base_movement.at(i).push_back(base_movement.at(i));
                    outer_stencil_movement.at(i).push_back(stencil_movement.at(i));
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
            cv::imshow(m_resultordner, image_02_frame);
            prevGray = curGray.clone();

        }

        for ( ushort i = 0; i < m_list_simulated_objects.size(); i++) {
            m_list_simulated_objects.at(i).generate_obj_extrapolated_shape_pixel_point_pixel_displacement(outer_base_movement.at(i));
            m_list_simulated_objects.at(i).generate_obj_extrapolated_stencil_pixel_point_pixel_displacement(outer_stencil_movement.at(i));
        }


        fs.release();

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



