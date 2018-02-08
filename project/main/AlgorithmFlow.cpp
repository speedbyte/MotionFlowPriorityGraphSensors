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

void AlgorithmFlow::setResultOrdner(ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise) {

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

    switch ( noise ) {
        case no_noise: {
            m_resultordner += "no_noise/";
            break;
        }
        case static_bg_noise: {
            m_resultordner += "static_bg_noise/";
            break;
        }
        case static_fg_noise: {
            m_resultordner += "static_fg_noise/";
            break;
        }
        case dynamic_bg_noise: {
            m_resultordner += "dynamic_bg_noise/";
            break;
        }
        case dynamic_fg_noise: {
            m_resultordner += "dynamic_fg_noise/";
            break;
        }
        default: {
            throw("algorithm not yet supported");
        }
    }

}

void AlgorithmFlow::prepare_directories(ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise) {

    setResultOrdner(algo, frame_types, noise);

    char char_dir_append[20];
    if ( boost::filesystem::exists(Dataset::getResultPath()) ) {
        system(("rm -rf " + Dataset::getResultPath().string() + "/" + m_resultordner).c_str());
    }
    std::cout << "Creating directories" << std::endl;
    boost::filesystem::create_directories(Dataset::getResultPath().string());
    // create flow directories
    for (int i = 1; i < MAX_SKIPS; ++i) {
        sprintf(char_dir_append, "%02d", i);
        boost::filesystem::create_directories(Dataset::getResultPath().string() + "/" + m_resultordner +
                                              "/flow_occ_" + char_dir_append);
        boost::filesystem::create_directories(Dataset::getResultPath().string() + "/" + m_resultordner +
                                              "/flow_obj_" + char_dir_append);
        boost::filesystem::create_directories(Dataset::getResultPath().string() + "/" + m_resultordner +
                                              "/trajectory_occ_" + char_dir_append);
        boost::filesystem::create_directories(Dataset::getResultPath().string() + "/" + m_resultordner +
                                              "/plots_" + char_dir_append);
    }
    std::cout << "Ending directories" << std::endl;
}


void AlgorithmFlow::calculate_flow(ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise) {

    prepare_directories(algo, frame_types, noise);

    for ( int frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++ ) {



        char folder_name_flow[50], folder_name_trajectory[50];
        char file_name_image[50];

        std::vector<unsigned> x_pts;
        std::vector<double> y_pts;
        std::vector<unsigned> z_pts;
        std::vector<float> time;
        double sum_time = 0;

        ushort count = 0;
        std::vector<boost::tuple<std::vector<unsigned>, std::vector<double>> > pts_exectime;

        FlowImageExtended F_png_write_trajectory(Dataset::getFrameSize().width, Dataset::getFrameSize()
                .height);


        bool needToInit = true;
        std::vector<cv::Point2f> prev_pts_array;

        std::cout << "results will be stored in " << m_resultordner << std::endl;

        if ( frame_types == video_frames) {
            cv::VideoCapture cap;
            cap.open(Dataset::getGtPath().string() + "image_02/movement.avi");
            if (!cap.isOpened()) {
                std::cout << "Could not initialize capturing...\n";
                return;
            }
        }
        cv::Mat curGray, prevGray;
        sprintf(folder_name_flow, "flow_occ_%02d", frame_skip);
        sprintf(folder_name_trajectory, "trajectory_occ_%02d", frame_skip);
        std::string results_flow_matrix_str = Dataset::getResultPath().string() + "/" + m_resultordner + "/" +
                                              folder_name_flow + "/" + "result_flow.yaml";
        cv::VideoWriter video_out;

        if ( frame_types == video_frames)
        {
            boost::filesystem::path video_out_path = Dataset::getResultPath().string() + "/" + std::string("/video/OpticalFlow.avi");
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
        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > frame_pixel_point_pixel_displacement;

        for (ushort frame_count=0; frame_count < MAX_ITERATION_RESULTS; frame_count++) {
            //draw new ground truth flow.


            if ( frame_count%frame_skip != 0 ) {
                continue;
            }
            sprintf(file_name_image, "000%03d_10.png", frame_count);
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

            std::string input_image_file_with_path = Dataset::getGtPath().string() + "/" + file_name_image;

            image_02_frame = cv::imread(input_image_file_with_path, CV_LOAD_IMAGE_COLOR);

            if ( image_02_frame.data == NULL ) {
                std::cerr << input_image_file_with_path << " not found" << std::endl;
                throw ("No image file found error");
            }

            temp_result_flow_path = Dataset::getResultPath().string() + "/" + m_resultordner + "/" +
                                    folder_name_flow + "/" + file_name_image;
            temp_result_trajectory_path = Dataset::getResultPath().string() + "/" + m_resultordner + "/" +
                                    folder_name_trajectory + "/" + file_name_image;

            // Convert to grayscale
            cv::cvtColor(image_02_frame, curGray, cv::COLOR_BGR2GRAY);

            cv::Mat flow_frame( Dataset::getFrameSize(), CV_32FC2 );

            std::vector<cv::Point2f> next_pts_array;
            tic = steady_clock::now();

            // Calculate optical calculate_flow map using LK algorithm
            if (prevGray.data) {  // Calculate only on second or subsequent images.
                std::vector<uchar> status;
                // Initialize parameters for the optical calculate_flow algorithm
                float pyrScale = 0.5;
                int numLevels = 3;
                int windowSize = 15;
                int numIterations = 3;
                int neighborhoodSize = 5;
                float stdDeviation = 1.2;

                std::vector<float> err;

                // Calculate optical calculate_flow map using Farneback algorithm
                // Farnback returns displacement frame and LK returns points.
                cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
                if ( lk == algo ) {
                    cv::calcOpticalFlowPyrLK(prevGray, curGray, prev_pts_array, next_pts_array, status,
                                             err, winSize, 5, termcrit, 0, 0.001);
                    // TODO create a flow_frame
                }
                else if ( fb == algo ) {
                    cv::calcOpticalFlowFarneback(prevGray, curGray, flow_frame, pyrScale, numLevels, windowSize,
                                                 numIterations, neighborhoodSize, stdDeviation,
                                                 cv::OPTFLOW_FARNEBACK_GAUSSIAN);
                    // OPTFLOW_USE_INITIAL_FLOW didnt work and gave NaNs
                    cv::Mat stencilFrame;
                    stencilFrame = flow_frame.clone();
                    for ( ushort i = 0; i < m_list_simulated_objects.size(); i++ ) {
                        //two objects
                        std::vector<std::pair<cv::Point2f, cv::Point2f> > base_movement;
                        int width = m_list_simulated_objects.at(i).getWidth();
                        int height = m_list_simulated_objects.at(i).getHeight();
                        float rowBegin = m_list_objects.at(i).getExtrapolatedPixelpoint_pixelDisplacement().at
                                (frame_skip-1).at(frame_count).first.y;
                        float columnBegin = m_list_objects.at(i).getExtrapolatedPixelpoint_pixelDisplacement().at
                                (frame_skip-1).at(frame_count).first.x;

                        cv::Mat roi = stencilFrame.rowRange(cvRound(rowBegin),(cvRound(rowBegin)+height)).colRange
                                (cvRound(columnBegin),(cvRound(columnBegin)+width));
                        //cv::Mat tempObject = roi.clone();

                        for (unsigned y = 0; y < roi.rows; y++) {
                            for (unsigned x = 0; x < roi.cols; x++) {

                                base_movement.push_back(std::make_pair(cv::Point2f(cvRound(columnBegin)+x, cvRound
                                                                                                                (rowBegin)
                                                                                                        +y),
                                                                       roi.at<cv::Vec2f>(y,x)));
                            }
                        }
                        m_list_simulated_objects.at(i).set_outer_base_movement(base_movement);
                    }
                }

                // Draw the optical calculate_flow map
                int stepSize = 1;

                if ( fb == algo ) {
                    // Draw the uniform grid of points on the input image along with the motion vectors
                    // Circles to indicate the uniform grid of points
                    //cv::circle(image_02_frame, cv::Point(x, y), 1, cv::Scalar(0, 0, 0), -1, 8);
                    prev_pts_array.clear();
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

                unsigned count = 0;

                std::vector<std::pair<cv::Point2f, cv::Point2f> > frame_points;
                for (unsigned i = 0; i < next_pts_array.size(); i++) {

                    int minDist = 1;

                    cv::Point2f next_pts, displacement;
                    cv::Point2f algorithmMovement ((next_pts_array[i].x - prev_pts_array[i].x), (next_pts_array[i].y - prev_pts_array[i]
                            .y));

                    // Check if the status vector is good
                    if (!status[i])
                        continue;


                    printf("flow_frame.at<cv::Point2f>(%f, %f).x =  %f\n", next_pts_array[i].x, next_pts_array[i].y,
                           algorithmMovement.x);
                    printf("flow_frame.at<cv::Point2f>(%f, %f).y =  %f\n", next_pts_array[i].x, next_pts_array[i].y,
                           algorithmMovement.y);

                    displacement.x = cvRound( algorithmMovement.x + 0.5);
                    displacement.y = cvRound( algorithmMovement.y + 0.5);

                    /* If the new point is within 'minDist' distance from an existing point, it will not be tracked */
                    // auto dist = cv::norm(prev_pts_array[i] - next_pts_array[i]);
                    double dist;
                    dist = pow(displacement.x,2)+pow(displacement.y,2);
                    //calculating distance by euclidean formula
                    dist = sqrt(dist);

                    if ( dist <= minDist ) {
                        printf("minimum distance for %i is %f\n", i, dist);
                        continue;
                    }

                    if ( displacement.x == 0 && displacement.y == 0) {
                        continue;
                    }

                    next_pts_array[count++] = next_pts_array[i];

                    // next_pts is the new pixel position !
                    next_pts.x = std::abs(cvRound(next_pts_array[i].x));
                    next_pts.y = std::abs(cvRound(next_pts_array[i].y));

                    printf("(iteration %u, coordinates x y (%f,%f) ->  Vx, Vy (%f,%f) \n", i,
                           next_pts.x, next_pts.y, displacement.x, displacement.y);
                    // Lines to indicate the motion vectors
                    frame_points.push_back(std::make_pair(next_pts, displacement));
                }
                frame_pixel_point_pixel_displacement.push_back(frame_points);
                next_pts_array.resize(count);


                for (unsigned i = 0; i < next_pts_array.size(); i++) {
                    cv::arrowedLine(image_02_frame, prev_pts_array[i], next_pts_array[i], cv::Scalar(0, 255, 0));
                }
                if ( next_pts_array.size() == 0 ) {
                    // pick up the last healthy points
                    next_pts_array = next_pts_healthy;
                }

            }
            else {
                needToInit = true;
            }
            if ( needToInit ) { //|| ( frame_count%4 == 0) ) {
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

            if (prevGray.data) {

                //Create png Matrix with 3 channels: x displacement. y displacment and Validation bit
                //kitti uses col, row specification
                FlowImageExtended F_png_write(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

                std::vector<std::pair<cv::Point2f, cv::Point2f> >::iterator it ;

                std::cout << "frame_count " << frame_count << std::endl;
                fs << "frame_count" << frame_count;


                for ( it = frame_pixel_point_pixel_displacement.at(count).begin(); it !=
                        frame_pixel_point_pixel_displacement.at(count).end(); it++ )
                {

                    F_png_write.setFlowU((*it).first.x,(*it).first.y,(*it).second.x);
                    F_png_write.setFlowV((*it).first.x,(*it).first.y,(*it).second.y);
                    F_png_write.setValid((*it).first.x,(*it).first.y,(bool)1.0f);
                    store_in_yaml(fs, (*it).first, (*it).second ); // coordinate - > movement y(row),x(col) ; x,y

                    F_png_write_trajectory.setFlowU((*it).first.x,(*it).first.y,(*it).second.x);
                    F_png_write_trajectory.setFlowV((*it).first.x,(*it).first.y,(*it).second.y);
                    F_png_write_trajectory.setValid((*it).first.x,(*it).first.y,(bool)1.0f);

                }

                count++;
                F_png_write.write(temp_result_flow_path);
                F_png_write_trajectory.write(temp_result_trajectory_path);

            }

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

        //F_png_write_trajectory.write(temp_result_flow_path);
        //m_algo_extrapolated_frame_pixel_point_pixel_displacement.push_back(frame_pixel_point_pixel_displacement);
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
        std::string tmp = std::string(" with points title ") + std::string("'") + Dataset::getGtPath().string() +
                std::string(" y axis - ms, x axis - image_02_frame\n'");
        //gp2d << "plot" << gp2d.binFile2d(pts_exectime, "record") << tmp;
        for ( ushort i = 0; i < m_list_simulated_objects.size(); i++) {
            m_list_simulated_objects.at(i).trigger_m_simulated_obj_extrapolated_shape_pixel_point_pixel_displacement();
        }
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



void AlgorithmFlow::generate_collision_points() {

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the trajectory in a png file


    char folder_name_flow[50];
    cv::FileStorage fs;

    fs.open(Dataset::getResultPath().string() + "/" + m_resultordner + "/" + folder_name_flow + "/" + "gt_flow.yaml",
            cv::FileStorage::WRITE);

    std::vector<SimulatedObjects>::const_iterator objectIterator = m_list_simulated_objects.begin();
    std::vector<SimulatedObjects>::const_iterator  objectIteratorNext;

    for ( ; objectIterator < m_list_simulated_objects.end() ; objectIterator++ ) {
        for ( objectIteratorNext = objectIterator+1; objectIteratorNext < m_list_simulated_objects.end();
              objectIteratorNext++) {

            m_list_objects_combination.push_back(std::make_pair((*objectIterator), (*objectIteratorNext)));
            std::cout << "collision between object id " << (*objectIterator).getObjectId() << " and object id " <<
                      (*objectIteratorNext).getObjectId() << "\n";

        }
    }

    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

        sprintf(folder_name_flow, "flow_obj_%02d", frame_skip);
        std::cout << "saving flow files for frame_skip " << frame_skip << std::endl;

        unsigned FRAME_COUNT = (unsigned)m_list_simulated_objects.at(0).get_simulated_obj_extrapolated_shape_pixel_point_pixel_displacement().at
                (frame_skip - 1).size();

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
            char file_name_image[50];
            std::cout << "frame_count " << frame_count << std::endl;

            sprintf(file_name_image, "000%03d_10.png", frame_count);
            std::string temp_gt_flow_image_path =
                    Dataset::getResultPath().string() + "/" + m_resultordner + "/" + folder_name_flow + "/" +
                            file_name_image;
            fs << "frame_count" << frame_count;


            float *data_ = (float*)malloc(Dataset::getFrameSize().width*Dataset::getFrameSize().height*3*sizeof(float));
            memset(data_, 255, Dataset::getFrameSize().width*Dataset::getFrameSize().height*3*sizeof(float));
            FlowImageExtended F_png_write(data_, Dataset::getFrameSize().width, Dataset::getFrameSize().height);
            cv::Mat tempMatrix;
            tempMatrix.create(Dataset::getFrameSize(), CV_32FC3);
            tempMatrix = cv::Scalar_<unsigned>(255,255,255);
            assert(tempMatrix.channels() == 3);


            for (unsigned i = 0; i < m_list_simulated_objects.size(); i++) {

                // object image_data_and_shape
                int width = m_list_simulated_objects.at(i).getWidth();
                int height = m_list_simulated_objects.at(i).getHeight();

                //if ( m_list_simulated_objects.at(i).getExtrapolatedVisibility().at(frame_skip - 1).at(frame_count)
                //      == true ) {
                if ( 1 ) {

                    // gt_displacement
                    cv::Point2f next_pts = m_list_simulated_objects.at(i)
                            .getSimulatedExtrapolatedPixelCentroid_DisplacementMean().at(frame_skip - 1)
                            .at(frame_count).first;
                    cv::Point2f displacement = m_list_simulated_objects.at(i)
                            .getSimulatedExtrapolatedPixelCentroid_DisplacementMean().at(frame_skip- 1)
                            .at(frame_count).second;

                    cv::Point2f gt_line_pts = m_list_simulated_objects.at(i).getLineParameters().at(frame_skip - 1)
                            .at(frame_count).second;


                    cv::Mat roi;
                    roi = tempMatrix.
                            colRange(cvRound(next_pts.x), cvRound(next_pts.x + width)).
                            rowRange(cvRound(next_pts.y), cvRound(next_pts.y + height));
                    //bulk storage
                    roi = cv::Scalar(displacement.x, displacement.y,
                                     static_cast<float>(m_list_simulated_objects.at(i).getObjectId()));

                    // cv line is intelligent and it can also project to values not within the frame size including negative values.
                    cv::line(tempMatrix, next_pts, gt_line_pts, cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0);
                }
            }

            std::vector<cv::Point2f> collision_points;

            for ( unsigned i = 0; i < m_list_objects_combination.size(); i++) {

                /*if ( ( m_list_objects_combination.at(i).first.getExtrapolatedVisibility().at(frame_skip - 1)
                               .at(frame_count) == true ) && ( m_list_objects_combination.at(i).second
                                                                       .getExtrapolatedVisibility()
                                                                       .at(frame_skip - 1)
                                                                       .at(frame_count) == true )) { */
                if ( 1 ) {

                    cv::Point2f lineparameters1 = m_list_objects_combination.at(i).first.getLineParameters().at(frame_skip - 1)
                            .at(frame_count).first;

                    cv::Point2f lineparameters2 = m_list_objects_combination.at(i).second.getLineParameters().at(frame_skip - 1)
                            .at(frame_count).first;

                    // first fill rowco
                    cv::Matx<float,2,2> coefficients (-lineparameters1.x,1,-lineparameters2.x,1);
                    cv::Matx<float,2,1> rhs(lineparameters1.y,lineparameters2.y);

                    std::cout << "object 1  = " << lineparameters1 << " and object 2 = " << lineparameters2 << std::endl ;

                    cv::Matx<float,2,1> result_manual;
                    if ( cv::determinant(coefficients ) != 0 ) {
                        result_manual = (cv::Matx<float,2,2>)coefficients.inv()*rhs;
                        //result_manual = coefficients.solve(rhs);
                        cv::circle(tempMatrix, cv::Point2f(result_manual(0,0), result_manual(1,0)), 5, cv::Scalar(0, 255, 0), -1,
                                   cv::LINE_AA);

                        std::cout << "collision points x = " << result_manual(0,0) << " and y = " << result_manual(1,0) << std::endl ;
                        collision_points.push_back(cv::Point2f(result_manual(0,0), result_manual(1,0)));
                    }
                    else {
                        std::cerr << "Determinant is singular" << std::endl;
                        //assert ( cv::determinant(coefficients ) != 0 );
                        //result_manual(0,0) = -5;
                        //result_manual(1,0) = -5;
                        //collision_points.push_back(cv::Point2f(result_manual(0,0), result_manual(1,0)));
                    }
                }
            }

            m_frame_collision_points.push_back(collision_points);



            //Create png Matrix with 3 channels: x displacement. y displacment and ObjectId
            for (int32_t row = 0; row < Dataset::getFrameSize().height; row++) { // rows
                for (int32_t column = 0; column < Dataset::getFrameSize().width; column++) {  // cols
                    if (tempMatrix.at<cv::Vec3f>(row, column)[2] > 0.5 ) {
                        F_png_write.setFlowU(column, row, tempMatrix.at<cv::Vec3f>(row, column)[1]);
                        F_png_write.setFlowV(column, row, tempMatrix.at<cv::Vec3f>(row, column)[0]);
                        F_png_write.setObjectId(column, row, tempMatrix.at<cv::Vec3f>(row, column)[2]);
                        //trajectory.store_in_yaml(fs, cv::Point2f(row, column), cv::Point2f(xValue, yValue) );
                    }
                }
            }

            F_png_write.writeExtended(temp_gt_flow_image_path);

        }
        m_frame_skip_collision_points.push_back(m_frame_collision_points);
    }

    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
    //toc_all = steady_clock::now();
    //time_map["ground truth"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    //std::cout << "ground truth flow generation time - " << time_map["ground truth"] << "ms" << std::endl;

}
