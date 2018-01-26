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
#include "Dataset.h"
#include "ObjectTrajectory.h"


using namespace std::chrono;

AlgorithmFlow::AlgorithmFlow(Dataset dataset) {

    m_dataset = dataset;

}

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
void AlgorithmFlow::prepare_result_directories(std::string resultordner) {

    char char_dir_append[20];
    if ( boost::filesystem::exists(m_dataset.getResultPath()) ) {
        system(("rm -rf " + m_dataset.getResultPath().string() + "/" + resultordner).c_str());
    }
    std::cout << "Creating directories" << std::endl;
    boost::filesystem::create_directories(m_dataset.getResultPath().string());
    // create flow directories
    for (int i = 1; i < 10; ++i) {
        sprintf(char_dir_append, "%02d", i);
        boost::filesystem::create_directories(m_dataset.getResultPath().string() + "/" + resultordner +
                                              "/flow_occ_" + char_dir_append);
        boost::filesystem::create_directories(m_dataset.getResultPath().string() + "/" + resultordner +
                                              "/plots_" + char_dir_append);
    }
    std::cout << "Ending directories" << std::endl;
}


void AlgorithmFlow::calculate_flow(const boost::filesystem::path dataset_path, const std::string input_image_folder,
                                     ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise) {

    std::string resultordner = "results_";
    switch ( algo ) {
        case lk: {
            resultordner += "LK_";
            break;
        }
        case fb: {
            resultordner += "FB_";
            break;
        }
        default: {
            throw("algorithm not yet supported");
            break;
        }
    }

    switch ( noise ) {
        case no_noise: {
            resultordner += "no_noise/";
            break;
        }
        case static_bg_noise: {
            resultordner += "static_bg_noise/";
            break;
        }
        case static_fg_noise: {
            resultordner += "static_fg_noise/";
            break;
        }
        case dynamic_bg_noise: {
            resultordner += "dynamic_bg_noise/";
            break;
        }
        case dynamic_fg_noise: {
            resultordner += "dynamic_fg_noise/";
            break;
        }
        default: {
            throw("algorithm not yet supported");
            break;
        }
    }

    prepare_result_directories(resultordner);


    for ( int frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++ ){

        char folder_name_flow[50];
        char file_name_image[50];

        std::vector<unsigned> x_pts;
        std::vector<double> y_pts;
        std::vector<unsigned> z_pts;
        std::vector<float> time;
        double sum_time = 0;

        std::vector<boost::tuple<std::vector<unsigned>, std::vector<double>> > pts_exectime;

        bool needToInit = true;
        std::vector<cv::Point2f> prev_pts;

        std::cout << "results will be stored in " << resultordner << std::endl;

        if ( frame_types == video_frames) {
            cv::VideoCapture cap;
            cap.open(m_dataset.getInputPath().string() + "image_02/movement.avi");
            if (!cap.isOpened()) {
                std::cout << "Could not initialize capturing...\n";
                return;
            }
        }
        cv::Mat curGray, prevGray;
        sprintf(folder_name_flow, "flow_occ_%02d", frame_skip);
        std::string results_flow_matrix_str = m_dataset.getResultPath().string() + "/" + resultordner + "/" +
                                              folder_name_flow + "/" + "result_flow.yaml";
        cv::VideoWriter video_out;

        if ( frame_types == video_frames)
        {
            boost::filesystem::path video_out_path = m_dataset.getResultPath().string() + "/" + std::string("/video/OpticalFlow.avi");
            assert(boost::filesystem::exists(video_out_path.parent_path()) != 0);
            //frame_size.height =	(unsigned) cap.get(CV_CAP_PROP_FRAME_HEIGHT );
            //frame_size.width =	(unsigned) cap.get(CV_CAP_PROP_FRAME_WIDTH );
            video_out.open(video_out_path.string(),CV_FOURCC('D','I','V','X'), 5, m_dataset.getFrameSize());
            printf("Writer eingerichtet\n");
        }

        cv::Mat image_02_frame = cv::Mat::zeros(m_dataset.getFrameSize(), CV_8UC3);
        cv::Mat flowImage(m_dataset.getFrameSize(), CV_32FC3);

        cv::Size subPixWinSize(10, 10), winSize(21, 21);

        const int MAX_COUNT = 5000;

        cv::namedWindow(resultordner, CV_WINDOW_AUTOSIZE);

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

        std::string temp_result_flow_path;
        cv::FileStorage fs;
        fs.open(results_flow_matrix_str, cv::FileStorage::WRITE);
        std::vector<cv::Point2f> next_pts_healthy;


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
                    prev_pts.clear();
                    break;
                default:
                    break;
            }

            //cap >> image_02_frame;
            //if (image_02_frame.empty())
            //    break;

            std::string input_image_file_with_path = m_dataset.getInputPath().string() + "/image_02/" + file_name_image;

            image_02_frame = cv::imread(input_image_file_with_path, CV_LOAD_IMAGE_COLOR);

            if ( image_02_frame.data == NULL ) {
                std::cerr << input_image_file_with_path << " not found" << std::endl;
                throw ("No image file found error");
            }

            temp_result_flow_path = m_dataset.getResultPath().string() + "/" + resultordner + "/" +
                                    folder_name_flow + "/" + file_name_image;

            // Convert to grayscale
            cv::cvtColor(image_02_frame, curGray, cv::COLOR_BGR2GRAY);

            cv::Mat flow_frame( m_dataset.getFrameSize(), CV_32FC2 );

            std::vector<cv::Point2f> next_pts;
            tic = steady_clock::now();
            ObjectTrajectory trajectory;

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
                    cv::calcOpticalFlowPyrLK(prevGray, curGray, prev_pts, next_pts, status,
                                             err, winSize, 5, termcrit, 0, 0.001);
                }
                else if ( fb == algo ) {
                    cv::calcOpticalFlowFarneback(prevGray, curGray, flow_frame, pyrScale, numLevels, windowSize,
                                                 numIterations, neighborhoodSize, stdDeviation,
                                                 cv::OPTFLOW_FARNEBACK_GAUSSIAN);
                    // OPTFLOW_USE_INITIAL_FLOW didnt work and gave NaNs
                }

                // Draw the optical calculate_flow map
                int stepSize = 10;

                if ( fb == algo ) {
                    // Draw the uniform grid of points on the input image along with the motion vectors
                    // Circles to indicate the uniform grid of points
                    //cv::circle(image_02_frame, cv::Point(x, y), 1, cv::Scalar(0, 0, 0), -1, 8);
                    prev_pts.clear();
                    for (int row = 0; row < image_02_frame.rows; row += stepSize) {
                        for (int col = 0; col < image_02_frame.cols; col += stepSize) {

                            cv::Point2f algorithmMovement ( flow_frame.at<cv::Point2f>(row, col).x, flow_frame
                                    .at<cv::Point2f>(row, col).y );

                            if (( cvFloor(std::abs(algorithmMovement.x)) == 0 && cvFloor(std::abs(algorithmMovement
                                                                                                          .y)) == 0 )) {
                                continue;
                            }

                            next_pts.push_back(cv::Point2f(col, row));
                            prev_pts.push_back(cv::Point2f((col - algorithmMovement.x), (row - algorithmMovement.y)));

                            status.push_back(1);
                        }
                    }
                }

                trajectory.storeData(prev_pts, next_pts, status);
                for (unsigned i = 0; i < next_pts.size(); i++) {
                    cv::arrowedLine(image_02_frame, prev_pts[i], next_pts[i], cv::Scalar(0, 255, 0));
                }
                if ( next_pts.size() == 0 ) {
                    // pick up the last healthy points
                    next_pts = next_pts_healthy;
                }

            }
            else {
                needToInit = true;
            }
            if ( needToInit ) { //|| ( frame_count%4 == 0) ) {
                //|| next_pts.size() == 0) { // the init should be also when there is no next_pts.
                // automatic initialization
                cv::goodFeaturesToTrack(curGray, next_pts, MAX_COUNT, 0.01, 10, cv::Mat(), 3, false, 0.04);
                // Refining the location of the feature points
                assert(next_pts.size() <= MAX_COUNT );
                std::cout << next_pts.size() << std::endl;
                std::vector<cv::Point2f> currentPoint;
                std::swap(currentPoint, next_pts);
                next_pts.clear();
                for (unsigned i = 0; i < currentPoint.size(); i++) {
                    std::vector<cv::Point2f> tempPoints;
                    tempPoints.push_back(currentPoint[i]);
                    // Function to refine the location of the corners to subpixel accuracy.
                    // Here, 'pixel' refers to the image patch of size 'windowSize' and not the actual image pixel
                    cv::TermCriteria termcrit_subpixel(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
                    cv::cornerSubPix(curGray, tempPoints, subPixWinSize, cv::Size(-1, -1), termcrit_subpixel);
                    next_pts.push_back(tempPoints[0]);
                }
                printf("old next_pts size is %ld and new next_pts size is %ld\n", currentPoint.size(), next_pts.size());
            }

            needToInit = false;
            prev_pts = next_pts;
            next_pts_healthy = prev_pts;
            next_pts.clear();

            if (prevGray.data) {

                trajectory.store_in_png(fs, frame_count, m_dataset.getFrameSize(), temp_result_flow_path);
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
            cv::imshow(resultordner, image_02_frame);
            prevGray = curGray.clone();
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
        std::string tmp = std::string(" with points title ") + std::string("'") + input_image_folder + std::string(" y "
                                                                                                                           "axis - ms, x axis - image_02_frame\n'");
        //gp2d << "plot" << gp2d.binFile2d(pts_exectime, "record") << tmp;
    }

}

void AlgorithmFlow::plot(std::string resultordner) {

    PlotFlow::plot(m_dataset, resultordner);

}
