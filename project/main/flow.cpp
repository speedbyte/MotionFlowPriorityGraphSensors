#include <vector>
#include <cmath>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv/cv.hpp>
#include <map>
#include <chrono>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/tuple/tuple.hpp>
#include <gnuplot-iostream/gnuplot-iostream.h>
#include <png++/png.hpp>
#include <kitti/io_flow.h>
#include "datasets.h"

//Standart settings from functions
//automotive plot



//Creating a movement path. The path is stored in a x and y vector

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
void prepare_directories_result_flow(const boost::filesystem::path dataset_path, const std::string unterordner) {

    boost::filesystem::path result_dir_path = dataset_path;
    result_dir_path = dataset_path;
    result_dir_path += unterordner;

    if ( boost::filesystem::exists(result_dir_path) ) {
        system(("rm " + result_dir_path.string() + std::string("/*")).c_str());
    }
    boost::filesystem::create_directories(result_dir_path.string());
}


void calculate_flow(const boost::filesystem::path dataset_path, const std::string unterordner, const std::string
input_image_folder, FRAME_TYPES frame_types, NOISE_TYPES noise ) {

    prepare_directories_result_flow(dataset_path, unterordner);
    
    std::cout << "results will be stored in " << unterordner << std::endl;


    char file_name[50];
    char xFlow[100];
    char yFlow[100];


    std::vector<unsigned> x_pts;
    std::vector<double> y_pts;
    std::vector<unsigned> z_pts;
    std::vector<float> time;
    double sum_time = 0;

    std::vector<boost::tuple<std::vector<unsigned>, std::vector<double>> > pts_exectime;

    bool needToInit = true;
    std::vector<cv::Point2f> prev_pts;
    std::vector<cv::Point2f> next_pts;
    cv::Mat curGray, prevGray;

    boost::filesystem::path image_in_path = dataset_path.string() + std::string("data/stereo_flow/") + 
            input_image_folder + std::string("dummy.txt");
    assert(boost::filesystem::exists(image_in_path.parent_path()) != 0);


    boost::filesystem::path results_flow = dataset_path.string() + unterordner + std::string("dummy.txt");
    assert(boost::filesystem::exists(results_flow.parent_path()) != 0);

    if ( frame_types == video_frames) {
        cv::VideoCapture cap;
        cap.open(image_in_path.string());
        if (!cap.isOpened()) {
            std::cout << "Could not initialize capturing...\n";
            return;
        }
    }
    std::string results_flow_matrix_str = results_flow.parent_path().string() + "/result_flow.yaml";

    cv::FileStorage fs;
    fs.open(results_flow_matrix_str, cv::FileStorage::WRITE);


    cv::Size_<unsigned> frame_size(1242,375);
    cv::VideoWriter video_out;

    if ( frame_types == video_frames)
    {
        boost::filesystem::path video_out_path = dataset_path.string() + unterordner + std::string("/video/OpticalFlow.avi");
        assert(boost::filesystem::exists(video_out_path.parent_path()) != 0);
        //frame_size.height =	(unsigned) cap.get(CV_CAP_PROP_FRAME_HEIGHT );
        //frame_size.width =	(unsigned) cap.get(CV_CAP_PROP_FRAME_WIDTH );
        video_out.open(video_out_path.string(),CV_FOURCC('D','I','V','X'), 5, frame_size);
        printf("Writer eingerichtet\n");
    }

    cv::Mat frame = cv::Mat::zeros(frame_size, CV_8UC3);
    cv::Mat flowImage(cv::Size(1242,375), CV_32FC3, cv::Scalar(255,255,255));

    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
    cv::Size subPixWinSize(10, 10), winSize(21, 21);

    const int MAX_COUNT = 5000;
    cv::Mat pyramid1, pyramid2;

    pyramid1.create(frame_size, CV_8UC1);
    pyramid2.create(frame_size, CV_8UC1);

    ushort frame_count = 0;

    cv::namedWindow(unterordner, CV_WINDOW_AUTOSIZE);

    //how many interations(frames)?
    auto tic = steady_clock::now();
    auto toc = steady_clock::now();

    ushort collision = 0, iterator = 0, sIterator = 0;
    std::vector<ushort> xPos, yPos;


    cv::Mat flow_frame( frame_size, CV_8UC3 );

    std::map<std::string, double> time_map = {{"generate",0},
                                              {"ground truth", 0},
                                              {"FB", 0},
                                              {"LK", 0},
                                              {"movement", 0},
                                              {"collision", 0},
    };

    bool plotTime = 1;
    std::vector<bool> error(2);
    error.at(0) = 0;
    error.at(1) = 0;

    std::string gt_image_path_str;
    std::string results_flow_path_str;

    while ( true ) {

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
                next_pts.clear();
                break;
            default:
                break;
        }

        //cap >> frame;
        //if (frame.empty())
        //    break;

        fs << "frame_count" << frame_count;

        sprintf(file_name, "000%03d_10", frame_count);
        gt_image_path_str = image_in_path.parent_path().string() + "/" + std::string(file_name) + ".png";
        frame = cv::imread(gt_image_path_str, CV_LOAD_IMAGE_COLOR);

        results_flow_path_str =
                results_flow.parent_path().string() + "/" + std::string(file_name) + ".png";

        // Convert to grayscale
        cv::cvtColor(frame, curGray, cv::COLOR_BGR2GRAY);

        //printf("%u, %u , %u, %u, %u\n", x, start, iterator, secondstart, sIterator);

        if (!unterordner.compare("results/FB")) {
            tic = steady_clock::now();
            if (prevGray.data) {
                // Initialize parameters for the optical calculate_flow algorithm
                float pyrScale = 0.5;
                int numLevels = 3;
                int windowSize = 15;
                int numIterations = 3;
                int neighborhoodSize = 5;
                float stdDeviation = 1.2;

                // Calculate optical calculate_flow map using Farneback algorithm
                cv::calcOpticalFlowFarneback(prevGray, curGray, flow_frame, pyrScale, numLevels, windowSize,
                                             numIterations,
                                             neighborhoodSize, stdDeviation, cv::OPTFLOW_USE_INITIAL_FLOW);

                // Draw the optical calculate_flow map
                int stepSize = 16;

                // Draw the uniform grid of points on the input image along with the motion vectors
                for (int y = 0; y < frame.rows; y += stepSize) {
                    for (int x = 0; x < frame.cols; x += stepSize) {
                        // Circles to indicate the uniform grid of points
                        cv::circle(frame, cv::Point(x, y), 1, cv::Scalar(0, 0, 0), -1, 8);

                        // Lines to indicate the motion vectors
                        cv::Point2f pt = flow_frame.at<cv::Point2f>(y, x);
                        cv::arrowedLine(frame, cv::Point(x, y), cv::Point(cvRound(x + pt.x), cvRound(y + pt.y)),
                                        cv::Scalar(0,
                                                   255, 0));

                        int row_coordinate = (int)(y);
                        int col_coordinate = (int)(x);
                        float Vx = (cvRound(x+pt.x) - x);
                        float Vy = (cvRound(y+pt.y) - y);
                        if ( Vx != 0 && Vy!= 0) {
                            printf("(iteration %u, coordinates x y (%i,%i) ->  Vx, Vy (%f,%f) \n", frame_count,
                                   row_coordinate, col_coordinate, Vx, Vy);
                            flowImage.at<cv::Vec3f>(row_coordinate, col_coordinate)[0] = cvRound(Vx+0.5);
                            flowImage.at<cv::Vec3f>(row_coordinate, col_coordinate)[1] = cvRound(Vy+0.5);
                            flowImage.at<cv::Vec3f>(row_coordinate, col_coordinate)[2] = 1.0f;
                        }
                    }
                }
            }
            toc = steady_clock::now();
            time_map["FB"] = duration_cast<milliseconds>(toc - tic).count();
            y_pts.push_back(time_map["FB"]);
        }

        else if (!unterordner.compare("results/LK")) {
            tic = steady_clock::now();
            // Calculate optical calculate_flow map using LK algorithm
            if (prevGray.data) {  // Calculate only on second or subsequent images.
                std::vector<uchar> status;
                std::vector<float> err;
                /*if (prevGray.empty()) {
                    curGray.copyTo(prevGray);
                }*/
                cv::calcOpticalFlowPyrLK(prevGray, curGray, prev_pts, next_pts, status,
                                         err, winSize, 5, termcrit, 0, 0.001);

                unsigned count = 0;
                int minDist = 0;

                std::ofstream flowX;
                flowX.open(xFlow);

                std::ofstream flowY;
                flowY.open(yFlow);
                for (unsigned i = 0; i < next_pts.size(); i++) {
                    /* If the new point is within 'minDist' distance from an existing point, it will not be tracked */
                    if (cv::norm(prev_pts[i] - next_pts[i]) <= minDist) {
                        //printf("minimum distance for %i\n", i);
                        continue;
                    }

                    if(noise == 0) {
                        sprintf(xFlow, "../../FlowTextFiles/Slow/no_noise/FlowX/x%03d.txt", frame_count);
                        sprintf(yFlow, "../../FlowTextFiles/Slow/no_noise/FlowY/y%03d.txt", frame_count);
                    }

                    if(noise == 1){
                        sprintf(xFlow, "../../FlowTextFiles/Slow/static_BG/FlowX/x%03d.txt", frame_count);
                        sprintf(yFlow, "../../FlowTextFiles/Slow/static_BG/FlowY/y%03d.txt", frame_count);
                    }

                    if(noise == 2){
                        sprintf(xFlow, "../../FlowTextFiles/Slow/static_FG/FlowX/x%03d.txt", frame_count);
                        sprintf(yFlow, "../../FlowTextFiles/Slow/static_FG/FlowY/y%03d.txt", frame_count);
                    }

                    if(noise == 3){
                        sprintf(xFlow, "../../FlowTextFiles/Slow/dynamic_BG/FlowX/x%03d.txt", frame_count);
                        sprintf(yFlow, "../../FlowTextFiles/Slow/dynamic_BG/FlowY/y%03d.txt", frame_count);
                    }

                    if(noise == 4){
                        sprintf(xFlow, "../../FlowTextFiles/Slow/dynamic_FG/FlowX/x%03d.txt", frame_count);
                        sprintf(yFlow, "../../FlowTextFiles/Slow/dynamic_FG/FlowY/y%03d.txt", frame_count);
                    }



                    for (int k = 0; k < prev_pts.size(); k++) {
                        flowX << (int) prev_pts[k].x << " " << next_pts[k].x - prev_pts[k].x << std::endl;
                        flowY << (int) prev_pts[k].y << " " << next_pts[k].y - prev_pts[k].y << std::endl;
                    }
                    // Check if the status vector is good
                    if (!status[i])
                        continue;

                    next_pts[count++] = next_pts[i];
                    //cv::circle(frame, next_pts[count], 3, cv::Scalar(0, 255, 0), -1, 8);
                    cv::arrowedLine(frame, prev_pts[i], next_pts[i], cv::Scalar(0, 255, 0), 1, CV_AA, 0);

                    int row_coordinate = std::abs(cvRound(next_pts[i].y));
                    int col_coordinate = std::abs(cvRound(next_pts[i].x));
                    float Vx = (next_pts[i].x - prev_pts[i].x);
                    float Vy = (next_pts[i].y - prev_pts[i].y);
                   // printf("(iteration %u, x y (%i,%i) -> ( Vx, Vy)(%f,%f) \n", frame_count, row_coordinate,
                   //        col_coordinate, Vx, Vy);
                    if(row_coordinate < 375 && col_coordinate < 1242) {
                        flowImage.at<cv::Vec3f>(row_coordinate, col_coordinate)[0] = cvRound(Vx);
                        flowImage.at<cv::Vec3f>(row_coordinate, col_coordinate)[1] = cvRound(Vy);
                        flowImage.at<cv::Vec3f>(row_coordinate, col_coordinate)[2] = 1.0f;
                    }
                }
                next_pts.resize(count);
                //printf(" new size is %i for frame number %u\n", count, frame_count);
                z_pts.push_back(count);
            }
            else {
                needToInit = true;
            }
            if (needToInit) {
                // automatic initialization
                cv::goodFeaturesToTrack(curGray, next_pts, MAX_COUNT, 0.01, 10, cv::Mat(), 3, false, 0.04);
                // Refining the location of the feature points
                assert(next_pts.size() <= MAX_COUNT );
                std::vector<cv::Point2f> currentPoint;
                std::swap(currentPoint, next_pts);
                next_pts.clear();
                for (unsigned i = 0; i < currentPoint.size(); i++) {
                    std::vector<cv::Point2f> tempPoints;
                    tempPoints.push_back(currentPoint[i]);
                    // Function to refine the location of the corners to subpixel accuracy.
                    // Here, 'pixel' refers to the image patch of size 'windowSize' and not the actual image pixel
                    cv::cornerSubPix(curGray, tempPoints, subPixWinSize, cv::Size(-1, -1), termcrit);
                    next_pts.push_back(tempPoints[0]);
                }
                printf("old next_pts size is %d and new next_pts size is %d\n", currentPoint.size(), next_pts.size());
                //std::swap(currentPoint, next_pts);
            }

            toc = steady_clock::now();
            time_map["LK"] = duration_cast<milliseconds>(toc - tic).count();
            y_pts.push_back(time_map["LK"]);
            time.push_back(duration_cast<milliseconds>(toc - tic).count());
        }

        if (prevGray.data) {
            //Create png Matrix with 3 channels: x displacement. y displacment and Validation bit
            FlowImage F_result_write(frame_size.width, frame_size.height);
            for (int32_t v=0; v<frame_size.height; v++) { // rows
                for (int32_t u=0; u<frame_size.width; u++) {  // cols
                    if (flowImage.at<cv::Vec3f>(v,u)[2] > 0.5 ) {
                        F_result_write.setFlowU(u,v,flowImage.at<cv::Vec3f>(v,u)[0]);
                        F_result_write.setFlowV(u,v,flowImage.at<cv::Vec3f>(v,u)[1]);
                        F_result_write.setValid(u,v,(bool)flowImage.at<cv::Vec3f>(v,u)[2]);
                    }
                }
            }
            F_result_write.write(results_flow_path_str);

            FlowImage F_result_read;
            F_result_read.read(results_flow_path_str);

            for (int32_t v=0; v<F_result_read.height(); v++) { // rows
                for (int32_t u=0; u<F_result_read.width(); u++) {  // cols
                    if ( F_result_read.isValid(u,v) ) {
                        fs << "png file read" << "[";
                        fs << "{:" << "row" <<  v << "col" << u << "displacement" << "[:";
                        fs << F_result_read.getFlowU(u,v);
                        fs << F_result_read.getFlowV(u,v);
                        fs << F_result_read.isValid(u,v);
                        fs << "]" << "}";
                        fs << "]";
                    }
                }
            }
        }
        

        // Scratch 2.. begin .. end
        // Scratch 2 end

        tic = steady_clock::now();

        auto end = steady_clock::now();

        x_pts.push_back(frame_count);


        if ( frame_types == video_frames) {
            video_out.write(frame);
        }

        // Display the output image
        cv::imshow(unterordner, frame);
        needToInit = false;
        prev_pts.clear();
        std::swap(next_pts, prev_pts);
        std::swap(prevGray, curGray);
        frame_count++;
        if ( frame_count == MAX_ITERATION ) {
            break;
        }
    }

    for(auto &n : time)
        sum_time +=n;

    std::cout << "Noise " << noise  << "Zeit " << sum_time << std::endl;
    std::cout << "time_map LK " << time_map["LK"] << std::endl;

    fs.release();
    auto max = (std::max_element(y_pts.begin(), y_pts.end())).operator*();

    pts_exectime.push_back(boost::make_tuple(x_pts, y_pts));
    if ( frame_types == video_frames) {
        video_out.release();
    }
    cv::destroyAllWindows();

    // gnuplot_2d
    Gnuplot gp2d;
    gp2d << "set xrange [0:" + std::to_string(MAX_ITERATION) + "]\n";
    gp2d << "set yrange [0:" + std::to_string(max*2) + "]\n";
    std::string tmp = std::string(" with lines title ") + std::string("'") + input_image_folder + std::string(" y axis - ms, x axis - frame\n'");
    gp2d << "plot" << gp2d.binFile2d(pts_exectime, "record") << tmp;

}
