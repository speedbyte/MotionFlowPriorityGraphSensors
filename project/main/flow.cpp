#include <vector>
#include <cmath>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv/cv.hpp>
#include <map>
#include <chrono>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/tuple/tuple.hpp>
#include <gnuplot/gnuplot-iostream.h>
#include <png++/png.hpp>

//Creating a movement path. The path is stored in a x and y vector

using namespace std::chrono;

#define MATLAB_DATASET_PATH "../../../matlab_dataset/"
#define CPP_DATASET_PATH "../../../cpp_dataset/"


void flow(std::string algo, ushort start, ushort secondstart) {


    const boost::filesystem::path dataset_path = CPP_DATASET_PATH;

    boost::filesystem::path name_frame, name_flow;
    name_frame = dataset_path.string() + std::string("data/stereo_flow/flow_noc/dummy.tx5");
    name_flow = dataset_path.string() + std::string("flow/dummy.txt");
    assert(boost::filesystem::exists(name_frame.parent_path()) != 0);
    assert(boost::filesystem::exists(name_flow.parent_path()) != 0);

    std::vector<unsigned> x_pts;
    std::vector<double> y_pts;
    std::vector<unsigned> z_pts;
    std::vector<boost::tuple<std::vector<unsigned>, std::vector<double>> > pts_exectime;

    bool needToInit = true;
    std::vector<cv::Point2f> prev_pts;
    std::vector<cv::Point2f> next_pts;
    cv::Mat curGray, prevGray;
    cv::VideoCapture cap;
    boost::filesystem::path video_in_path = dataset_path.string() + std::string("ground_truth/gtMovement.avi");
    assert(boost::filesystem::exists(video_in_path.parent_path()) != 0);
    boost::filesystem::path video_out_path = dataset_path.string() + std::string("flow/OpticalFlow_"+algo+".avi");

    std::cout << video_in_path.string() << std::endl;
    cap.open(video_in_path.string());
    if (!cap.isOpened()) {
        std::cout << "Could not initialize capturing...\n";
        return;
    }

    name_frame = std::string(MATLAB_DATASET_PATH) + std::string("frames/") + std::string("dummy.txt");
    name_flow = std::string(MATLAB_DATASET_PATH) + std::string("flow/") + std::string("dummy.txt");
    assert(boost::filesystem::exists(name_frame.parent_path()) != 0);
    assert(boost::filesystem::exists(name_flow.parent_path()) != 0);


    cv::Size_<unsigned> frame_size;
    cv::VideoWriter video_out;
    frame_size.height =	(unsigned) cap.get(CV_CAP_PROP_FRAME_HEIGHT );
    frame_size.width =	(unsigned) cap.get(CV_CAP_PROP_FRAME_WIDTH );
    video_out.open(video_out_path.string(),CV_FOURCC('D','I','V','X'), 5, frame_size);
    printf("Writer eingerichtet\n");

    cv::Mat frame = cv::Mat::zeros(frame_size, CV_8UC3);
    cv::Mat flowImage(frame_size, CV_32FC3, cv::Scalar(255,255,255));

    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
    cv::Size subPixWinSize(10, 10), winSize(31, 31);

    const int MAX_COUNT = 500;
    cv::Mat pyramid1, pyramid2;

    pyramid1.create(frame_size, CV_8UC1);
    pyramid2.create(frame_size, CV_8UC1);

    unsigned frame_count = 0;

    cv::namedWindow(algo, CV_WINDOW_AUTOSIZE);

    //how many interations(frames)?
    auto tic = steady_clock::now();
    auto toc = steady_clock::now();

    const ushort MAX_ITERATION = 360;
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


    while ( true ) {

        flowImage = cv::Scalar(0,0,0);
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
        cap >> frame;
        if (frame.empty())
            break;
        frame_count++;

        // Convert to grayscale
        cv::cvtColor(frame, curGray, cv::COLOR_BGR2GRAY);


        //printf("%u, %u , %u, %u, %u\n", x, start, iterator, secondstart, sIterator);


        cap >> frame;
        if (frame.empty())
            break;
        frame_count++;


        cv::cvtColor(frame, curGray, CV_BGR2GRAY);

        if (algo.compare("FB") == 0) {
            tic = steady_clock::now();
            if (prevGray.data) {
                // Initialize parameters for the optical flow algorithm
                float pyrScale = 0.5;
                int numLevels = 3;
                int windowSize = 15;
                int numIterations = 3;
                int neighborhoodSize = 5;
                float stdDeviation = 1.2;

                // Calculate optical flow map using Farneback algorithm
                cv::calcOpticalFlowFarneback(prevGray, curGray, flow_frame, pyrScale, numLevels, windowSize,
                                             numIterations,
                                             neighborhoodSize, stdDeviation, cv::OPTFLOW_USE_INITIAL_FLOW);

                // Draw the optical flow map
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
                    }
                }
            }
            toc = steady_clock::now();
            time_map["FB"] = duration_cast<milliseconds>(toc - tic).count();
            y_pts.push_back(time_map["FB"]);
        }

        else if (algo.compare("LK") == 0) {
            tic = steady_clock::now();
            // Calculate optical flow map using LK algorithm
            if (needToInit) {
                // automatic initialization
                cv::goodFeaturesToTrack(curGray, next_pts, MAX_COUNT, 0.01, 10, cv::Mat(), 3, false, 0.04);
                // Refining the location of the feature points
                if (next_pts.size() < MAX_COUNT) {
                    std::vector<cv::Point2f> currentPoint;
                    std::swap(currentPoint, next_pts);
                    for (unsigned i = 0; i < currentPoint.size(); i++) {
                        std::vector<cv::Point2f> tempPoints;
                        tempPoints.push_back(currentPoint[i]);
                        // Function to refine the location of the corners to subpixel accuracy.
                        // Here, 'pixel' refers to the image patch of size 'windowSize' and not the actual image pixel
                        cv::cornerSubPix(curGray, tempPoints, subPixWinSize, cv::Size(-1, -1), termcrit);
                        next_pts.push_back(tempPoints[0]);
                    }
                    std::swap(currentPoint, next_pts);
                }
            }

            if (!prev_pts.empty()) {
                std::vector<uchar> status;
                std::vector<float> err;
                if (prevGray.empty()) {
                    curGray.copyTo(prevGray);
                }
                cv::calcOpticalFlowPyrLK(prevGray, curGray, prev_pts, next_pts, status,
                                         err, winSize, 3, termcrit, 0, 0.001);

                unsigned count = 0;
                int minDist = 0;

                for (unsigned i = 0; i < next_pts.size(); i++) {
                    /* If the new point is within 'minDist' distance from an existing point, it will not be tracked */
                    if (cv::norm(prev_pts[i] - next_pts[i]) <= minDist) {
                        //printf("minimum distance for %i\n", i);
                        continue;
                    }

                    // Check if the status vector is good
                    if (!status[i])
                        continue;

                    next_pts[count++] = next_pts[i];
                    //cv::circle(frame, next_pts[count], 3, cv::Scalar(0, 255, 0), -1, 8);
                    cv::arrowedLine(frame, prev_pts[i], next_pts[i], cv::Scalar(0, 255, 0), 1, CV_AA, 0);
                    int row_coordinate = (int)(prev_pts[i].y);
                    int col_coordinate = (int)(prev_pts[i].x);
                    float Vx = (next_pts[i].x - prev_pts[i].x);
                    float Vy = (next_pts[i].y - prev_pts[i].y);
                    printf("(iteration %u, x y (%i,%i) -> ( Vx, Vy)(%f,%f) \n", frame_count, row_coordinate,
                           col_coordinate, Vx, Vy);
                    flowImage.at<cv::Vec3f>(row_coordinate, col_coordinate)[0] = Vx;
                    flowImage.at<cv::Vec3f>(row_coordinate, col_coordinate)[1] = Vy;
                    flowImage.at<cv::Vec3f>(row_coordinate, col_coordinate)[2] = 1.0f;
                }
                next_pts.resize(count);
                //printf(" new size is %i for frame number %u\n", count, frame_count);
                z_pts.push_back(count);
            }
            toc = steady_clock::now();
            time_map["LK"] = duration_cast<milliseconds>(toc - tic).count();
            y_pts.push_back(time_map["LK"]);
            char file_name[50];
            sprintf(file_name, "0000000%03d.png", frame_count);
            std::string flow_image_path = name_frame.parent_path().string() + '/' + std::string(file_name);

            png::image< png::rgb_pixel_16 > image(frame_size.width,frame_size.height);
            for (int32_t v=0; v<frame_size.height; v++) {
                for (int32_t u=0; u<frame_size.width; u++) {
                    png::rgb_pixel_16 val;
                    val.red   = 0;
                    val.green = 0;
                    val.blue  = 0;
                    if (flowImage.at<cv::Vec3f>(v,u)[2] < 0.5) {
                        val.red   = (uint16_t)std::max(std::min((flowImage.at<cv::Vec3f>(v,u)[0])*64.0f+32768.0f,
                                                                65535.0f),0.0f);
                        val.green = (uint16_t)std::max(std::min((flowImage.at<cv::Vec3f>(v,u)[1])*64.0f+32768.0f,
                                                                65535.0f),0.0f);
                        val.blue  = 1;
                    }
                    image.set_pixel(u,v,val);
                }
            }
            image.write(flow_image_path);
            //cv::imwrite(flow_image_path, flowImage);
        }

        // Scratch 2.. begin .. end

        // Scratch 2 end

        tic = steady_clock::now();

        auto end = steady_clock::now();

        x_pts.push_back(frame_count);

        video_out.write(frame);
        // Display the output image
        cv::imshow(algo, frame);
        needToInit = false;
        prev_pts.clear();
        std::swap(next_pts, prev_pts);
        std::swap(prevGray, curGray);
    }

    auto max = (std::max_element(y_pts.begin(), y_pts.end())).operator*();


    pts_exectime.push_back(boost::make_tuple(x_pts, y_pts));
    video_out.release();
    cv::destroyAllWindows();

    // gnuplot_2d
    Gnuplot gp2d;
    gp2d << "set xrange [0:360]\n";
    gp2d << "set yrange [0:" + std::to_string(max*2) + "]\n";
    gp2d << "plot" << gp2d.binFile2d(pts_exectime, "record") << " with lines title 'y axis - ms, x axis - frame "
            "count'\n";

}
