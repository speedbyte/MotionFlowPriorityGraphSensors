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

//Creating a movement path. The path is stored in a x and y vector

using namespace std::chrono;

#define MATLAB_DATASET_PATH "../../../matlab_dataset/"

void flow(std::string algo) {

    const boost::filesystem::path dataset_path = MATLAB_DATASET_PATH;

    boost::filesystem::path name_frame, name_flow;
    name_frame = dataset_path.string() + std::string("frames/dummy.tx5");
    name_flow = dataset_path.string() + std::string("flow/.dummy.txt");
    assert(boost::filesystem::exists(name_frame.parent_path()) != 0);
    assert(boost::filesystem::exists(name_flow.parent_path()) != 0);

    std::vector<unsigned> x_pts;
    std::vector<double> y_pts;
    std::vector<unsigned> z_pts;
    std::vector<boost::tuple<std::vector<unsigned>, std::vector<double>> > pts_exectime;

    bool needToInit = true;
    std::vector<cv::Point2f> prev_pts;
    std::vector<cv::Point2f> next_pts;
    cv::Mat flowImage;
    cv::Mat curGray, prevGray;
    cv::VideoCapture cap;
    boost::filesystem::path video_path = dataset_path.string() + std::string("flow/OpticalFlow_"+algo+".avi");
    assert(boost::filesystem::exists(video_path.parent_path()) != 0);

    std::cout << video_path.string() << std::endl;
    cap.open(video_path.string());
    if (!cap.isOpened()) {
        std::cout << "Could not initialize capturing...\n";
        return;
    }

    cv::VideoWriter video_out;  // OpticalFlow_FB.avi und OpticalFlow_LK.avi

    cv::Size frame_size;
    frame_size.height =	(int) cap.get(CV_CAP_PROP_FRAME_HEIGHT );
    frame_size.width =	(int) cap.get(CV_CAP_PROP_FRAME_WIDTH );
    video_out.open(video_path.string(),CV_FOURCC('D','I','V','X'), 5, frame_size);
    printf("Writer eingerichtet\n");


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

    //ushort xMovement=0,yMovement=0,secondXMovement=0,secondYMovement=0;

    ushort actualX;
    ushort actualY;
    ushort secondActualX;
    ushort secondActualY;

    //object specs
    const ushort width = 30;
    const ushort height = 100;

    //Start is somewhere on the path
    ushort start = 30;
    //ushort xOrigin = xPos.at(start);
    //ushort yOrigin = yPos.at(start);

    //Start of the second object is somewhere on the path
    ushort secondStart = 200;
    //ushort secondXOrigin = xPos.at(secondStart);
    //ushort secondYOrigin = yPos.at(secondStart);

    //for moving the objects later
    //actualX = xOrigin;
    //actualY = yOrigin;
    //secondActualX = secondXOrigin;
    //secondActualY = secondYOrigin;

    //Initialization
    //Ground Truth Movement (First Object x, first Object y, second object x, second object y movement
    iterator = 0;
    sIterator = 0;
    //xMovement = 0;
    //secondXMovement = 0;
    //yMovement = 0;
    //secondYMovement = 0;

    cv::Mat absoluteFlow = cv::Mat::zeros(375, 1242, CV_16UC3);
    cv::Mat flow1 = cv::Mat::zeros(375, 1242, CV_16UC3);
    cv::Mat flow2 = cv::Mat::zeros(375, 1242, CV_16UC3);
    cv::Mat frame = cv::Mat::zeros(375, 1242, CV_8UC3);

    cv::Mat flow_frame(cv::Size(1242,375),CV_8UC3);
    std::vector<bool> estimatedCollisionVector(MAX_ITERATION);

    std::map<std::string, double> time_map;

    bool plotTime = 1;
    std::vector<bool> error;
    error.at(0) = 0;
    error.at(1) = 0;


    for (ushort x=0; x <= MAX_ITERATION; x++) {

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

        // Resize the frame
        //cv::resize(frame, frame, cv::Size(), scalingFactor, scalingFactor, cv::INTER_AREA);

        // Convert to grayscale
        cv::cvtColor(frame, curGray, cv::COLOR_BGR2GRAY);

        auto tic = steady_clock::now();
        // Check if the image is valid

        //Used to store the GT images for the kitti devkit

        char file_name[20];
        sprintf(file_name, "0000000%03d.png ", x);

        name_frame = std::string(MATLAB_DATASET_PATH) + std::string("frames/") + std::string(file_name);
        name_flow = std::string(MATLAB_DATASET_PATH) + std::string("flow/") + std::string(file_name);
        assert(boost::filesystem::exists(name_frame.parent_path()) != 0);
        assert(boost::filesystem::exists(name_flow.parent_path()) != 0);

        printf("%u, %u , %u, %u, %u\n", x, start, iterator, secondStart, sIterator);

        //end of path vector? reset
        if ((iterator + start) >= xPos.size()) {
            start = 0;
            iterator = 0;
        }

        if ((sIterator + secondStart) >= xPos.size()) {
            secondStart = 0;
            sIterator = 0;
        }

        //Object specification
        std::vector<ushort> xSpec;
        for (ushort i = 0; i < width; i++) {
            xSpec.push_back(actualX + i);
        }
        std::vector<ushort> ySpec;
        for (ushort i = 0; i < height; i++) {
            ySpec.push_back(actualY + i);
        }

        std::vector<ushort> secondXSpec;
        for (ushort i = 0; i < width; i++) {
            secondXSpec.push_back(secondActualX + i);
        }

        std::vector<ushort> secondYSpec;
        for (ushort i = 0; i < height; i++) {
            secondYSpec.push_back(secondActualY + i);
        }


        cap >> frame;
        if (frame.empty())
            break;
        frame_count++;

        // Resize the frame
        //cv::resize(frame, frame, cv::Size(), scalingFactor, scalingFactor, cv::INTER_AREA);


        tic = steady_clock::now();

        cv::cvtColor(frame, curGray, CV_BGR2GRAY);

        if (algo.compare("FB") == 0) {
            if (prevGray.data) {
                // Initialize parameters for the optical flow algorithm
                float pyrScale = 0.5;
                int numLevels = 3;
                int windowSize = 15;
                int numIterations = 3;
                int neighborhoodSize = 5;
                float stdDeviation = 1.2;

                //flow_frame = cv::calcOpticalFlowFarneback(opticFlow, curGray);
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
                        cv::circle(frame, cv::Point(x, y), 0.5, cv::Scalar(0, 255, 0), -1, 8);

                        // Lines to indicate the motion vectors
                        cv::Point2f pt = flow_frame.at<cv::Point2f>(y, x);
                        cv::arrowedLine(frame, cv::Point(x, y), cv::Point(cvRound(x + pt.x), cvRound(y + pt.y)),
                                        cv::Scalar(0,
                                                   255, 0));
                    }
                }
            }
        }

        else if (algo.compare("LK") == 0) {
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
                }
                next_pts.resize(count);
                printf(" new size is %i for frame number %u\n", count, frame_count);
                z_pts.push_back(count);
            }
        }


        toc = steady_clock::now();
        time_map["flow"] = duration_cast<milliseconds>(toc - tic).count();


        cv::Mat vxCopy, vyCopy, vXYCopy, vCopy, flow_frame_individual_channels[3];
        cv::split(flow_frame, flow_frame_individual_channels);
        //flow_frame.Vx ~ = 0); // flow_frame != 0 ? 1 : 0
        cv::threshold (flow_frame_individual_channels[0], vxCopy, 0, 1, cv::THRESH_BINARY);
        cv::threshold (flow_frame_individual_channels[1], vyCopy, 0, 1, cv::THRESH_BINARY);
        vXYCopy = vxCopy + vyCopy;
        cv::threshold (vXYCopy, vCopy, 0, 1, cv::THRESH_BINARY);

        cv::Mat res;

        cv::Mat flow;
        flow_frame_individual_channels->copyTo(flow);

        //channel copy !!!!!!
        flow_frame_individual_channels[0] = (flow_frame_individual_channels[0] * 64 ) + 2 ^ 15; // Vx
        flow_frame_individual_channels[1] = (flow_frame_individual_channels[1] * 64 ) + 2 ^ 15; // Vy
        flow_frame_individual_channels[2] = vCopy;

        // merge in a image file
        cv::merge(flow_frame_individual_channels,3,res);
        cv::imwrite("result.png", res);

        //create flow matrix to store the estimated displacemend in.

        //absolute Flow - get absolute estimated flow.

        tic = steady_clock::now();

        bool estimatedCollision;
        std::vector<float> estMovement(4); // xMean,yMean,secondObjectXMean,secondObjectYMean


        //Threshold of the object flow
        estimatedCollision = 0;
        cv::Mat flowFirstObjectX = cv::Mat::zeros(375,1242, CV_16UC1);
        cv::Mat flowFirstObjectY = cv::Mat::zeros(375,1242,CV_16UC1);
        cv::Mat flowSecondObjectX = cv::Mat::zeros(375,1242,CV_16UC1);
        cv::Mat flowSecondObjectY = cv::Mat::zeros(375,1242,CV_16UC1);

        ushort upperheigtht = ySpec.at(ySpec.size()-1);
        ushort lowerheight = ySpec.at(0);
        ushort upperwidth = xSpec.at(ySpec.size()-1);
        ushort lowerwidth = xSpec.at(0);

        ushort secondObjectUpperheight = secondYSpec.at(ySpec.size()-1);
        ushort secondObjectLowerheight = secondYSpec.at(0);
        ushort secondObjectUpperWidth = secondXSpec.at(ySpec.size()-1);
        ushort secondObjectLowerWidth = secondXSpec.at(0);


        for ( int k = ySpec.at(0); k < ySpec.at(ySpec.size()-1); k++ )  {
            for ( int j = xSpec.at(0); j < xSpec.at(xSpec.size()-1); j++ )  {
                flow1.at<cv::Vec3i>(k, j)[0] = absoluteFlow.at<cv::Vec3i>(ySpec.at(k), xSpec.at(j))[0];
                flow1.at<cv::Vec3i>(k, j)[1] = absoluteFlow.at<cv::Vec3i>(ySpec.at(k), xSpec.at(j))[1];
            }
        }

        for ( int k = secondYSpec.at(0); k < secondYSpec.at(secondYSpec.size()-1); k++ )  {
            for ( int j = secondXSpec.at(0); j < secondXSpec.at(secondXSpec.size()-1); j++ )  {
                flow2.at<cv::Vec3i>(k, j)[0] = absoluteFlow.at<cv::Vec3i>(secondYSpec.at(k), secondXSpec.at(j))[0];
                flow2.at<cv::Vec3i>(k, j)[1] = absoluteFlow.at<cv::Vec3i>(secondYSpec.at(k), secondXSpec.at(j))[1];
            }
        }

        //%
        //%get flow from the objects
        for ( ushort k = lowerheight; k < upperheigtht; k++ ) {
            for ( ushort j = lowerwidth; j < upperwidth; j++ ) {
                flowFirstObjectX.at<ushort>(k,j) = flow.at<cv::Vec3i>(k,j)[0];
                flowFirstObjectY.at<ushort>(k,j) = flow.at<cv::Vec3i>(k,j)[1];
            }
        }

        for ( ushort kk = secondObjectLowerheight; kk < secondObjectUpperheight; kk++ ) {
            for ( ushort jj = secondObjectLowerWidth; jj < secondObjectUpperWidth; jj++ ) {
                flowSecondObjectX.at<ushort>(kk,jj) = flow.at<cv::Vec3i>(kk,jj)[0];
                flowSecondObjectY.at<ushort>(kk,jj) = flow.at<cv::Vec3i>(kk,jj)[1];
            }
        }

        //%
        //Extract the movement of the object.
        cv::Mat firstObjectX, firstObjectY, secondObjectX, secondObjectY;

        for ( ushort i = 0; i < flowFirstObjectX.rows; i++ ) {
            for (ushort j = 0; j < flowFirstObjectX.cols; i++) {
                if ( flowFirstObjectX.at<ushort>(i,j) != 0 || cv::abs(flowFirstObjectX.at<ushort>(i,j) < 0.2)) {
                    firstObjectX.push_back(flowFirstObjectX.at<ushort>(i,j));
                }
            }
        }

        for ( ushort i = 0; i < flowFirstObjectY.rows; i++ ) {
            for (ushort j = 0; j < flowFirstObjectY.cols; i++) {
                if ( flowFirstObjectY.at<ushort>(i,j) != 0 || cv::abs(flowFirstObjectY.at<ushort>(i,j) < 0.2)) {
                    firstObjectY.push_back(flowFirstObjectY.at<ushort>(i,j));
                }
            }
        }

        for ( ushort i = 0; i < flowSecondObjectX.rows; i++ ) {
            for (ushort j = 0; j < flowSecondObjectX.cols; i++) {
                if ( flowSecondObjectX.at<ushort>(i,j) != 0 || cv::abs(flowSecondObjectX.at<ushort>(i,j) < 0.2)) {
                    secondObjectX.push_back(flowSecondObjectX.at<ushort>(i,j));
                }
            }
        }

        for ( ushort i = 0; i < flowSecondObjectY.rows; i++ ) {
            for (ushort j = 0; j < flowSecondObjectY.cols; i++) {
                if ( flowSecondObjectY.at<ushort>(i,j) != 0 || cv::abs(flowSecondObjectY.at<ushort>(i,j) < 0.2)) {
                    secondObjectY.push_back(flowSecondObjectY.at<ushort>(i,j));
                }
            }
        }

        cv::Scalar_<float> xMean, yMean, secondObjectXMean, secondObjectYMean;

        //Get the movement by getting the mean of the flow objects.

        xMean = cv::mean(firstObjectX);
        yMean = cv::mean(firstObjectY);
        secondObjectXMean = cv::mean(secondObjectX);
        secondObjectYMean = cv::mean(secondObjectY);

        std::cout << "Estimated Movement of the First Object:" << std::endl;;
        std::cout << "x" << std::endl;
        std::cout << "xMean" << std::endl;
        std::cout << "y" << std::endl;
        std::cout << "yMean" << std::endl;
        std::cout << "Estimated Movement of the second Object" << std::endl;;
        std::cout << "x" << std::endl;
        std::cout << "secondObjectXMean" << std::endl;
        std::cout << "y" << std::endl;
        std::cout << "secondObjectYMean" << std::endl;

        //Estimate the future collision. Floor call in order to get possibly matching results


        for ( ushort i = 0; i < xSpec.size(); i++ ) {
            xSpec.at(i) = std::floor(xSpec.at(i) + xMean[0]);
            secondXSpec.at(i) = std::floor(secondXSpec.at(i) + secondObjectXMean[0]);
        }

        for ( ushort i = 0; i < ySpec.size(); i++ ) {
            ySpec.at(i) = std::floor(ySpec.at(i) + yMean[0]);
            secondYSpec.at(i) = std::floor(secondYSpec.at(i) + secondObjectYMean[0]);
        }

        std::vector<ushort> checkX, checkY;
        std::vector<ushort> collisionVector(MAX_ITERATION);

        for ( ushort i = 0; i < xSpec.size() ; i++) {
            for ( ushort j = 0; j < secondXSpec.size() ; j++) {
                if ( std::abs(xSpec.at(i) - secondXSpec.at(j)) == 0) {
                    checkX.push_back(i);  // index of collision
                }
            }
        }
        for ( ushort i = 0; i < ySpec.size() ; i++) {
            for ( ushort j = 0; j < secondYSpec.size() ; j++) {
                if ( std::abs(ySpec.at(i) - secondYSpec.at(j)) == 0) {
                    checkY.push_back(i);  // index of collision
                }
            }
        }

        if (!checkX.empty() && !checkY.empty()) {
            collisionVector.at(x) = 1;
        }
        else {
            collisionVector.at(x) = 0;
        }


        estMovement.at(0) = xMean[0];
        estMovement.at(1) = yMean[0];
        estMovement.at(2) = secondObjectXMean[0];
        estMovement.at(3) = secondObjectYMean[0];



        toc = steady_clock::now();

        time_map.insert(std::make_pair("movement", duration_cast<milliseconds>(toc - tic).count()));
        time_map.insert(std::make_pair("collision", duration_cast<milliseconds>(toc - tic).count()));



        for ( int k = 0; k < ySpec.size(); k++ )  {
            for ( int j = 0; j < xSpec.size(); j++ ) {
                absoluteFlow.at<cv::Vec3i>(k, j)[0] = estMovement.at(0) + j;
                absoluteFlow.at<cv::Vec3i>(k, j)[1] = estMovement.at(1) + k;
                absoluteFlow.at<cv::Vec3i>(k, j)[2] = 1;
            }
        }
        for ( int k = 0; k < secondYSpec.size(); k++ )  {
            for ( int j = 0; j < secondXSpec.size(); j++ ) {
                absoluteFlow.at<cv::Vec3i>(k, j)[0] = estMovement.at(2) + j;
                absoluteFlow.at<cv::Vec3i>(k, j)[1] = estMovement.at(3) + k;
                absoluteFlow.at<cv::Vec3i>(k, j)[2] = 1;
            }
        }
        //abs1 = absoluteFlow(:,:,1);


        ////collision checkers(first round has bad flow, dont plot and estimate collision
        //time to check for collision

        if (estimatedCollision == 1)
            estimatedCollisionVector.at(x) = 1;
        else
            estimatedCollisionVector.at(x) = 0;

        tic = steady_clock::now();


        iterator++;
        sIterator++;

        //Update position (the objects of interest are tracked via Ground Truth here)

        actualX = xPos.at(start + iterator);
        actualY = yPos.at(start + iterator);
        secondActualX = xPos.at(secondStart + sIterator);
        secondActualY = yPos.at(secondStart + sIterator);

        cv::imwrite(name_frame.string(), frame );

        auto end = steady_clock::now();

        x_pts.push_back(frame_count);
        y_pts.push_back(duration_cast<milliseconds>(toc - tic).count());

        video_out.write(frame);
        // Display the output image
        cv::imshow(algo, frame);
        needToInit = false;
        prev_pts.clear();
        std::swap(next_pts, prev_pts);
        std::swap(prevGray, curGray);
    }

    pts_exectime.push_back(boost::make_tuple(x_pts, y_pts));
    video_out.release();
    cv::destroyAllWindows();

    // gnuplot_2d
    Gnuplot gp2d;
    gp2d << "set xrange [0:200]\n";
    gp2d << "set yrange [0:100]\n";
    gp2d << "plot" << gp2d.binFile2d(pts_exectime, "record") << " with lines title 'vec of boost::tuple of vec'\n";

}
