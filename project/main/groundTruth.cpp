

// This file converts Initialisation.m, gtColision.m and gtMovement.m and main.m from matlab.
//movement.m and flowCollision.m, estimatedMovment

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

//Creating a movement path. The path is stored in a x and y vector

using namespace std::chrono;



void ground_truth() {

    //how many interations(frames)?
    const unsigned MAX_ITERATION = 360;
    unsigned collision = 0, iterator = 0, sIterator = 0;
    std::vector<unsigned> xPos, yPos;
    unsigned start;
    unsigned secondStart;
    //object specs
    const unsigned width = 30;
    const unsigned height = 100;

    unsigned xMovement=0,yMovement=0,secondXMovement=0,secondYMovement=0;

    unsigned actualX;
    unsigned actualY;
    unsigned secondActualX;
    unsigned secondActualY;


    std::vector<unsigned> theta;
    for ( unsigned x = 0; x <= MAX_ITERATION; x++) {
        theta.push_back(x);
    }

    for ( int i = 0; i< MAX_ITERATION; i++) {
        xPos.push_back((unsigned)(1 * cos(theta[i] * CV_PI / 180.0) / (1.0 + std::pow(sin(theta[i] * CV_PI / 180.0), 2))));
        yPos.push_back((unsigned)(1 * (cos(theta[i] * CV_PI / 180.0) * sin(theta[i] * CV_PI / 180.0)) / (0.2 + std::pow(sin
                                                                                                                     (theta[i] *
                                                                                                                      CV_PI /
                                                                                                                      180.0),
                                                                                                             2))));
    }

    //Start is somewhere on the path
    start = 30;
    unsigned xOrigin = xPos.at(start);
    unsigned yOrigin = yPos.at(start);

    //Start of the second object is somewhere on the path
    secondStart = 200;
    unsigned secondXOrigin = xPos.at(secondStart);
    unsigned secondYOrigin = yPos.at(secondStart);

    //for moving the objects later
    actualX = xOrigin;
    actualY = yOrigin;
    secondActualX = secondXOrigin;
    secondActualY = secondYOrigin;

    cv::Mat kittiGT(375,1242,CV_16UC3,cv::Scalar(0,0,0));
    cv::Mat relativeGroundTruth(375,1242,CV_16UC3,cv::Scalar(0,0,0));
    cv::Mat absoluteGroundTruth(375,1242,CV_16UC3,cv::Scalar(0,0,0));


    for (unsigned x=0; x < MAX_ITERATION; x++) {

        //Used to store the GT images for the kitti devkit

        char name_dense[50];
        sprintf(name_dense, "../../../matlab_dataset/ground_truth/0000000%03d.png", x);

        //Initialization
        if (x == 0) {
            iterator = 0;
            sIterator = 0;
        }

        //Ground Truth Movement (First Object x, first Object y, second object x, second object y movement

        if (x == 0) {
            xMovement = 0;
            secondXMovement = 0;
            yMovement = 0;
            secondYMovement = 0;
        }


        if ((iterator+start) > xPos.size()) {

            xMovement = xPos.at(0) - xPos.at(xPos.size());
            yMovement = yPos.at(0) - yPos.at(yPos.size());

        }

        if ( (sIterator+secondStart) > xPos.size()) {

            secondXMovement = xPos.at(0) - xPos.at(xPos.size());
            secondYMovement = yPos.at(0) - yPos.at(yPos.size());

        }

        if ((iterator+start) <= xPos.size()) {

            xMovement = xPos.at(start+iterator) - xPos.at(start+iterator-1);
            yMovement = yPos.at(start+iterator) - yPos.at(start+iterator-1);

        }

        if ( (sIterator+secondStart) <= xPos.size()) {

            secondXMovement = xPos.at(secondStart+sIterator) - xPos.at(secondStart+sIterator-1);
            secondYMovement = yPos.at(secondStart+sIterator) - yPos.at(secondStart+sIterator-1);

        }

        //If we are at the end of the path vector, we need to reset our iterators
        if ((iterator+start) > xPos.size()) {
            start = 1;
            iterator = 0;
        }

        if ((sIterator+start) > xPos.size()) {
            secondStart = 1;
            sIterator = 0;
        }

        //Object specification
        std::vector<unsigned> xSpec;
        for ( unsigned i = 0; i < width; i++) {
            xSpec.push_back(actualX+i);
        }
        std::vector<unsigned> ySpec;
        for ( unsigned i = 0; i < height; i++) {
            ySpec.push_back(actualY+i);
        }

        std::vector<unsigned> secondXSpec;
        for ( unsigned i = 0; i < width; i++) {
            secondXSpec.push_back(secondActualX+i);
        }

        std::vector<unsigned> secondYSpec;
        for ( unsigned i = 0; i < height; i++) {
            secondYSpec.push_back(secondActualY+i);
        }

        //
        //calculating the relative Ground Truth for the Kitti devkit and store it
        //in a png file

        for ( int k = 0; k < ySpec.size(); k++ )  {
            for ( int j = 0; j < xSpec.size(); j++ )  {
                    relativeGroundTruth.at<cv::Vec3f>(k,j)[0] = (unsigned)(xMovement*64+std::pow(2,15));
                    relativeGroundTruth.at<cv::Vec3f>(k,j)[1] = (unsigned)(yMovement*64+std::pow(2,15));
                    relativeGroundTruth.at<cv::Vec3f>(k,j)[2] = 1;
                    absoluteGroundTruth.at<cv::Vec3f>(k,j)[0] = xMovement+j;
                    absoluteGroundTruth.at<cv::Vec3f>(k,j)[1] = yMovement+k;
                    absoluteGroundTruth.at<cv::Vec3f>(k,j)[2] = 1;

                }
        }

        for ( int k = 0; k < secondYSpec.size(); k++ )  {
            for ( int j = 0; j < secondXSpec.size(); j++ )  {
                relativeGroundTruth.at<cv::Vec3f>(k,j)[0] = (unsigned)(secondXMovement*64+std::pow(2,15));
                relativeGroundTruth.at<cv::Vec3f>(k,j)[1] = (unsigned)(secondYMovement*64+std::pow(2,15));
                relativeGroundTruth.at<cv::Vec3f>(k,j)[2] = 1;
                absoluteGroundTruth.at<cv::Vec3f>(k,j)[0] = secondXMovement+j;
                absoluteGroundTruth.at<cv::Vec3f>(k,j)[1] = secondYMovement+k;
                absoluteGroundTruth.at<cv::Vec3f>(k,j)[2] = 1;
            }
        }

        //Create png Matrix with 3 channels: OF in vertical. OF in Horizontal and Validation bit
        //kittiGT = cat(3,relativeGroundTruth(:,:,1),relativeGroundTruth(:,:,2),relativeGroundTruth(:,:,3));
        //kittiGT =  uint16(kittiGT);
        relativeGroundTruth.copyTo(kittiGT);
        cv::imwrite(name_dense, kittiGT);

        //check for each frame (iteration) if the objects are colliding
        std::vector<unsigned> xCol = xSpec;
        std::vector<unsigned> yCol = ySpec;

        std::vector<unsigned> secondXCol= secondXSpec;
        std::vector<unsigned> secondYCol = secondYSpec;

        std::vector<unsigned> checkX, checkY;
        std::vector<unsigned> collisionVector(MAX_ITERATION);

        for ( unsigned i = 0; i < xCol.size() ; i++) {
            for ( unsigned j = 0; j < secondXCol.size() ; j++) {
                if ( std::abs(xCol.at(i) - secondXCol.at(j)) == 0) {
                    checkX.push_back(i);  // index of collision
                }
            }
        }
        for ( unsigned i = 0; i < yCol.size() ; i++) {
            for ( unsigned j = 0; j < secondYCol.size() ; j++) {
                if ( std::abs(yCol.at(i) - secondYCol.at(j)) == 0) {
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

        iterator = iterator+1;
        sIterator = sIterator+1;

        actualX = actualX+xMovement;
        actualY = actualY+yMovement;
        secondActualX = secondActualX+secondXMovement;
        secondActualY = secondActualY+secondYMovement;
    }
    std::cout << "end" << std::endl;
}


void flow() {

//how many interations(frames)?
    const unsigned MAX_ITERATION = 360;
    unsigned collision = 0, iterator = 0, sIterator = 0;
    std::vector<unsigned> xPos, yPos;
    unsigned start;
    unsigned secondStart;
//object specs
    const unsigned width = 30;
    const unsigned height = 100;

    unsigned actualX;
    unsigned actualY;
    unsigned secondActualX;
    unsigned secondActualY;

    cv::Mat absoluteFlow = cv::Mat::zeros(375, 1242, CV_16UC3);
    cv::Mat flow1 = cv::Mat::zeros(375, 1242, CV_16UC3);
    cv::Mat flow2 = cv::Mat::zeros(375, 1242, CV_16UC3);



    cv::Mat frame = cv::Mat::zeros(375, 1242, CV_8UC3);
    cv::Mat bg = cv::Mat::zeros(375,1242,CV_8UC3);

    cv::Mat frame_gray, prevGray, flow_frame(cv::Size(1242,375),CV_8UC3);

    std::vector<bool> estimatedCollisionVector(MAX_ITERATION);

    std::map<std::string, double> time_map;
    auto tic=steady_clock::now(), toc=steady_clock::now();

    bool plotTime = 1;
    std::vector<bool> error;
    error.at(0) = 0;
    error.at(1) = 0;

    for ( int k = 0; k < bg.rows; k++ )  {
        for ( int j = 0; j < bg.cols; j++ )  {
            bg.at<cv::Vec3f>(k,j)[0] = rand()%255;
            bg.at<cv::Vec3f>(k,j)[1] = rand()%255;
            bg.at<cv::Vec3f>(k,j)[2] = 0;
        }
    }

    for (unsigned x=0; x <= MAX_ITERATION; x++) {

        //Used to store the GT images for the kitti devkit

        char name_frame[50], name_flow[50];
        sprintf(name_frame, "../../../matlab_dataset/frames/0000000%03d.png", x);
        sprintf(name_flow, "../../../matlab_dataset/flow/0000000%03d.png", x);

        //Initialization
        if (x == 0) {
            iterator = 0;
            sIterator = 0;
        }

        //end of path vector? reset
        if ((iterator + start) > xPos.size()) {

            start = 1;
            iterator = 0;

        }

        if ((sIterator + secondStart) > xPos.size()) {

            secondStart = 1;
            sIterator = 0;

        }

        //Object specification
        std::vector<unsigned> xSpec;
        for (unsigned i = 0; i < width; i++) {
            xSpec.push_back(actualX + i);
        }
        std::vector<unsigned > ySpec;
        for (unsigned i = 0; i < height; i++) {
            ySpec.push_back(actualY + i);
        }

        std::vector<unsigned > secondXSpec;
        for (unsigned i = 0; i < width; i++) {
            secondXSpec.push_back(secondActualX + i);
        }

        std::vector<unsigned > secondYSpec;
        for (unsigned i = 0; i < height; i++) {
            secondYSpec.push_back(secondActualY + i);
        }

        // create the frame
        // std::cout << x << std::endl;
        // frame = movement(xSpec, ySpec, secondXSpec, secondYSpec, bg);
        //MOVEMENT Summary of this function goes here. Detailed explanation goes here

        tic = steady_clock::now();

        unsigned r = 0;
        unsigned b = 0;

        //reset the image to white
        for (int k = 0; k < bg.rows; k++) {
            for (int j = 0; j < bg.cols; j++) {
                frame.at<cv::Vec3f>(k, j)[0] = 255;
                frame.at<cv::Vec3f>(k, j)[1] = 255;
                frame.at<cv::Vec3f>(k, j)[2] = 255;
            }
        }

        //draw new image.
        for (int k = 0; k < ySpec.size(); k++) {
            for (int j = 0; j < xSpec.size(); j++) {
                frame.at<cv::Vec3f>(k, j)[0] = b;
                frame.at<cv::Vec3f>(k, j)[1] = 0;
                frame.at<cv::Vec3f>(k, j)[2] = r;
                r = r + 2;
                b = b + 2;
                if (r > 254)
                    r = 130;
            }
            if (b > 254)
                b = 46;
        }

        r = 0;
        b = 0;

        //expand with 2nd Object
        //draw new image.
        for (int k = 0; k < secondYSpec.size(); k++) {
            for (int j = 0; j < secondXSpec.size(); j++) {
                frame.at<cv::Vec3f>(k, j)[0] = b;
                frame.at<cv::Vec3f>(k, j)[1] = 0;
                frame.at<cv::Vec3f>(k, j)[2] = r;
                r = r + 2;
                b = b + 2;
                if (r > 254)
                    r = 130;
            }
            if (b > 254)
                b = 46;
        }

        r = 0;
        b = 0;


        //frame = imnoise(frame,'gaussian',0.5);

        toc = steady_clock::now();
        time_map["generate"] = duration_cast<milliseconds>(toc - tic).count();

        tic = steady_clock::now();

        cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);

        if (prevGray.data) {
            // Initialize parameters for the optical flow algorithm
            float pyrScale = 0.5;
            int numLevels = 3;
            int windowSize = 15;
            int numIterations = 3;
            int neighborhoodSize = 5;
            float stdDeviation = 1.2;

            //flow_frame = cv::calcOpticalFlowFarneback(opticFlow, frame_gray);
            // Calculate optical flow map using Farneback algorithm
            cv::calcOpticalFlowFarneback(prevGray, frame_gray, flow_frame, pyrScale, numLevels, windowSize,
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

        unsigned upperheigtht = ySpec.at(ySpec.size()-1);
        unsigned lowerheight = ySpec.at(0);
        unsigned upperwidth = xSpec.at(ySpec.size()-1);
        unsigned lowerwidth = xSpec.at(0);

        unsigned secondObjectUpperheight = secondYSpec.at(ySpec.size()-1);
        unsigned secondObjectLowerheight = secondYSpec.at(0);
        unsigned secondObjectUpperWidth = secondXSpec.at(ySpec.size()-1);
        unsigned secondObjectLowerWidth = secondXSpec.at(0);


        for ( int k = 0; k < ySpec.size(); k++ )  {
            for ( int j = 0; j < xSpec.size(); j++ ) {
                flow1.at<cv::Vec3f>(k, j)[0] = absoluteFlow.at<cv::Vec3f>(ySpec.at(k), xSpec.at(j))[0];
                flow1.at<cv::Vec3f>(k, j)[1] = absoluteFlow.at<cv::Vec3f>(ySpec.at(k), xSpec.at(j))[1];
            }
        }

        for ( int k = 0; k < secondYSpec.size(); k++ )  {
            for ( int j = 0; j < secondXSpec.size(); j++ ) {
                flow2.at<cv::Vec3f>(k, j)[0] = absoluteFlow.at<cv::Vec3f>(secondYSpec.at(k), secondXSpec.at(j))[0];
                flow2.at<cv::Vec3f>(k, j)[1] = absoluteFlow.at<cv::Vec3f>(secondYSpec.at(k), secondXSpec.at(j))[1];
            }
        }

       //%
       //%get flow from the objects
        for ( unsigned k = lowerheight; k < upperheigtht; k++ ) {
            for ( unsigned j = lowerwidth; j < upperwidth; j++ ) {
                    flowFirstObjectX.at<unsigned>(k,j) = flow.at<cv::Vec3f>(k,j)[0];
                    flowFirstObjectY.at<unsigned>(k,j) = flow.at<cv::Vec3f>(k,j)[1];
            }
        }

        for ( unsigned kk = secondObjectLowerheight; kk < secondObjectUpperheight; kk++ ) {
            for ( unsigned jj = secondObjectLowerWidth; jj < secondObjectUpperWidth; jj++ ) {
                flowSecondObjectX.at<unsigned>(kk,jj) = flow.at<cv::Vec3f>(kk,jj)[0];
                flowSecondObjectY.at<unsigned>(kk,jj) = flow.at<cv::Vec3f>(kk,jj)[1];
            }
        }

       //%
        //Extract the movement of the object.
        cv::Mat firstObjectX, firstObjectY, secondObjectX, secondObjectY;

        for ( unsigned i = 0; i < flowFirstObjectX.rows; i++ ) {
            for (unsigned j = 0; j < flowFirstObjectX.cols; i++) {
                if ( flowFirstObjectX.at<unsigned>(i,j) != 0 || cv::abs(flowFirstObjectX.at<unsigned>(i,j) < 0.2)) {
                    firstObjectX.push_back(flowFirstObjectX.at<unsigned>(i,j));
                }
            }
        }

        for ( unsigned i = 0; i < flowFirstObjectY.rows; i++ ) {
            for (unsigned j = 0; j < flowFirstObjectY.cols; i++) {
                if ( flowFirstObjectY.at<unsigned>(i,j) != 0 || cv::abs(flowFirstObjectY.at<unsigned>(i,j) < 0.2)) {
                    firstObjectY.push_back(flowFirstObjectY.at<unsigned>(i,j));
                }
            }
        }

        for ( unsigned i = 0; i < flowSecondObjectX.rows; i++ ) {
            for (unsigned j = 0; j < flowSecondObjectX.cols; i++) {
                if ( flowSecondObjectX.at<unsigned>(i,j) != 0 || cv::abs(flowSecondObjectX.at<unsigned>(i,j) < 0.2)) {
                    secondObjectX.push_back(flowSecondObjectX.at<unsigned>(i,j));
                }
            }
        }

        for ( unsigned i = 0; i < flowSecondObjectY.rows; i++ ) {
            for (unsigned j = 0; j < flowSecondObjectY.cols; i++) {
                if ( flowSecondObjectY.at<unsigned>(i,j) != 0 || cv::abs(flowSecondObjectY.at<unsigned>(i,j) < 0.2)) {
                    secondObjectY.push_back(flowSecondObjectY.at<unsigned>(i,j));
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


        for ( unsigned i = 0; i < xSpec.size(); i++ ) {
            xSpec.at(i) = std::floor(xSpec.at(i) + xMean[0]);
            secondXSpec.at(i) = std::floor(secondXSpec.at(i) + secondObjectXMean[0]);
        }

        for ( unsigned i = 0; i < ySpec.size(); i++ ) {
            ySpec.at(i) = std::floor(ySpec.at(i) + yMean[0]);
            secondYSpec.at(i) = std::floor(secondYSpec.at(i) + secondObjectYMean[0]);
        }

        std::vector<unsigned> checkX, checkY;
        std::vector<unsigned> collisionVector(MAX_ITERATION);

        for ( unsigned i = 0; i < xSpec.size() ; i++) {
            for ( unsigned j = 0; j < secondXSpec.size() ; j++) {
                if ( std::abs(xSpec.at(i) - secondXSpec.at(j)) == 0) {
                    checkX.push_back(i);  // index of collision
                }
            }
        }
        for ( unsigned i = 0; i < ySpec.size() ; i++) {
            for ( unsigned j = 0; j < secondYSpec.size() ; j++) {
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

        time_map["movement"] = duration_cast<milliseconds>(toc - tic).count();
        time_map["collision"] = duration_cast<milliseconds>(toc - tic).count();


        for ( int k = 0; k < ySpec.size(); k++ )  {
            for ( int j = 0; j < xSpec.size(); j++ ) {
                absoluteFlow.at<cv::Vec3f>(k, j)[0] = estMovement.at(0) + j;
                absoluteFlow.at<cv::Vec3f>(k, j)[1] = estMovement.at(1) + k;
                absoluteFlow.at<cv::Vec3f>(k, j)[2] = 1;
            }
        }
        for ( int k = 0; k < secondYSpec.size(); k++ )  {
            for ( int j = 0; j < secondXSpec.size(); j++ ) {
                absoluteFlow.at<cv::Vec3f>(k, j)[0] = estMovement.at(2) + j;
                absoluteFlow.at<cv::Vec3f>(k, j)[1] = estMovement.at(3) + k;
                absoluteFlow.at<cv::Vec3f>(k, j)[2] = 1;
            }
        }
        //abs1 = absoluteFlow(:,:,1);


        ////
        ////collision checkers(first round has bad flow, dont plot and
        ////estimate collision
        //time to check for collision



        if (estimatedCollision == 1)
            estimatedCollisionVector.at(x) = 1;
        else
            estimatedCollisionVector.at(x) = 0;

        tic = steady_clock::now();


        //Update position (the objects of interest are tracked via Ground Truth here)

        actualX = xPos.at(start + iterator);
        actualY = yPos.at(start + iterator);
        secondActualX = xPos.at(secondStart + sIterator);
        secondActualY = yPos.at(secondStart + sIterator);


        iterator = iterator + 1;
        sIterator = sIterator + 1;

        //imwrite(frame, name_frame);

    }

    //Create Video out of frames.
    std::string dir_path = "image_02/data/";
    cv::VideoWriter video_out;
    char file_name_char[100];
    video_out.open("Movement.avi",CV_FOURCC('D','I','V','X'), 5, cv::Size(1242,375), true);
    for (int x = 0; x < MAX_ITERATION; x++) {
        sprintf(file_name_char, "0000000%03d", x);
        video_out.write(cv::imread(file_name_char, cv::IMREAD_COLOR));

    }
    video_out.release();

 }

int main() {

    ground_truth();
    //flow();


}