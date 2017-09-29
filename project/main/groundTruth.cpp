

// This file converts Initialisation.m, gtColision.m and gtMovement.m and main.m from matlab.
//movement.m and flowCollision.m

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

//Creating a movement path. The path is stored in a x and y vector

//how many interations(frames)?
const unsigned maxIteration = 360;
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

cv::Mat absoluteFlow = cv::Mat::zeros(375, 1242, CV_16UC3);
cv::Mat flow1 = cv::Mat::zeros(375, 1242, CV_16UC3);
cv::Mat flow2 = cv::Mat::zeros(375, 1242, CV_16UC3);



void ground_truth() {

    std::vector<unsigned> theta;
    for ( unsigned x = 0; x <= 360; x++) {
        theta.push_back(x);
    }

    for ( int i = 0; i< 360; i++) {
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


    for (unsigned x=0; x < maxIteration; x++) {

        //Used to store the GT images for the kitti devkit

        char name_dense[50];
        std::sprintf(name_dense, "../../../matlab_dataset/ground_truth/0000000%03d.png", x);

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

        std::vector<unsigned> checkX(1), checkY(1);
        std::vector<unsigned> collisionVector(360);

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

        if (!checkX.empty() && checkY.empty()) {
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
    //Loads the Initialization specs.
    //kitti einbinden
    //Document
    //make everything more generic
    //classify everything that moves as object
    //noise(static, dynamic)

    //59-70
    //Err Mean 2.50880

    //Noise  frame = imnoise(frame,'gaussian',0.05);
    //Err Mean 4.4

    //Noise  frame = imnoise(frame,'gaussian',0.05);
    // Err Mean 4.76

    //21.9
    //NOISE MIGHT HAVE POSITIVE INFLUENCE


    //load('Initialize.mat');
    //load('collisionVector.mat');

    //opticFlow=opticalFlowFarneback;//('NoiseThreshold',0.004);

    cv::Mat frame = cv::Mat::zeros(375,1242,CV_8UC3);
    cv::Mat bg = cv::Mat::zeros(375,1242,CV_8UC3);

    bool plotTime = 1;
    std::vector<bool> error;
    error.at(0) = 0;
    error.at(1) = 0;

    for ( int k = 0; k < bg.rows; k++ )  {
        for ( int j = 0; j < bg.cols; j++ )  {
            bg.at<cv::Vec3f>(k,j)[0] = rand()%255;
            bg.at<cv::Vec3f>(k,j)[1] = rand()%255;

        }
    }

    for (unsigned x=1; x <= maxIteration; x++) {

        //Used to store the GT images for the kitti devkit

        char name_frame[50], name_flow[50];
        std::sprintf(name_frame, "../../../matlab_dataset/frames/0000000%03d.png", x);
        std::sprintf(name_flow, "../../../matlab_dataset/flow/0000000%03d.png", x);

        //Initialization
        if (x == 1) {
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
        std::vector<float> xSpec;
        for (unsigned i = 0; i < width; i++) {
            xSpec.push_back(actualX + width);
        }
        std::vector<float> ySpec;
        for (unsigned i = 0; i < height; i++) {
            ySpec.push_back(actualY + height);
        }

        std::vector<float> secondXSpec;
        for (unsigned i = 0; i < width; i++) {
            secondXSpec.push_back(secondActualX + width);
        }

        std::vector<float> secondYSpec;
        for (unsigned i = 0; i < height; i++) {
            secondYSpec.push_back(secondActualY + height);
        }



        // create the frame
        // disp(x);
        //frame = movement(xSpec, ySpec, secondXSpec, secondYSpec, bg);
        //MOVEMENT Summary of this function goes here
        // Detailed explanation goes here
        cv::Mat frame = cv::Mat::zeros(375, 1242, CV_8UC3);
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
                frame.at<cv::Vec3f>(k, j)[0] = r;
                frame.at<cv::Vec3f>(k, j)[1] = b;
                frame.at<cv::Vec3f>(k, j)[2] = 0;
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
                frame.at<cv::Vec3f>(k, j)[0] = r;
                frame.at<cv::Vec3f>(k, j)[1] = b;
                frame.at<cv::Vec3f>(k, j)[2] = 0;
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


        //   frame = imnoise(frame,'gaussian',0.5);

        //add noise

        //timeToGenerateObject(x) = toc;

        //addpath('LKpyramid Codes');
        ////
        //Optical Flow
        //tic;
        cv::Mat frame_gray;
        cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
/*        flow_frame = estimateFlow(opticFlow, frame_gray);
        flowstop(x) = toc;


        vxCopy = (flow_frame.Vx ~ = 0);
        vyCopy = (flow_frame.Vy ~ = 0);
        vXYCopy = vxCopy + vyCopy;
        vCopy = (vXYCopy~ = 0);

        res = cat(3, flow_frame.Vx * 64 + 2 ^ 15, flow_frame.Vy * 64 + 2 ^ 15, vCopy);
        res = uint16(res);
        imwrite(res, 'result.png');

        //create flow matrix to store the estimated displacemend in.
        flow(:,:,1) = flow_frame.Vx;
        flow(:,:,2) = flow_frame.Vy;

        //absolute Flow
        //get absolute estimated flow.

        tic;
        [estMovement, estimatedCollision] = estimatedMovement(flow, xSpec, ySpec, secondXSpec, secondYSpec);
        timeMovement(x) = toc;
        collisionTime(x) = toc;

        std::vector<float> estMovement(2);
        for ( int k = 0; k < ySpec.size(); k++ )  {
            for ( int j = 0; j < xSpec.size(); j++ ) {
                absoluteFlow.at(k, j)[0] = estMovement.at(1) + j;
                absoluteFlow.at(k, j)[1] = estMovement.at(2) + k;
                absoluteFlow.at(k, j)[2] = 1;
            }
        }
        for ( int k = 0; k < secondYSpec.size(); k++ )  {
            for ( int j = 0; j < secondXSpec.size(); j++ ) {
                absoluteFlow.at(k, j)[0] = estMovement.at(1) + j;
                absoluteFlow.at(k, j)[1] = estMovement.at(2) + k;
                absoluteFlow.at(k, j)[2] = 1;
            }
        }
        //abs1 = absoluteFlow(:,:,1);


        ////
        ////collision checkers(first round has bad flow, dont plot and
        ////estimate collision
        //time to check for collision


        //  estimatedCollision = flowCollision(absoluteFlow, xSpec,ySpec, secondXSpec,secondYSpec);

        // Estimate if the objects will collide
        // Substract the absolute Flows of the bothobjects of interest. If the result is <= width and height of the
        // object, the objects are colliding

        unsigned estCollision = 0;
        //Estimate the future collision. Floor call in order to get possibly matching results

        for ( int k = 0; k < ySpec.size(); k++ )  {
            for ( int j = 0; j < xSpec.size(); j++ ) {
                flow1.at(k, j)[0] = absoluteFlow.at(ySpec.at(k), xSpec.at(j))[0];
                flow1.at(k, j)[1] = absoluteFlow.at(ySpec.at(k), xSpec.at(j))[1];
            }
        }

        for ( int k = 0; k < secondYSpec.size(); k++ )  {
            for ( int j = 0; j < secondXSpec.size(); j++ ) {
                flow2.at(k, j)[0] = absoluteFlow.at(secondYSpec.at(k), secondXSpec.at(j))[0];
                flow2.at(k, j)[1] = absoluteFlow.at(secondYSpec.at(k), secondXSpec.at(j))[1];
            }
        }

        abs1 = absoluteFlow(:,:,1);
        f1 = flow1(:,:,1);
        f2 = flow2(:,:,1);
        f3 = f1-f2;
        finalFlow = flow1-flow2;
        finalFlow = abs(finalFlow);


        sizeCheckX = length(xSpec);
        sizeCheckY = length(ySpec);

        xChecker = ismember([0:sizeCheckX],finalFlow(:,:,1));
        yChecker = ismember([0:sizeCheckY],finalFlow(:,:,2));

        if ismember(1,xChecker) & ismember(1,yChecker)
        estCollision = 1;
        end
                end




        if (estimatedCollision == 1)
            estimatedCollisionVector.at(x) = 1;
        else
            estimatedCollisionVector(x) = 0;
        end

                tic;

        //Kitti Plot, Error calculation and mean error calculation
        if x > 2
        tau = [3
        0.05];
        name = sprintf('./GroundTruth/%06d_10.png', x);
        addpath(genpath('../../../kitti_eval/devkit_stereo_opticalflow_sceneflow/matlab/'));
        F_gt = flow_read(name);
        F_est = flow_read('result.png');
        f_err = flow_error(F_gt, F_est, tau);
        f_err = f_err * 100;
        error(x) = f_err;
        F_err = flow_error_image(F_gt, F_est, tau);
        errSum = sum(error);
        errorMean(x) = errSum / x;


        plotter(frame, flow_frame, collisionVector, estimatedCollisionVector, actualX, actualY, secondActualX,
                secondActualY, estMovement, x, timeToGenerateObject, flowstop, plotTime, collisionTime, timeMovement,
                error, f_err, errorMean, F_est, F_gt, F_err);
        plotTime(x) = toc;
        end

        ////
        //Update position(the objects of interest are tracked via Ground Truth
        //here)

                actualX = xPos(start + iterator);
        actualY = yPos(start + iterator);
        secondActualX = xPos(secondStart + sIterator);
        secondActualY = yPos(secondStart + sIterator);


        iterator = iterator + 1;
        sIterator = sIterator + 1;

        imwrite(frame, name_frame);
*/
    }

    //Create Video out of frames.
    std::string dir_path = "image_02/data/";
    cv::VideoWriter video_out;
    char file_name_char[100];
    video_out.open("Movement.avi",CV_FOURCC('D','I','V','X'), 5, cv::Size(1242,375), true);
    for (int x = 0; x < 360; x++) {
        sprintf(file_name_char, "0000000%03d", x);
        video_out.write(cv::imread(file_name_char, cv::IMREAD_COLOR));

    }
    video_out.release();

 }

int main() {

    ground_truth();
    //flow();


}