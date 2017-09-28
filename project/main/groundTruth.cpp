

//Generates the ground truth Kitti Maps and the Ground Truth collision


//This script initializes the movement.
//First, the path is calculated.
//Then objects specs are set
//Then start points of the first and second object are set

#include <vector>
#include <cmath>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

//Creating a movement path. The path is stored in a x and y vector

void ground_truth() {

    std::vector<unsigned> theta;
    std::vector<float> xPos, yPos;
    for ( unsigned x = 0; x <= 360; x++) {
        theta.push_back(x);
    }

    for ( int i = 0; i< 360; i++) {
        xPos.push_back(1 * cos(theta[i] * CV_PI / 180.0) / (1.0 + std::pow(sin(theta[i] * CV_PI / 180.0), 2)));
        yPos.push_back(1 * (cos(theta[i] * CV_PI / 180.0) * sin(theta[i] * CV_PI / 180.0)) / (0.2 + std::pow(sin
                                                                                                                      (theta[i] *
                                                                                                                       CV_PI /
                                                                                                                       180.0),
                                                                                                              2)));
    }

    //object specs
    unsigned width = 30;
    unsigned height = 100;

    //Start is somewhere on the path
    unsigned start = 30;
    float xOrigin = xPos.at(start);
    float yOrigin = yPos.at(start);

    //Start of the second object is somewhere on the path
    unsigned secondStart = 200;
    float secondXOrigin = xPos.at(secondStart);
    float secondYOrigin = yPos.at(secondStart);

    //for moving the objects later
    float actualX = xOrigin;
    float actualY = yOrigin;
    float secondActualX = secondXOrigin;
    float secondActualY = secondYOrigin;


    //how many interations(frames)?
    unsigned maxIteration = 360;


    float xMovement,yMovement,secondXMovement,secondYMovement;

    unsigned collision = 0, iterator = 0, sIterator = 0;

    cv::Mat kittiGT(375,1242,CV_32FC3,cv::Scalar(0,0,0));
    cv::Mat relativeGroundTruth(375,1242,CV_32FC3,cv::Scalar(0,0,0));
    cv::Mat absoluteGroundTruth = cv::Mat::zeros(375,1242,CV_32FC3);


    for (unsigned x=1; x <= maxIteration; x++) {

        //Used to store the GT images for the kitti devkit

        char name_dense[50];
        sprintf(name_dense, "../../../matlab_dataset/ground_truth/0000000%03d.png", x);

        //Initialization
        if (x == 1) {
            iterator = 0;
            sIterator = 0;
        }

        //Ground Truth Movement (First Object x, first Object y, second object x, second object y movement

        if (x == 1) {
            xMovement = 0;
            secondXMovement = 0;
            yMovement = 0;
            secondYMovement = 0;
        }


        if ((iterator+start) > xPos.size()) {

            xMovement = xPos.at(1) - xPos.at(xPos.size());
            yMovement = yPos.at(1) - yPos.at(yPos.size());

        }

        if ( (sIterator+secondStart) > xPos.size()) {

            secondXMovement = xPos.at(1) - xPos.at(xPos.size());
            secondYMovement = yPos.at(1) - yPos.at(yPos.size());

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
        std::vector<double> xSpec;
        for ( unsigned i = 0; i < width; i++) {
            xSpec.push_back(actualX+width);
        }
        std::vector<double> ySpec;
        for ( unsigned i = 0; i < height; i++) {
            ySpec.push_back(actualY+height);
        }

        std::vector<double> secondXSpec;
        for ( unsigned i = 0; i < width; i++) {
            secondXSpec.push_back(secondActualX+width);
        }

        std::vector<double> secondYSpec;
        for ( unsigned i = 0; i < height; i++) {
            secondYSpec.push_back(secondActualY+height);
        }

        //
        //calculating the relative Ground Truth for the Kitti devkit and store it
        //in a png file

        for ( int k = 0; k < ySpec.size(); k++ )  {
            for ( int j = 0; j < xSpec.size(); j++ )  {
                    relativeGroundTruth.at<cv::Vec3f>(k,j)[0] = xMovement*64+std::pow(2,15);
                    relativeGroundTruth.at<cv::Vec3f>(k,j)[1] = yMovement*64+std::pow(2,15);
                    relativeGroundTruth.at<cv::Vec3f>(k,j)[2] = 1;
                    absoluteGroundTruth.at<cv::Vec3f>(k,j)[0] = xMovement+j;
                    absoluteGroundTruth.at<cv::Vec3f>(k,j)[1] = yMovement+k;
                    absoluteGroundTruth.at<cv::Vec3f>(k,j)[2] = 1;

                }
        }

        for ( int k = 0; k < secondYSpec.size(); k++ )  {
            for ( int j = 0; j < secondXSpec.size(); j++ )  {
                relativeGroundTruth.at<cv::Vec3f>(k,j)[0] = secondXMovement*64+std::pow(2,15);
                relativeGroundTruth.at<cv::Vec3f>(k,j)[1] = secondYMovement*64+std::pow(2,15);
                relativeGroundTruth.at<cv::Vec3f>(k,j)[2] = 1;
                absoluteGroundTruth.at<cv::Vec3f>(k,j)[0] = secondXMovement+j;
                absoluteGroundTruth.at<cv::Vec3f>(k,j)[1] = secondYMovement+k;
                absoluteGroundTruth.at<cv::Vec3f>(k,j)[2] = 1;
            }
        }

        //Create png Matrix with 3 channels: OF in vertical. OF in Horizontal.
        //and Validation bit
        //kittiGT = cat(3,relativeGroundTruth(:,:,1),relativeGroundTruth(:,:,2),relativeGroundTruth(:,:,3));
        //kittiGT =  uint16(kittiGT);
        relativeGroundTruth.convertTo(kittiGT, CV_16UC3);
        cv::imwrite(name_dense, kittiGT);


        /*
        //check for each frame (iteration) if the objects are colliding
        std::vector<double> xCol = xSpec;
        std::vector<double> yCol = ySpec;

        std::vector<double> secondXCol= secondXSpec;
        std::vector<double> secondYCol = secondYSpec;

        checkX = intersect(xCol,secondXCol);
        checkY = intersect(yCol,secondYCol);

        if ~isempty(checkX) & ~isempty(checkY)
        collisionVector(x) = 1;
        else
        collisionVector(x) = 0;
        end

                iterator = iterator+1;
        sIterator = sIterator+1;

        actualX = actualX+calcMove(1);
        actualY = actualY+calcMove(2);
        secondActualX = secondActualX+calcMove(3);
        secondActualY = secondActualY+calcMove(4);
        */
    }
    std::cout << "end" << std::endl;
}