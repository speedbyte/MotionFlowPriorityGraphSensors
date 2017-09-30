

#include <vector>
#include <cmath>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>
#include <map>
#include <chrono>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

//Creating a movement path. The path is stored in a x and y vector

using namespace std::chrono;

extern void flow(std::string algo);

#define MATLAB_DATASET_PATH "../../../matlab_dataset/"

void ground_truth() {

    cv::Size frame_size(1242,275);
    std::map<std::string, double> time_map = {{"generate",0}, {"ground truth", 0}};
    boost::filesystem::path gt_video_path;
    gt_video_path = std::string(MATLAB_DATASET_PATH) + std::string("ground_truth/gtMovement.avi");
    assert(boost::filesystem::exists(gt_video_path.parent_path()) != 0);

    cv::VideoWriter video_out;
    video_out.open(gt_video_path.string(), CV_FOURCC('D', 'I', 'V', 'X'), 30, frame_size, true);

    //how many interations(frames)?
    auto tic= steady_clock::now();
    auto toc= steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();

    const ushort MAX_ITERATION = 360;
    ushort collision = 0, iterator = 0, sIterator = 0;
    std::vector<ushort> xPos, yPos;

    ushort xMovement,yMovement,secondXMovement,secondYMovement;

    ushort actualX;
    ushort actualY;
    ushort secondActualX;
    ushort secondActualY;

    //object specs
    const ushort width = 30;
    const ushort height = 100;

    std::vector<ushort> theta;
    for ( ushort x = 0; x < MAX_ITERATION; x++) {
        theta.push_back(x);
    }

    for ( int i = 0; i< MAX_ITERATION; i++) {
        xPos.push_back((ushort)(1 * cos(theta[i] * CV_PI / 180.0) / (1.0 + std::pow(sin(theta[i] * CV_PI / 180.0), 2))));
        yPos.push_back((ushort)(1 * (cos(theta[i] * CV_PI / 180.0) * sin(theta[i] * CV_PI / 180.0)) / (0.2 + std::pow(sin
                                                                                                                              (theta[i] *
                                                                                                                               CV_PI /
                                                                                                                               180.0),
                                                                                                                      2))));
    }

    //Start is somewhere on the path
    ushort start = 30;
    ushort xOrigin = xPos.at(start);
    ushort yOrigin = yPos.at(start);

    //Start of the second object is somewhere on the path
    ushort secondStart = 200;
    ushort secondXOrigin = xPos.at(secondStart);
    ushort secondYOrigin = yPos.at(secondStart);

    //for moving the objects later
    actualX = xOrigin;
    actualY = yOrigin;
    secondActualX = secondXOrigin;
    secondActualY = secondYOrigin;

    //Initialization
    //Ground Truth Movement (First Object x, first Object y, second object x, second object y movement
    iterator = 0;
    sIterator = 0;
    xMovement = 0;
    secondXMovement = 0;
    yMovement = 0;
    secondYMovement = 0;

    cv::Mat kittiGT(frame_size,CV_16UC3,cv::Scalar(0,0,0));
    cv::Mat relativeGroundTruth(frame_size,CV_16UC3,cv::Scalar(0,0,0));
    cv::Mat absoluteGroundTruth(frame_size,CV_16UC3,cv::Scalar(0,0,0));

    cv::Mat frame = cv::Mat::zeros(frame_size, CV_8UC3);



    for ( int k = 0; k < frame.rows; k++ )  {
        for ( int j = 0; j < frame.cols; j++ )  {
            frame.at<cv::Vec3b>(k,j)[0] = (uchar)(rand()%255);
            frame.at<cv::Vec3b>(k,j)[1] = (uchar)(rand()%255);
            frame.at<cv::Vec3b>(k,j)[2] = 0;
        }
    }


    tic_all = steady_clock::now();
    for (ushort x=0; x < MAX_ITERATION; x++) {


        //Used to store the GT images for the kitti devkit

        char file_name[20];
        sprintf(file_name, "0000000%03d.png", x);
        std::string gt_image_path = gt_video_path.parent_path().string() + '/' + std::string(file_name);
        printf("%u, %u , %u, %u, %u\n", x, start, iterator, secondStart, sIterator);

        //If we are at the end of the path vector, we need to reset our iterators
        if ((iterator+start) >= xPos.size()) {
            start = 0;
            iterator = 0;
            xMovement = xPos.at(0) - xPos.at(xPos.size() - 1);
            yMovement = yPos.at(0) - yPos.at(yPos.size() - 1);
        } else {
            xMovement = xPos.at(start+iterator) - xPos.at(start+iterator-(ushort)1);
            yMovement = yPos.at(start+iterator) - yPos.at(start+iterator-(ushort)1);
        }

        if ((sIterator+secondStart) >= xPos.size()) {
            secondStart = 0;
            sIterator = 0;
            secondXMovement = xPos.at(0) - xPos.at(xPos.size()-1);
            secondYMovement = yPos.at(0) - yPos.at(yPos.size()-1);
        } else {
            secondXMovement = xPos.at(secondStart+sIterator) - xPos.at(secondStart+sIterator-(ushort)1);
            secondYMovement = yPos.at(secondStart+sIterator) - yPos.at(secondStart+sIterator-(ushort)1);
        }


        //Object specification
        std::vector<ushort> xSpec;
        for ( ushort i = 0; i < width; i++) {
            xSpec.push_back(actualX+i);
        }
        std::vector<ushort> ySpec;
        for ( ushort i = 0; i < height; i++) {
            ySpec.push_back(actualY+i);
        }

        std::vector<ushort> secondXSpec;
        for ( ushort i = 0; i < width; i++) {
            secondXSpec.push_back(secondActualX+i);
        }

        std::vector<ushort> secondYSpec;
        for ( ushort i = 0; i < height; i++) {
            secondYSpec.push_back(secondActualY+i);
        }


        // create the frame
        tic = steady_clock::now();

        uchar r = 0;
        uchar b = 0;

        //reset the image to white
        for (int k = 0; k < frame.rows; k++) {
            for (int j = 0; j < frame.cols; j++) {
                frame.at<cv::Vec3b>(k, j)[0] = 255;
                frame.at<cv::Vec3b>(k, j)[1] = 255;
                frame.at<cv::Vec3b>(k, j)[2] = 255;
            }
        }

        //draw new image.
        for (int k = ySpec.at(0); k < ySpec.at(ySpec.size() - 1); k++) {
            for (int j = xSpec.at(0); j < xSpec.at(xSpec.size() - 1); j++) {
                frame.at<cv::Vec3b>(k, j)[0] = b;
                frame.at<cv::Vec3b>(k, j)[1] = 0;
                frame.at<cv::Vec3b>(k, j)[2] = r;
                r = r + (uchar)2;
                b = b + (uchar)2;
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
        for (int k = secondYSpec.at(0); k < secondYSpec.at(secondYSpec.size() - 1); k++) {
            for (int j = secondXSpec.at(0); j < secondXSpec.at(secondXSpec.size() - 1); j++) {
                frame.at<cv::Vec3b>(k, j)[0] = b;
                frame.at<cv::Vec3b>(k, j)[1] = 0;
                frame.at<cv::Vec3b>(k, j)[2] = r;
                r = r + (uchar)2;
                b = b + (uchar)2;
                if (r > 254)
                    r = 130;
            }
            if (b > 254)
                b = 46;
        }

        r = 0;
        b = 0;

        if ( !video_out.isOpened() ) {
            std::cerr << "Could not open video" << std::endl;
        }
        video_out.write(frame);

        //frame = imnoise(frame,'gaussian',0.5);

        toc = steady_clock::now();
        time_map["generate"] =  duration_cast<milliseconds>(toc - tic).count();


        //calculating the relative Ground Truth for the Kitti devkit and store it in a png file

        for ( int k = ySpec.at(0); k < ySpec.at(ySpec.size()-1); k++ )  {
            for ( int j = xSpec.at(0); j < xSpec.at(xSpec.size()-1); j++ )  {
                assert(relativeGroundTruth.channels() == 3);
                relativeGroundTruth.at<cv::Vec3w>(k,j)[0] = (ushort)(xMovement*64+std::pow(2,15));
                relativeGroundTruth.at<cv::Vec3w>(k,j)[1] = (ushort)(yMovement*64+std::pow(2,15));
                relativeGroundTruth.at<cv::Vec3w>(k,j)[2] = 1;
                absoluteGroundTruth.at<cv::Vec3w>(k,j)[0] = (ushort)(xMovement+j);
                absoluteGroundTruth.at<cv::Vec3w>(k,j)[1] = (ushort)(yMovement+k);
                absoluteGroundTruth.at<cv::Vec3w>(k,j)[2] = 1;

                }
        }

        for ( int k = secondYSpec.at(0); k < secondYSpec.at(secondYSpec.size()-1); k++ )  {
            for ( int j = secondXSpec.at(0); j < secondXSpec.at(secondXSpec.size()-1); j++ )  {
                assert(relativeGroundTruth.channels() == 3);
                relativeGroundTruth.at<cv::Vec3w>(k,j)[0] = (ushort)(secondXMovement*64+std::pow(2,15));
                relativeGroundTruth.at<cv::Vec3w>(k,j)[1] = (ushort)(secondYMovement*64+std::pow(2,15));
                relativeGroundTruth.at<cv::Vec3w>(k,j)[2] = 1;
                absoluteGroundTruth.at<cv::Vec3w>(k,j)[0] = (ushort)(secondXMovement+j);
                absoluteGroundTruth.at<cv::Vec3w>(k,j)[1] = (ushort)(secondYMovement+k);
                absoluteGroundTruth.at<cv::Vec3w>(k,j)[2] = 1;
            }
        }

        //Create png Matrix with 3 channels: OF in vertical. OF in Horizontal and Validation bit

        relativeGroundTruth.copyTo(kittiGT);
        relativeGroundTruth.convertTo(kittiGT, CV_8UC3);
        //cv::imwrite(gt_image_path, kittiGT);

        //check for each frame (iteration) if the objects are colliding
        std::vector<ushort> xCol = xSpec;
        std::vector<ushort> yCol = ySpec;

        std::vector<ushort> secondXCol= secondXSpec;
        std::vector<ushort> secondYCol = secondYSpec;

        std::vector<ushort> checkX, checkY;
        std::vector<ushort> collisionVector;
        for ( ushort i = 0; i < MAX_ITERATION; i++) {
            collisionVector.push_back(0);
        }

        for ( ushort i = 0; i < xCol.size() ; i++) {
            for ( ushort j = 0; j < secondXCol.size() ; j++) {
                if ( std::abs(xCol.at(i) - secondXCol.at(j)) == 0) {
                    checkX.push_back(i);  // index of collision
                    break;
                }
            }
        }
        for ( ushort i = 0; i < yCol.size() ; i++) {
            for ( ushort j = 0; j < secondYCol.size() ; j++) {
                if ( std::abs(yCol.at(i) - secondYCol.at(j)) == 0) {
                    checkY.push_back(i);  // index of collision
                    break;
                }
            }
        }

        if (!checkX.empty() && !checkY.empty()) {
            collisionVector.at(x) = 1;
        }
        else {
            collisionVector.at(x) = 0;
        }

        iterator++;
        sIterator++;

        actualX = actualX+xMovement;
        actualY = actualY+yMovement;
        secondActualX = secondActualX+secondXMovement;
        secondActualY = secondActualY+secondYMovement;
        std::cout << "generate frame - " << time_map["generate"]  << "ms" << std::endl;

    }
    video_out.release();
    toc_all = steady_clock::now();
    time_map["ground truth"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth generation time - " << time_map["ground truth"]  << "ms" << std::endl;

}


int main() {

    ground_truth();
    //flow("FB");


}