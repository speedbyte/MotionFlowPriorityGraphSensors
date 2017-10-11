
#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>

using std::cout;
using std::endl;
using std::vector;



int main(int argc, char** argv) {

    char im1[100];
    char im2[100];

    char xFlow[100];
    char yFlow[100];
    // Initialize, load two images from the file system, and
    // allocate the images and other structures we will need for
    // results.
    //

    for (int it = 0; it < 200; it++) {


        sprintf(im1,"../../../../kitti_flow_dataset/data/stereo_flow/image_02/000%03d_10.png",it);
        sprintf(im2,"../../../../kitti_flow_dataset/data/stereo_flow/image_02/000%03d_11.png",it);

        cout << im1 << endl;



        cv::Mat imgA = cv::imread(im1, CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat imgB = cv::imread(im2, CV_LOAD_IMAGE_GRAYSCALE);
        int win_size = 10;
        // The first thing we need to do is get the features
        // we want to track.
        //
        vector<cv::Point2f> cornersA, cornersB;
        const int MAX_CORNERS = 1242*375;
        cv::goodFeaturesToTrack(
                imgA,                         // Image to track
                cornersA,                     // Vector of detected corners (output)
                MAX_CORNERS,                  // Keep up to this many corners
                0.005,                         // Quality level (percent of maximum)
                1,                            // Min distance between corners
                cv::noArray(),                // Mask
                3,                            // Block size
                false,                        // true: Harris, false: Shi-Tomasi
                0.04                          // method specific parameter
        );


        cv::cornerSubPix(
                imgA,                           // Input image
                cornersA,                       // Vector of corners (input and output)
                cv::Size(win_size, win_size),   // Half side length of search window
                cv::Size(-1, -1),               // Half side length of dead zone (-1=none)
                cv::TermCriteria(
                        cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                        20,                         // Maximum number of iterations
                        0.03                        // Minimum change per iteration
                )
        );

        // Call the Lucas Kanade algorithm
        //



        vector<uchar> features_found;
        cv::calcOpticalFlowPyrLK(
                imgA,                         // Previous image
                imgB,                         // Next image
                cornersA,                     // Previous set of corners (from imgA)
                cornersB,                     // Next set of corners (from imgB)
                features_found,               // Output vector, each is 1 for tracked
                cv::noArray(),                // Output vector, lists errors (optional)
                cv::Size(win_size * 2 + 1, win_size * 2 + 1),  // Search window size
                5,                            // Maximum pyramid level to construct
                cv::TermCriteria(
                        cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                        20,                         // Maximum number of iterations
                        0.3                         // Minimum change per iteration
                )
        );


        sprintf(xFlow,"../../FlowTextFiles/FlowX/x%03d.txt",it);
        sprintf(yFlow,"../../FlowTextFiles/FlowY/y%03d.txt",it);


        std::ofstream flowX;
        flowX.open(xFlow);

        std::ofstream flowY;
        flowY.open(yFlow);

        for (int k = 0; k < cornersA.size(); k++) {
            flowX << (int) cornersA[k].x << " " << cornersB[k].x - cornersA[k].x << endl;
            flowY << (int) cornersA[k].y << " " << cornersB[k].y - cornersA[k].y << endl;


        }


    }



    return 0;
}