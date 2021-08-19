/**
 * @function calcHist_Demo.cpp
 * @brief Demo code to use the function calcHist
 * @author
 */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <map>
#include <ctime>
#include <iomanip>

using namespace std;
using namespace cv;

std::map<float, int> hist;
std::vector<float> myVector;

void data_generator(std::string data_type) {

    std::cout << FLT_MAX << " " << INT32_MIN << " " << INT32_MAX << " " << std::numeric_limits<float>::infinity() << std::endl;

    time_t rawtime; time(&rawtime);
    std::cout << asctime(localtime(&rawtime));
    // Seed with a real random value, if available
    std::random_device r;
    std::cout << r() << std::endl;

    std::default_random_engine e1(r());
    std::uniform_real_distribution<float> uniform_dist(-10.0f, 10.0f);
    float mean = uniform_dist(e1);
    std::cout << "Randomly-chosen mean: " << mean << '\n';

    // Generate a normal distribution around that mean
    std::seed_seq seed{r(), r(), r(), r(), r(), r(), r(), r()};
    std::mt19937 e2(seed);
    std::normal_distribution<> normal_dist(mean, 2);

    if ( data_type == "hist") {
        for (int n = 0; n < 10000; ++n) {
            ++hist[std::round((normal_dist(e2))*100)/100];
        }
        std::cout << "Normal distribution around " << mean << ":\n";
        for (auto p : hist) {
            std::cout << std::fixed << std::setprecision(2) << std::setw(2)
                      << p.first << ' ' << std::string(p.second, '*') << '\n';
        }
    }
    else if ( data_type == "vector") {
        // Choose a random mean between -10 and 10
        std::srand ( unsigned ( std::time(0) ) );

        // set some values:
        for (int i=0; i<10000; ++i) myVector.push_back((float(std::round((normal_dist(e2))*100)/100))); // 1 2 3 4 5 6 7 8 9
        // using built-in random generator:
        std::random_shuffle ( myVector.begin(), myVector.end() );

        // print out content:
        /*
        std::cout << "myVector contains:";
        for (std::vector<float>::iterator it=myVector.begin(); it!=myVector.end(); ++it)
            std::cout << ' ' << *it;
        */
        //randn(m1,mean,stddev);
    }


}

/**
 * @function main
 */
int main( int, char** argv )
{

    data_generator("vector");
    Mat src, dst;

    /// Load image
    src = imread( "../pics_dataset/lena.jpg", CV_LOAD_IMAGE_ANYCOLOR);

    if( !src.data )
    { return -1; }

    /// Separate the image in 3 places ( B, G and R )
    vector<Mat> bgr_planes;
    split( src, bgr_planes );

    /// Establish the number of bins
    int histSize = 2000;

    /// Set the ranges ( for B,G,R) )

    float range[] = { -20, 20 } ;
    const float* histRange = { range };

    bool uniform = true; bool accumulate = false;

    Mat b_hist, g_hist, r_hist;

    cv::Mat_<float> myVectorMat;

    for ( auto it = myVector.begin(); it != myVector.end(); it++ ) {
        myVectorMat.push_back(*it);
    }

    /// Compute the histograms:
    calcHist (&myVectorMat, 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
    //calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
    //calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

    assert(b_hist.rows == histSize);

    // Draw the histograms for B, G and R
    int hist_w = 1024; int hist_h = 768;
    int bin_w = cvRound( (double) hist_w/histSize );

    Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

    /// Normalize the result to [ 0, histImage.rows ]

    normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    //normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    //normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    double    max_val;
    cv::Point max_pt;

    cv::minMaxLoc(
            b_hist,    // input histogram
            NULL,       // don't care about the min value
            &max_val,   // place to put the maximum value
            NULL,       // don't care about the location of the min value
            &max_pt     // place to put the maximum value location (a cv::Point)
    );

    std::cout << "pt" << max_pt << " maxval  " << max_val << " bin " << (range[0] + max_pt.y*(range[1]-range[0])/histSize);



    /// Draw for each channel
    for( int i = 1; i < histSize; i++ )
    {
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
              Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
              Scalar( 255, 0, 0), 2, 8, 0  );
        /*
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                         Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                         Scalar( 0, 255, 0), 2, 8, 0  );
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                         Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                         Scalar( 0, 0, 255), 2, 8, 0  );
                         */

    }

    /// Display
    //namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
    //imshow("calcHist Demo", histImage );

    //waitKey(0);

    return 0;

}


int cpp_style_histogram() {
    data_generator("vector");
    Mat src, dst;

    /// Load image
    src = imread( "../pics_dataset/lena.jpg", CV_LOAD_IMAGE_ANYCOLOR);

    if( !src.data )
    { return -1; }

    /// Separate the image in 3 places ( B, G and R )
    vector<Mat> bgr_planes;
    split( src, bgr_planes );

    /// Establish the number of bins
    std::vector<int> histSize = {2000};

    /// Set the ranges ( for B,G,R) )

    ;
    std::vector<int> channels = {1};
    const std::vector<float> histRange = { { -20, 20 } };

    bool uniform = true; bool accumulate = false;

    Mat b_hist, g_hist, r_hist;

    cv::Mat_<float> myVectorMat;

    for ( auto it = myVector.begin(); it != myVector.end(); it++ ) {
        myVectorMat.push_back(*it);
    }

    /// Compute the histograms:
    calcHist (myVectorMat, channels, cv::noArray(), b_hist, histSize, histRange, accumulate );
    //calcHist (&myVectorMat, 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
    //calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
    //calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

    std::cout << b_hist;

    // Draw the histograms for B, G and R
    int hist_w = 1024; int hist_h = 768;
    int bin_w = cvRound( (double) hist_w/histSize[0] );

    Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

    /// Normalize the result to [ 0, histImage.rows ]
    normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    //normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    //normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    /// Draw for each channel
    for( int i = 1; i < histSize[0]; i++ )
    {
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
              Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
              Scalar( 255, 0, 0), 2, 8, 0  );
        /*
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                         Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                         Scalar( 0, 255, 0), 2, 8, 0  );
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                         Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                         Scalar( 0, 0, 255), 2, 8, 0  );
                         */

    }

    /// Display
    namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
    imshow("calcHist Demo", histImage );

    waitKey(0);

    return 0;

}