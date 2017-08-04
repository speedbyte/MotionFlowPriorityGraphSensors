#include<iostream>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<boost/filesystem.hpp>

int main ( int argc, char *argv[] ) {
    cv::Mat image, image1;
    boost::filesystem::path path = "../data/image.png";
    image = cv::imread(path.string());
    std::cout << "Hello World " << path.string();
    cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);
    cv::imshow("Original", image);
    cv::waitKey(0);
    int colSize = image.cols;

    cv::Mat M(100,100,CV_8U);  // create a 100*100 8 bit single channel matrix.
    cv::Mat_<char> M2(100,100);
    cv::Mat_<cv::Vec3b> img(240, 320, cv::Vec3b(0,255,0)); // allocate a 320*240 multi channel color image and fill it
    // with green.

    // Not a row, column but a column, row.
    for ( int i = 0; i < image.cols; i++ ) {
        image1 = cv::Vec3b(255,255,255);  // Draw a diagonal white line.
    }
    cv::namedWindow("Modified", CV_WINDOW_AUTOSIZE);
    cv::imshow("Modified", image1);
    cv::waitKey(0);
    return 0;
}

