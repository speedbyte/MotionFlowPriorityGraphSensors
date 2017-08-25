#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

void cartToPolar() {
    std::vector<cv::Point2f> xy;
    xy.push_back(cv::Point2f(10, 20));
    xy.push_back(cv::Point2f(6, 8));
    xy.push_back(cv::Point2f(5, 5));

    cv::Mat_<float> xpts(xy.size(), 1, &xy[0].x, 2 * sizeof(float)); // matrix of x points
    cv::Mat_<float> ypts(xy.size(), 1, &xy[0].y, 2 * sizeof(float)); // matrix of y points

    // Proves that there is only one allocated data and Mat is simply pointing to the std::vector i.e it does not
    // have its own data.
    xy[0].x = 3;
    xy[0].y = 4;

    cv::Matx<float,3,1> magnitude;
    cv::Matx<float,3,1> angle;
    cv::cartToPolar(xpts, ypts, magnitude, angle);

    std::cout << "\nsamples\n" << xy;
    std::cout << "\nmagnitude\n" << magnitude.t();
    std::cout << "\nangle\n" << (angle.t() * ( 180. / CV_PI ))<< std::endl;
    //std::cout << "\ncovar: " << covar.at<float>(0,0);
    //cv::Mahalanobis(xpts, ypts, covar);

}

void polarToCart() {

    cv::Vec2f x,y;
    cv::Vec2f mag(5,6), angle(40,100);
    cv::polarToCart(mag, angle, x, y, true);
    std::cout << x[0]+x[1] << y[0]+y[1] << std::endl;

}

/**
 * Given any number of vectors, the function will compute
 * 1. The mean of the gaussian approximaiton to the distribution of the sample points.
 * 2. The covariance for the Guassian approximation to the distribution of the sample points.
 *
 */
void calcCovarMatrix() {

    cv::Mat_<float> x(3,3);  x << -1, 1, 2, -2, 3, 1 , 4, 0, 3;
    cv::Mat_<float> covar, mean;
    std::cout << "\nsamples\n" << x.t();
    calcCovarMatrix( x, covar, mean, cv::COVAR_NORMAL|cv::COVAR_ROWS, CV_32FC1);
    std::cout << "\nMean\n" << mean << "\nCovar\n" << covar << std::endl;
}

void linearPolar() {
    cv::Mat src = cv::imread("my.png", 0); // read a grayscale img
    cv::Mat dst; // empty.
    cv::linearPolar(src,dst, cv::Point(src.cols/2,src.rows/2), 120, cv::INTER_CUBIC );
    cv::imshow("linear", dst);
    cv::waitKey();
}

void logPolar() {
    cv::Mat src = cv::imread("my.png", 0); // read a grayscale img
    cv::Mat dst; // empty.
    cv::logPolar(src,dst,cv::Point(src.cols/2,src.rows/2),40,cv::INTER_CUBIC );
    cv::imshow("linear", dst);
    cv::waitKey();
}

/*
    cv::calcCovarMatrix();
    cv::meanStdDev();
    cv::polarToCart();
    cv::countNonZero();
    cv::dct();
    cv::dft();
    cv::eigen();
    cv::determinant(); // of a square matrix
    cv::magnitude(); // magnitudes from a 2D vector field.
    cv::Mahalanobis(); // compute Mahalonobis distance between two vectors.
*/

int main ( int argc, char *argv[]) {

    std::cout << "cartToPolar----------------------------------------------" << std::endl;
    cartToPolar();
    std::cout << "polarToCart----------------------------------------------" << std::endl;
    polarToCart();
    std::cout << "calcCovarMatrix----------------------------------------------" << std::endl;
    calcCovarMatrix();
    std::cout << "----------------------------------------------" << std::endl;
    return 0;
}

