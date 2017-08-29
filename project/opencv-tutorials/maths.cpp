#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

void meanStdDeviation() {
    cv::Mat_<cv::Vec4i> m0(1,4),m1(1,4); // fills all the channel. Mean and stddev will be calcluated on ind channels.
    cv::randu(m0, 0, 4);
    //m0 << 6,5,13,8;
    cv::Scalar mean, stddev;
    cv::meanStdDev(m0,mean,stddev);
    cv::randn(m1,mean,stddev);
    std::cout << "Mean " << mean << " Std Dev " << stddev << "\nArray " << m0 << "\nArray " << m1;
}

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
    cv::Vec2f mag(7,5), angle(200,70);
    cv::polarToCart(mag, angle, x, y, true);
    std::cout << x[0]+x[1] << " " << y[0]+y[1] << std::endl;

}

/**
 * Given any number of vectors, the function will compute
 * 1. The mean of the gaussian approximaiton to the distribution of the sample points.
 * 2. The covariance for the Guassian approximation to the distribution of the sample points.
 *
 */
void calcCovarMatrix() {

    cv::Mat_<uchar> samples(2,9);  samples << 1,3,2,5,8,7,12,2,4,8,6,9,4,3,3,2,7,7;
    cv::Mat_<uchar> x_sample(1,9);  x_sample << 1,3,2,5,8,7,12,2,4;
    cv::Mat_<uchar> y_sample(1,9);  y_sample << 8,6,9,4,3,3,2,7,7;
    std::vector<cv::Mat> matPtr;
    matPtr.push_back(x_sample);
    matPtr.push_back(y_sample);
    cv::Mat_<float> covar, mean;
    std::cout << "\nsamples\n" << samples;
    //cv::calcCovarMatrix( &matPtr, 2, covar, mean, cv::COVAR_NORMAL|cv::COVAR_ROWS, CV_32FC1);
    cv::calcCovarMatrix( samples, covar, mean, cv::COVAR_NORMAL|cv::COVAR_COLS|cv::COVAR_SCALE, CV_32FC1);
    std::cout << "\nMean\n" << mean << "\nCovar\n" << covar << std::endl;
}

void linearPolar() {
    cv::Mat src = cv::imread("../../../pics-dataset/lena.png", 0); // read a grayscale img
    cv::Mat dst; // empty.
    cv::linearPolar(src,dst, cv::Point(src.cols/2,src.rows/2), 120, cv::INTER_CUBIC );
    cv::imshow("linear", dst);
    cv::waitKey(0);
}

void logPolar() {
    cv::Mat src = cv::imread("my.png", 0); // read a grayscale img
    cv::Mat dst; // empty.
    cv::logPolar(src,dst,cv::Point(src.cols/2,src.rows/2),40,cv::INTER_CUBIC );
    cv::imshow("linear", dst);
    cv::waitKey();
}

void solveLinear() {
    cv::Matx31f rhs(3,0,-2);
    cv::Matx33f coefficients (1,1,1,1,2,3,1,3,4), transpose;
    float determinant;
    cv::Matx<float,3,1> result;
    cv::Matx<cv::Complexf,3,1> result_manual;
    cv::solve(coefficients, rhs, result);
    result_manual = coefficients.solve(rhs); // equivalent to coefficients.inv()*rhs and this can also handle Complexf
    std::cout << result << std::endl << result_manual;
}

void solvePolynomial() {
    //cv::Matx<cv::Complexf,2,1> roots;
    cv::Mat roots;
    cv::Vec3f coefficients(6,-5,1);
    //cv::Matx<cv::Complexf,3,1> result;
    //cv::Matx<cv::Complexf,3,1> result_manual;
    cv::solvePoly(coefficients, roots, 300 );
    std::cout << roots << std::endl;
}

void matrixOperations() {
    float determinant;
    cv::Matx33f coefficients (1,1,1,1,2,3,1,3,4), transpose;
    transpose = coefficients.t(); // equivalent to coefficients.inv()*rhs;
    determinant = cv::determinant(coefficients);
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
    //cartToPolar();
    std::cout << "polarToCart----------------------------------------------" << std::endl;
    //polarToCart();
    std::cout << "calcCovarMatrix----------------------------------------------" << std::endl;
    calcCovarMatrix();
    std::cout << "solveLinear----------------------------------------------" << std::endl;
    solveLinear();
    std::cout << "solvePoly----------------------------------------------" << std::endl;
    solvePolynomial();
    std::cout << "mean and std dev----------------------------------------------" << std::endl;
    meanStdDeviation();
    std::cout << "linearPolar----------------------------------------------" << std::endl;
    linearPolar();
    return 0;
}

