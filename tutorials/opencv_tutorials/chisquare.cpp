//
// Created by veikas on 06.06.18.
//
#include <iostream>
#include <cmath>

//You need OpenCV for this demo
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

cv::RotatedRect getErrorEllipse(float chisquare_val, cv::Point2f mean, cv::Mat covmat);

int main()
{

    //Covariance matrix of our data
    cv::Mat covmat(2,2, CV_32FC1 );
    covmat.at<float>(0,0) = 500.5886;
    covmat.at<float>(0,1) = 400.6111;
    covmat.at<float>(1,0) = 400.6111;
    covmat.at<float>(1,1) = 500.7801;

    //The mean of our data
    cv::Point2f mean(160,120);

    //Calculate the error ellipse for a 95% confidence intervanl
    cv::RotatedRect ellipse = getErrorEllipse(2.4477, mean, covmat);

    //Show the result
    cv::Mat visualizeimage(240, 320, CV_8UC1, cv::Scalar::all(0));
    cv::ellipse(visualizeimage, ellipse, cv::Scalar::all(255), 2);
    cv::imshow("EllipseDemo", visualizeimage);
    cv::waitKey();

}

cv::RotatedRect getErrorEllipse(float chisquare_val, cv::Point2f mean, cv::Mat covmat){

    //Get the eigenvalues and eigenvectors
    cv::Mat_<float> eigenvectors(2,2);
    cv::Mat_<float> eigenvalues(1,2);
    cv::eigen(covmat, eigenvalues, eigenvectors);

    std::cout << eigenvectors << "\n" << eigenvalues ;
    //Calculate the angle between the largest eigenvector and the x-axis
    float angle = std::atan2(eigenvectors(0,1), eigenvectors(0,0));

    //Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
    if(angle < 0)
        angle += 6.28318530718;

    //Conver to degrees instead of radians
    angle = 180*angle/3.14159265359;

    //Calculate the size of the minor and major axes
    float halfmajoraxissize=chisquare_val*sqrt(eigenvalues(0));
    float halfminoraxissize=chisquare_val*sqrt(eigenvalues(1));

    //Return the oriented ellipse
    //The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
    return cv::RotatedRect(mean, cv::Size2f(halfmajoraxissize, halfminoraxissize), -angle);

}
