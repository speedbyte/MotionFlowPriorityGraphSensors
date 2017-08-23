// Based on http://mehdi.rabah.free.fr/SSIM/SSIM.cpp
// Converted to OpenCV C++ API by B...

/*
 * The equivalent of Zhou Wang's SSIM matlab code using OpenCV.
 * from http://www.cns.nyu.edu/~zwang/files/research/ssim/index.html
 * The measure is described in :
 * "Image quality assessment: From error measurement to structural similarity"
 * C++ code by Rabah Mehdi. http://mehdi.rabah.free.fr/SSIM
 *
 * This implementation is under the public domain.
 * @see http://creativecommons.org/licenses/publicdomain/
 * The original work may be under copyrights. 
 */

//#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#include <highgui.h>
#include "opencv2/highgui/highgui.hpp"

//#include <iostream.h>
#include <iostream>
using namespace std;
/*
 * Parameters : complete path to the two image to be compared
 * The file format must be supported by your OpenCV build
 */
int main(int argc, char** argv)
{
    if(argc!=3)
        return -1;
    // default settings
    double C1 = 6.5025, C2 = 58.5225;
    /***************************** INITS **********************************/
    cv::Mat img1 = cv::imread(argv[1]);
    cv::Mat img2 = cv::imread(argv[2]);
    if(img1.empty() || img2.empty())
        return -1;
    img1.convertTo(img1, CV_32F);
    img2.convertTo(img2, CV_32F);
    cv::Mat img1_sq;
    cv::Mat img2_sq;
    cv::Mat img1_img2;
    cv::pow(img1, 2, img1_sq);
    cv::pow(img1, 2, img1_sq);
    cv::multiply(img1, img2, img1_img2);
    cv::Mat mu1;
    cv::Mat mu2;
    cv::Mat mu1_sq;
    cv::Mat mu2_sq;
    cv::Mat mu1_mu2;
    cv::Mat sigma1_sq;
    cv::Mat sigma2_sq;
    cv::Mat sigma12;
    cv::Mat ssim_map;

    /*************************** END INITS **********************************/
    //////////////////////////////////////////////////////////////////////////
    // PRELIMINARY COMPUTING

    cv::GaussianBlur(img1, mu1, cv::Size(11, 11), 1.5);
    cv::GaussianBlur(img2, mu2, cv::Size(11, 11), 1.5);
    cv::pow(mu1, 2, mu1_sq);
    cv::pow(mu2, 2, mu2_sq);
    cv::multiply(mu1, mu2, mu1_mu2);
    cv::GaussianBlur(img1_sq, sigma1_sq, cv::Size(11, 11), 1.5);
    cv::addWeighted(sigma1_sq, 1.0, mu1_sq, -1.0, 0.0, sigma1_sq);
    cv::GaussianBlur(img2_sq, sigma2_sq, cv::Size(11, 11), 1.5);
    cv::addWeighted(sigma2_sq, 1.0, mu2_sq, -1.0, 0.0, sigma2_sq);
    cv::GaussianBlur(img1_img2, sigma12, cv::Size(11, 11), 1.5);
    cv::addWeighted(sigma12, 1.0, mu1_mu2, -1.0, 0.0, sigma12);

    //////////////////////////////////////////////////////////////////////////
    // FORMULA

    cv::Mat temp3 = 2 * mu1_mu2 + C1;
    temp3 = temp3.mul(2 * sigma12 + C2);
    cv::Mat temp1 = mu1_sq + mu2_sq + C1;
    temp1 = temp1.mul(sigma1_sq + sigma2_sq + C2);
    cv::divide(temp3, temp1, ssim_map);
    cv::Scalar index_scalar = cv::mean(ssim_map);
    cout << "(R, G & B SSIM index)" << endl ;
    cout << index_scalar.val[2] * 100 << "%" << endl ;
    cout << index_scalar.val[1] * 100 << "%" << endl ;
    cout << index_scalar.val[0] * 100 << "%" << endl ;
    return 0;
}
