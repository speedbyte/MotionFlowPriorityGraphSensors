#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


//cv::GaussianBlur(inputImage,outputImage,Size(1,1),0,0)

/*
 Various border types, image boundaries are denoted with '|'

 * BORDER_REPLICATE:     aaaaaa|abcdefgh|hhhhhhh
 * BORDER_REFLECT:       fedcba|abcdefgh|hgfedcb
 * BORDER_REFLECT_101:   gfedcb|abcdefgh|gfedcba
 * BORDER_DEFAULT:       BORDER_REFLECT_101
 * BORDER_WRAP:          cdefgh|abcdefgh|abcdefg
 * BORDER_CONSTANT:      iiiiii|abcdefgh|iiiiiii  with some specified 'i'
 */
/**
The function convolves the source image with the specified Gaussian kernel. In-place filtering is
        supported.

@param ksize Gaussian kernel size. ksize.width and ksize.height can differ but they both must be
positive and odd. Or, they can be zero's and then they are computed from sigma.
@param sigmaX Gaussian kernel standard deviation in X direction.
@param sigmaY Gaussian kernel standard deviation in Y direction; if sigmaY is zero, it is set to be
        equal to sigmaX, if both sigmas are zeros, they are computed from ksize.width and ksize.height,
respectively (see cv::getGaussianKernel for details); to fully control the result regardless of
        possible future modifications of all this semantics, it is recommended to specify all of ksize,
        sigmaX, and sigmaY.
@param borderType pixel extrapolation method, see cv::BorderTypes

@sa  sepFilter2D, filter2D, blur, boxFilter, bilateralFilter, medianBlur
 */

void GaussianBlur() {
    cv::Mat inputImage, outputImage;
    inputImage = cv::imread("../../../pics_dataset/lena.png");
    cv::getGaussianKernel(3,0,0);
    cv::GaussianBlur(inputImage,outputImage,cv::Size(3,3),0,0,cv::BORDER_DEFAULT);
}

int filter2D() {
    cv::Mat src, dst;

    cv::Mat kernel;
    cv::Point anchor;
    double delta;
    int ddepth;
    int kernel_size;
    char* window_name = "filter2D Demo";

    int c;

/// Load an image
    src = cv::imread("../../../pics_dataset/lena.png", CV_LOAD_IMAGE_COLOR); //CV_LOAD_IMAGE_COLOR=1

    if( !src.data )
    { return -1; }

/// Create window
    cv::namedWindow( window_name, CV_WINDOW_AUTOSIZE );

/// Initialize arguments for the filter
    anchor = cv::Point( -1, -1 );
    delta = 0;
    ddepth = -1;

/// Loop - Will filter the image with different kernel sizes each 0.5 seconds
    int ind = 0;
    while( true )
    {
        c = cv::waitKey(500);
/// Press 'ESC' to exit the program
        if( (char)c == 27 )
        { break; }

/// Update kernel size for a normalized box filter
        kernel_size = 3 + 2*( ind%5 );
        kernel = cv::Mat::ones( kernel_size, kernel_size, CV_32F )/ (float)(kernel_size*kernel_size);

/// Apply filter
        cv::filter2D(src, dst, ddepth , kernel, anchor, delta, cv::BORDER_DEFAULT );
        cv::imshow( window_name, dst );
        ind++;
    }
    return 0;
}

int main ( int argc, char *argv[]) {
    filter2D();
}