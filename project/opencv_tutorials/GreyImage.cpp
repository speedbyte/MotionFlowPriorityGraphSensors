#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "GreyImage.hpp"

using namespace cv;

int greyImage( Mat image )

{
 Mat gray_image;
 cvtColor( image, gray_image, CV_BGR2GRAY );

 imwrite( "/local/pics/Gray_Image.jpg", gray_image );

 namedWindow( "Original Image", CV_WINDOW_AUTOSIZE );
 namedWindow( "Gray image", CV_WINDOW_AUTOSIZE );

 imshow( "Original Image", image );
 imshow( "Gray image", gray_image );

 waitKey(0);

 return 0;
}
