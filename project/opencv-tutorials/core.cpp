#include <opencv2/core.hpp>
#include <iostream>

using namespace cv;

int main ( int argc, char *argv[]) {
    std::vector<cv::Point2f> vec;
// points or a circle
    for( int i = 0; i < 30; i++ )
        vec.push_back(cv::Point2f((float)(100 + 30*cos(i*CV_PI*2/5)),
                              (float)(100 - 30*sin(i*CV_PI*2/5))));
    cv::transform(vec, vec, cv::Matx23f(0.707, -0.707, 10, 0.707, 0.707, 20));
    return 0;
}
