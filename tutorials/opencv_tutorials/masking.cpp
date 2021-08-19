//
// Created by veikas on 31.08.18.
//



#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv/cv.hpp>

int main(int argc, char *argv[]) {

    cv::Size image_size(1200,200);
    cv::Size double_image_size(1200,600);

    cv::Mat circle(image_size, CV_8UC3);
    cv::Mat result(image_size, CV_8UC3);
    cv::Mat final(image_size, CV_8UC3);
    cv::Mat black_background(image_size, CV_8UC3);
    cv::Mat mask;

    cv::Mat output(double_image_size, CV_8UC3);

    //cv::randu(rectangle, 0, 255);

    cv::Mat rectangle(image_size, CV_8UC3);
    uchar r = 0;
    uchar b = 0;

    r = 0;
    b = 0;

    for (int k = 0; k < ( rectangle.rows - 1); k++) {
        for (int j = 0; j < (rectangle.cols -1 ); j++) {
            rectangle.at<cv::Vec3b>(k, j)[0] = b;
            rectangle.at<cv::Vec3b>(k, j)[1] = 0;
            rectangle.at<cv::Vec3b>(k, j)[2] = r;
            r = r + (uchar)2;
            b = b + (uchar)2;
            if (r > 254)
                r = 130;
        }
        if (b > 254)
            b = 46;
    }

    //rectangle = cv::Scalar(0,0,0);

    cv::circle(circle, cv::Point(image_size.width/2,image_size.height/2), 50, cv::Scalar(1,1,1), CV_FILLED);

    result = rectangle.mul(circle);

    cv::imshow("rectangle", result);
    cv::waitKey(0);

    cv::compare(circle, cv::Scalar(0,0,0), mask, CV_CMP_EQ);
    cv::imshow("rectangle", mask);
    cv::waitKey(0);

    result.copyTo(final, mask);


    cv::imshow("rectangle", final);
    cv::waitKey(0);



    rectangle = rectangle.mul(circle);

    cv::Mat roi1 = output.rowRange(0,200).colRange(0,1200);
    cv::Mat roi2 = output.rowRange(200,400).colRange(0,1200);
    cv::Mat roi3 = output.rowRange(400,600).colRange(0,1200);

    cv::inRange(circle, cv::Scalar(0,1,1), cv::Scalar(255,2,2), black_background);

    //black_background = (circle == 255);

    circle.copyTo(roi1);
    result.copyTo(roi2);
    black_background.copyTo(roi3);

    cv::imshow("circle", output);
    cv::waitKey(0);

    return 0;
}