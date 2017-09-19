

#include <opencv2/videoio.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <iostream>
#include <opencv/cv.hpp>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>

void start_video_capture(boost::filesystem::path input_video_file) {

    cv::VideoCapture video_read;
    if ( input_video_file.empty() ) {
        video_read.open(0);
        //frame properties : fps 30 ; width : 640 ; height : 480
        //codec GPJM
    }
    else {
        video_read.open(input_video_file.string());
    }

    int width = (int)video_read.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = (int)video_read.get(cv::CAP_PROP_FRAME_HEIGHT);
    int fps = (int)video_read.get(cv::CAP_PROP_FPS);
    printf("frame properties : fps %d ; width : %d ; height : %d\n", fps, width, height);

    unsigned f = (unsigned)video_read.get(cv::CAP_PROP_FOURCC);
    printf("codec %c%c%c%c\n", (char)(f>>24), (char)(f>>16), (char)(f>>8), (char)f);

    cv::Mat image(height, width, CV_8UC3, cv::Scalar(255,255,255));
    cv::namedWindow("frame", CV_WINDOW_AUTOSIZE);

    while(char(cv::waitKey(1)) != 'q' && video_read.isOpened()) {
        video_read.grab();
        // do the heavier decoding in the second step after the grab is successful.
        video_read.retrieve(image);
        //video_read >> image;
        if (image.empty()) {
            break;
        }
        cv::imshow("frame", image);
    }

    cv::VideoWriter video_out;
    boost::filesystem::path output_video_file("../../../video_dataset/my_video.avi");
    video_out.open(
            output_video_file.string(),
            CV_FOURCC('D','I','V','X'),   // MPEG-4 codec
            30.0,                         // Frame rate (FPS)
            cv::Size( height, width),         // Write out frames at 640x480 resolution
            true                          // Expect only color frames
    );
    video_out.write(image);

    video_read.release();
    video_out.release();

}


int main (int argc, char *argv[]) {

    boost::filesystem::path input_video_file = "../../../video_dataset/megamind.avi";
    boost::filesystem::path input_camera;
    try {
        if (boost::filesystem::exists(input_video_file) == 0) {
            throw ("No file exists");
        }
    }
    catch(std::string &e) {
        std::cout << "Error in reading video flle";
        return 0;
    }
    start_video_capture(input_video_file);
    start_video_capture(input_camera);
}