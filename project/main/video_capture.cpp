

#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv/cv.hpp>
#include "InputOutput.h"


extern void salt(cv::Mat image, int n);


void canny(cv::Mat& img, cv::Mat& out) {
    // Convert to gray
    if (img.channels()==3)
        cv::cvtColor(img,out,CV_BGR2GRAY);
    // Compute Canny edges
    cv::Canny(out,out,100,200);
    // Invert the image
    cv::threshold(out,out,128,255,cv::THRESH_BINARY_INV);
}



void video_capture(cv::Mat& image_manual_bare1) {
    cv::VideoWriter writer;

    // Create video procesor instance
    VideoProcessor processor;

    // Create feature tracker instance
    FeatureTracker tracker;

    char codec[4];
    writer.write(image_manual_bare1); // add the frame to the video file

    // Open video file
    processor.setInput("../../../video_dataset/Megamind.avi");
    processor.setFrameProcessor(canny);
    processor.setOutput("../../../video_dataset/Megamind_out.avi");
    // Start the process
    processor.run();

    processor.setOutput("bikeOut",  //prefix
                        ".jpg",     // extension
                        3,          // number of digits
                        0);// starting index

    cv::VideoCapture capture("../../../video_dataset/Megamind.avi");
    // check if video successfully opened
    if (!capture.isOpened())
        throw("capture failed");

    // Get the frame rate
    double rate= capture.get(CV_CAP_PROP_FPS);

    bool stop(false);
    cv::Mat frame; // current video frame
    cv::namedWindow("Extracted Frame");

    // Delay between each frame in ms
    // corresponds to video frame rate
    int delay= 1000/rate;

    // for all frames in video
    while (!stop) {

        // read next frame if any
        if (!capture.read(frame))
            break;

        cv::imshow("Extracted Frame",frame);

        // introduce a delay
        // or press key to stop
        if (cv::waitKey(delay)>=0)
            stop= true;
    }

    // Close the video file.
    // Not required since called by destructor
    capture.release();
    //return 0;
    // Open video file
    processor.setInput("../../../video_dataset/Megamind.avi");

    // set frame processor
    processor.setFrameProcessor(&tracker);

    // Declare a window to display the video
    processor.displayOutput("Tracked Features");

    // Play the video at the original frame rate
    //processor.setDelay(1000./processor.getFrameRate());

    // Start the process
    processor.run();

    // open the image
    cv::Mat image= cv::imread("lena.jpg");

    // call function to add noise
    salt(image,3000);

    // display image
    cv::namedWindow("Image");
    cv::imshow("Image",image);
}