
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <boost/filesystem.hpp>


#include "InputOutput.h"
#include "GridLayout.h"

#define RAW_DATASET_PATH "../../../kitti_dataset/raw_dataset_with_calib/2011_09_28_drive_0016_sync/"

extern void salt(cv::Mat image, int n);

using boost_path=boost::filesystem::path;
extern boost_path get_file(const boost_path &dataset_path, const boost_path &subfolder, const boost_path
&file_name);


int main ( int argc, char *argv[]) {
    // Thread 2: Read two kitti image files without rain. The two image files are from kitti

    boost::filesystem::path kitti_full_image_path1, kitti_full_image_path2;

    kitti_full_image_path1 = get_file(RAW_DATASET_PATH, "image_02/data/", "0000000169.png");
    kitti_full_image_path2 = get_file(RAW_DATASET_PATH, "image_02/data/", "0000000170.png");

    std::cout << kitti_full_image_path1 << std::endl << kitti_full_image_path2 << std::endl;

    cv::Mat image_manual_bare1(cv::imread(kitti_full_image_path1.string(), CV_LOAD_IMAGE_COLOR));
    cv::Mat image_manual_bare2(cv::imread(kitti_full_image_path2.string(), CV_LOAD_IMAGE_COLOR));

    assert(image_manual_bare1.empty() == 0);
    assert(image_manual_bare2.empty() == 0);

    GridLayout grid_manual(image_manual_bare1, image_manual_bare2);
    cv::Mat image_manual_bare_grid = grid_manual.render();
    ImageShow grid_manual_show;
    grid_manual_show.show(image_manual_bare_grid);


    cv::waitKey(0);
    cv::destroyAllWindows();

    cv::VideoWriter writer;

    // Create video procesor instance
    VideoProcessor processor;

    // Create feature tracker instance
    FeatureTracker tracker;

    char codec[4];

    writer.write(image_manual_bare1); // add the frame to the video file




    // Open video file
    processor.setInput("bike.avi");
    processor.setFrameProcessor(canny);
    processor.setOutput("bikeOut.avi");
    // Start the process
    processor.run();

    processor.setOutput("bikeOut",  //prefix
                        ".jpg",     // extension
                        3,          // number of digits
                        0);// starting index

    cv::VideoCapture capture("lena.avi");
    // check if video successfully opened
    if (!capture.isOpened())
        return 1;

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
    processor.setInput("../bike.avi");

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

