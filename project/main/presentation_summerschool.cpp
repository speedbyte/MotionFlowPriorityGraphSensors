
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <boost/filesystem.hpp>

#define RAW_DATASET_PATH "../../../kitti_dataset/raw_dataset_with_calib/2011_09_28_drive_0016_sync/"

#include "input_output.h"

extern void salt(cv::Mat image, int n);

//template <int row, int col>
class GridLayout {
public:

    GridLayout(const cv::Mat& m1, const cv::Mat& m2):m_upper_image(m1),m_lower_image(m2)  {
        m_row = m_upper_image.rows;
        m_col = m_upper_image.cols;
        m_grid.create(m_row*2,m_col,m_upper_image.type());
    }

    const cv::Mat& render() {

        cv::Rect roi_upper(0,0,m_col,m_row);
        cv::Rect roi_lower(0,m_row,m_col,m_row);

        cv::Mat roi_grid_upper(m_grid, roi_upper);
        cv::Mat roi_grid_lower(m_grid, roi_lower );

        m_upper_image.copyTo(roi_grid_upper);
        m_lower_image.copyTo(roi_grid_lower);

        return m_grid;
    }

private:
    // declaring new cv::Mat headers
    cv::Mat m_upper_image;
    cv::Mat m_lower_image;
    int m_row;
    int m_col;
    cv::Mat m_grid;
};

class ImageShow {
public:
    void show(const cv::Mat& img) {
        cv::namedWindow("Grid", CV_WINDOW_AUTOSIZE);
        cv::imshow("Grid", img);
    }
};

int main ( int argc, char *argv[]) {
    // Thread 2: Read two kitti image files without rain. The two image files are from kitti

    boost::filesystem::path kitti_dataset_path, kitti_image_name1,
            kitti_image_name2, kitti_image_path1, kitti_image_path2;

    kitti_dataset_path += RAW_DATASET_PATH;
    kitti_dataset_path += "image_02/data/";
    kitti_image_name1 += "0000000169.png";
    kitti_image_name2 += "0000000170.png";
    kitti_image_path1 += kitti_dataset_path;
    kitti_image_path1 += kitti_image_name1;
    kitti_image_path2 += kitti_dataset_path;
    kitti_image_path2 += kitti_image_name2;

    try {
        if ( !boost::filesystem::exists(kitti_image_path1) ) {
            throw ("file not found");
        }
    }
    catch ( const char* exception) {
        std::cout << kitti_image_path1.string() << ":" <<  exception;
        exit(0);
    }

    try {
        if ( !boost::filesystem::exists(kitti_image_path2) ) {
            throw ("file not found");
        }
    }
    catch ( const char* exception) {
        std::cout << kitti_image_path2.string() << ":" <<  exception;
        exit(0);
    }

    std::cout << kitti_image_path1 << std::endl << kitti_image_path2 << std::endl;;

    cv::Mat image_manual_bare1(cv::imread(kitti_image_path1.string(), CV_LOAD_IMAGE_COLOR));
    cv::Mat image_manual_bare2(cv::imread(kitti_image_path2.string(), CV_LOAD_IMAGE_COLOR));

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
    writer.write(frame); // add the frame to the video file

    processor.getCodec(codec);
    std::cout << "Codec: " << codec[0] << codec[1] << codec[2] << codec[3] << std::endl;
    writer.open(outputFile, // filename
                codec,          // codec to be used
                framerate,      // frame rate of the video
                frameSize,      // frame size
                isColor);       // color video?
    // Open the video file



    // Create instance
    VideoProcessor processor;

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
    processor.setDelay(1000./processor.getFrameRate());

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

