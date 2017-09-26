

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <iostream>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>


#define KITTI_RAW_DATASET_PATH "../../../kitti_dataset/raw_dataset_with_calib/2011_09_28_drive_0016_sync/"

static const double pi = 3.14159265358979323846;




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

void make_video_from_png(boost::filesystem::path kitti_raw_dataset_path) {
    cv::VideoWriter write;
    cv::Mat temp_image;

    boost::filesystem::path video_path = kitti_raw_dataset_path;
    video_path += "video/" ;

    std::cout << video_path.string();

    std::string file_name, path;
    char file_name_char[10];
    int number = 0;
    std::string dir_path = kitti_raw_dataset_path.string() + "image_02/data/";
    if (boost::filesystem::exists(video_path) == 0) {
        throw("no video file");
    }

    do {
        sprintf(file_name_char, "0000000%03d", number);
        path = dir_path + std::string(file_name_char) + ".png";
        temp_image = cv::imread(path, cv::IMREAD_COLOR);
        if ( number == 0 ) {
            write.open((video_path.string()+"2011_09_28_drive_0016_sync.avi"), CV_FOURCC('D', 'I', 'V', 'X'), 30.0,
                       cv::Size(temp_image.cols, temp_image.rows), true);
        }
        write.write(temp_image);
        number++;
    } while ( boost::filesystem::exists(path) != 0);

    write.release();


}

int samples_lkdemo(boost::filesystem::path kitti_raw_dataset_path) {

    boost::filesystem::path video_path = kitti_raw_dataset_path;
    video_path += "video/2011_09_28_drive_0016_sync.avi" ;

    if (boost::filesystem::exists(video_path) == 0) {
        throw("no video file");
    }

    cv::Point2f point;
    bool addRemovePt = true;
    cv::VideoCapture cap;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
    cv::Size subPixWinSize(10, 10), winSize(31, 31);

    const int MAX_COUNT = 5;
    bool needToInit = true;
    int i = 0;


    std::cout << video_path.string();
    cap.open(video_path.string());

    if (!cap.isOpened()) {
        std::cout << "Could not initialize capturing...\n";
        return 0;
    }

    cv::namedWindow("LK Demo", 1);

    cv::Mat gray, prevGray, frame;
    std::vector<cv::Point2f> prev_pts;
    std::vector<cv::Point2f> next_pts;

    for (;;) {
        cap >> frame;
        if (frame.empty())
            break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        if ( prevGray.empty() ) {
            gray.copyTo(prevGray);
        }

        if (i == -1 || needToInit) {
            // automatic initialization
            cv::goodFeaturesToTrack(gray, next_pts, MAX_COUNT, 0.01, 10, cv::Mat(), 3, false, 0.04);
            cv::cornerSubPix(gray, next_pts, subPixWinSize, cv::Size(-1, -1), termcrit);

            addRemovePt = false;
            i = 0;

        } else if (!prev_pts.empty()) {
            std::vector<uchar> status;
            std::vector<float> err;
            cv::calcOpticalFlowPyrLK(prevGray, gray, prev_pts, next_pts, status,
                                     err, winSize, 3, termcrit, 0, 0.001);

            size_t i, k;
            for (i = k = 0; i < next_pts.size(); i++) {
                if (addRemovePt) {
                    if (cv::norm(point - next_pts[i]) <= 1) {
                        addRemovePt = false;
                        continue;
                    }
                }

                if (!status[i])
                    continue;

                next_pts[k++] = next_pts[i];
                cv::circle(frame, next_pts[i], 3, cv::Scalar(0, 255, 0), -1, 8);
                printf("X = %f , Y = %f \n", prev_pts[i].x, prev_pts[i].y);
            }
            next_pts.resize(k);
        }

        if (addRemovePt && next_pts.size() < (size_t) MAX_COUNT) {
            std::vector<cv::Point2f> tmp;
            tmp.push_back(point);
            cv::cornerSubPix(gray, tmp, winSize, cv::Size(-1, -1), termcrit);
            next_pts.push_back(tmp[0]);
            addRemovePt = false;
        }

        needToInit = false;
        cv::imshow("LK Demo", frame);

        char c = (char) cv::waitKey(30);
        if (c == 27)
            break;
        switch (c) {
            case 'r':
                needToInit = true;
                break;
            case 'c':
                prev_pts.clear();
                next_pts.clear();
                break;

        }
        i++;

        cv::swap(next_pts, prev_pts);
        cv::swap(prevGray, gray);
    }
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
    //start_video_capture(input_video_file);
    //start_video_capture(input_camera);

    boost::filesystem::path  raw_dataset_path = KITTI_RAW_DATASET_PATH;

    //make_video_from_png(raw_dataset_path);
    samples_lkdemo(raw_dataset_path);
}

