

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

void LK(int argc, char *argv) {

    //Zeitsteuerung �ber clock()
    clock_t start, start2, end;
    start2 = clock();

    int i = 0;
    char c = NULL;

//    while(c != 'q') {
//    }

    //fprintf(datei,"Loop;Zeit[ms];Länge;LängeX;LängeY\n");

    //zeiten fur die ausgabe
    int t_0 = 0, t_1;

    cv::Point2f point;
    bool addRemovePt = false;

    cv::VideoCapture input_video;
    cv::Mat image;
    input_video.open("../../../video_dataset/megamind.avi");

    /* Read the video's frame size out of the Cam */
    CvSize frame_size;
    frame_size.height =	(int) input_video.get(CV_CAP_PROP_FRAME_HEIGHT );
    frame_size.width =	(int) input_video.get(CV_CAP_PROP_FRAME_WIDTH );

    cvNamedWindow("Optical Flow from Cam 1", 0);
    cvNamedWindow("Optical Flow from Cam 2", 0);

    printf("Fenster AN \n");

    //Schreibt das Video mit
    boost::filesystem::path VideoPath = "Mitschnitt.mpeg";
    boost::filesystem::path BildPath = "Bild.jpg";
    boost::filesystem::path TextPath = "Werte.txt";
    cv::VideoWriter video_out;
    video_out.open(VideoPath.string(),CV_FOURCC('P','I','M','1'),25,frame_size);
    printf("Writer eingerichtet\n");


    while ( cv::waitKey(1) != 'q' && cap.isOpened() ) {
        cap.grab();
        cap.retrieve(image);
    }
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
    cv::Size subPixWinSize(10, 10), winSize(31, 31);

    const int MAX_COUNT = 500;
    bool needToInit = true;
    int i = 0;



    for (;;) {
        cap >> frame;
        if (frame.empty())
            break;

        frame.copyTo(image);
        cvtColor(image, gray, cv::COLOR_BGR2GRAY);


        if (i == 20 || needToInit) {
            // automatic initialization
            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3,
                                0, 0.04);
            cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1),
                         termcrit);
            addRemovePt = false;
            i = 0;
        } else if (!points[0].empty()) {
            vector<uchar> status;
            vector<float> err;
            if (prevGray.empty())
                gray.copyTo(prevGray);
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status,
                                 err, winSize, 3, termcrit, 0, 0.001);
            size_t i, k;
            for (i = k = 0; i < points[1].size(); i++) {
                if (addRemovePt) {
                    if (norm(point - points[1][i]) <= 5) {
                        addRemovePt = false;
                        continue;
                    }
                }

                if (!status[i])
                    continue;

                points[1][k++] = points[1][i];
                circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
            }
            points[1].resize(k);
        }

        if (addRemovePt && points[1].size() < (size_t) MAX_COUNT) {
            vector<Point2f> tmp;
            tmp.push_back(point);
            cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
            points[1].push_back(tmp[0]);
            addRemovePt = false;
        }

        needToInit = false;
        cv::imshow("LK Demo", image);

        char c = (char) waitKey(30);
        if (c == 27)
            break;
        switch (c) {
            case 'r':
                needToInit = true;
                break;
            case 'c':
                points[0].clear();
                points[1].clear();
                break;

        }
        i++;

        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);
    }
}


int main (int argc, char *argv[]) {

    cv::VideoCapture video_read;
    boost::filesystem::path input_video_file = "../../../video_dataset/megamind.avi";
    try {
        if (boost::filesystem::exists(input_video_file) == 0) {
            throw ("No file exists");
        }
    }
    catch(std::string &e) {
        std::cout << "Error in reading video flle";
        return 0;
    }
    video_read.open(input_video_file.string());
    //video_read.open(0);
    //frame properties : fps 30 ; width : 640 ; height : 480
    //codec GPJM

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