

#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv/cv.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
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


struct Points {
    cv::Point2f p1;
    cv::Point2f p2;
};


void video_capture(boost::filesystem::path kitti_raw_dataset_path) {

    boost::filesystem::path video_path = kitti_raw_dataset_path;
    video_path += "video/2011_09_28_drive_0016_sync.avi" ;

    if (boost::filesystem::exists(video_path) == 0) {
        throw("no video file");
    }

    cv::Point2f point;
    cv::VideoCapture cap;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
    cv::Size subPixWinSize(10, 10), winSize(31, 31);

    const int MAX_COUNT = 500;
    bool needToInit = true;
    int i = 0;


    std::cout << video_path.string();
    cap.open(video_path.string());

    if (!cap.isOpened()) {
        std::cout << "Could not initialize capturing...\n";
        return;
    }

    cv::namedWindow("LK Demo", 1);

    cv::Mat gray, prevGray, frame;
    cv::Mat pyramid1, pyramid2;
    std::vector<cv::Point2f> prev_pts;
    std::vector<cv::Point2f> next_pts;

    //Zeitsteuerung über clock()
    clock_t start, start2, end;
    int LOOPtime = 100;
    start = clock();

    int RecordFlag;

    CvFont Font_= cvFont(0.5,1); //Vareablen zur Textausgabe
    char str[256];
    char str2[256];

    //Schreibt das Video mit
    cv::VideoWriter video_out;
    boost::filesystem::path VideoOutFile = video_path.parent_path();
    VideoOutFile += "/OpticalFlow.avi";
    boost::filesystem::path BildOutFile = video_path.parent_path();
    BildOutFile += "/Bild.jpg";
    boost::filesystem::path TextOutFile = video_path.parent_path();
    TextOutFile += "/Werte.txt";

    FILE *datei;
    char fileName[200];
    sprintf(fileName, "%s", TextOutFile.string().data());
    memcpy(fileName, TextOutFile.string().data(), TextOutFile.string().size());
    datei = std::fopen(fileName, "w");
    fprintf(datei,"Loop;Zeit[ms];Magnitude;Length_X;Length_Y\n");

    cv::Size frame_size;
    frame_size.height =	(int) cap.get(CV_CAP_PROP_FRAME_HEIGHT );
    frame_size.width =	(int) cap.get(CV_CAP_PROP_FRAME_WIDTH );
    video_out.open(VideoOutFile.string(),CV_FOURCC('P','I','M','1'),25,frame_size);
    printf("Writer eingerichtet\n");

    cv::Mat RecordFrame;
    RecordFrame.create(frame_size, CV_8UC3);

    for (;;) {
        cap >> frame;
        if (frame.empty())
            break;

        pyramid1.create(frame_size, CV_8UC1);
        pyramid2.create(frame_size, CV_8UC1);

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        if ( prevGray.empty() ) {
            gray.copyTo(prevGray);
        }

        if (i == -1 || needToInit) {
            // automatic initialization
            cv::goodFeaturesToTrack(gray, next_pts, MAX_COUNT, 0.01, 10, cv::Mat(), 3, false, 0.04);
            cv::cornerSubPix(gray, next_pts, subPixWinSize, cv::Size(-1, -1), termcrit);

            i = 0;

        } else if (!prev_pts.empty()) {
            std::vector<uchar> status;
            std::vector<float> err;
            cv::calcOpticalFlowPyrLK(prevGray, gray, prev_pts, next_pts, status,
                                     err, winSize, 3, termcrit, 0, 0.001);

            unsigned i, k;
            std::vector<std::vector<float>> all_x;
            std::vector<std::vector<float>> all_y;

            std::vector<float> displacement_vector_x;
            std::vector<float> displacement_vector_y;

            for (i = k = 0; i < next_pts.size(); i++) {

                if (!status[i])
                    continue;

                next_pts[k++] = next_pts[i];
                //cv::circle(frame, next_pts[i], 3, cv::Scalar(0, 255, 0), -1, 8);


                cv::Point frame_center;
                frame_center.x = frame_size.width/2;
                frame_center.y = frame_size.height/2;

                displacement_vector_x.clear();
                displacement_vector_y.clear();
                //std::vector<float>().swap(displacement_vector_x);

                displacement_vector_x.push_back(cv::abs(prev_pts[i].x - next_pts[i].x));
                displacement_vector_y.push_back(cv::abs(prev_pts[i].y - next_pts[i].y));

                double magnitude = cv::norm(prev_pts[i] - next_pts[i]);
                std::cout << "Mag" << magnitude << std::endl;

                // Textausgabe der x y Werte
                sprintf(str, "X = %f ", (displacement_vector_x[i]));
                cv::putText(frame, str, cv::Point(frame_size.width-40, 10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));

                sprintf(str, "Y = %f ", (displacement_vector_y[i]));
                cv::putText(frame, str, cv::Point(frame_size.width-40, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));

                cv::arrowedLine(frame, prev_pts[i], next_pts[i] , cv::Scalar(0,255,0), 1, CV_AA, 0);

                end = clock();

                sprintf(str, "Alg: %i ", (int) ((end - start)));
                cv::putText(frame, str, cv::Point(10, 10), cv::FONT_HERSHEY_PLAIN, 1, cvScalar(0, 0, 0));
                cv::putText(frame, str2, cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1, cvScalar(0, 0, 0));

                if (RecordFlag == 1) {
                    cv::cvtColor(frame, RecordFrame, CV_CVTIMG_FLIP);
                    video_out.write(RecordFrame);

                }
                //fprintf(datei, "%i;%i;%f;%f;%f\n", (int)i, (int) (clock() - start2), magnitude,
                //        displacement_vector_x[i], displacement_vector_y[i] );

            }

            //all_x.push_back(displacement_vector_x);
            //all_y.push_back(displacement_vector_y);

            next_pts.resize(k);

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

