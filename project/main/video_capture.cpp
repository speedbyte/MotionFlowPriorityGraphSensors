

#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv/cv.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <iostream>


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


void of_farneback( boost::filesystem::path dataset_path ) {

/**
 void cv::calcOpticalFlowFarneback(
  cv::InputArray       prevImg,    // An input image
  cv::InputArray       nextImg,    // Image immediately subsequent to 'prevImg'
  cv::InputOutputArray flow,       // Flow vectors will be recorded here
  double               pyrScale,   // Scale between pyramid levels (< '1.0')
  int                  levels,     // Number of pyramid levels
  int                  winsize,    // Size of window for pre-smoothing pass
  int                  iterations, // Iterations for each pyramid level
  int                  polyN,      // Area over which polynomial will be fit
  double               polySigma,  // Width of fit polygon, usually '1.2*polyN'
  int                  flags       // Option flags, combine with OR operator

 The polyN argument determines the size of the area considered when fitting the polynomial around a point. This is
 different from winsize, which is used only for presmoothing. polyN could be thought of as analogous to the window
 size associated with Sobel derivatives. If this number is large, high-frequency fluctuations will not contribute to
 the polynomial fitting. Closely related to polyN is polySigma, which is the source of the intrinsic scale for the
 motion field. The derivatives computed as part of the fit use a Gaussian kernel (not the one associated with the
 smoothing) with variance polySigma and whose total extent is polyN. The value of polySigma should be a bit more than
 20% of polyN. (The pairings polyN=5, polySigma=1.1, and polyN=7, polySigma=1.5 have been found to work well and are
 recommended in source code.
);

 */
    bool needToInit = true;
    std::vector<cv::Point2f> prev_pts;
    std::vector<cv::Point2f> next_pts;
    cv::Mat flowImage;
    cv::Mat curGray, prevGray;
    cv::Mat frame;
    cv::VideoCapture cap;
    boost::filesystem::path video_path = dataset_path;
    video_path += "video/2011_09_28_drive_0016_sync.avi" ;
    if (boost::filesystem::exists(video_path) == 0) {
        throw("no video file");
    }
    std::cout << video_path.string() << std::endl;
    cap.open(video_path.string());
    if (!cap.isOpened()) {
        std::cout << "Could not initialize capturing...\n";
        return;
    }
    //Schreibt das Video mit
    cv::VideoWriter video_out;
    boost::filesystem::path VideoOutFile = video_path.parent_path();
    VideoOutFile += "/OpticalFlow_Farneback.avi";

    cv::Size frame_size;
    frame_size.height =	(int) cap.get(CV_CAP_PROP_FRAME_HEIGHT );
    frame_size.width =	(int) cap.get(CV_CAP_PROP_FRAME_WIDTH );
    video_out.open(VideoOutFile.string(),CV_FOURCC('P','I','M','1'), 30, frame_size);
    printf("Writer eingerichtet\n");

    double scalingFactor = 0;

    // Iterate until the user presses the Esc key
    while (true) {
        // Break out of the loop if the user presses the Esc key
        char c = (char)cv::waitKey(10);
        switch (c) {
            case 27:
                break;
            case 'r':
                needToInit = true;
                break;
            case 'c':
                prev_pts.clear();
                next_pts.clear();
                break;
            default:
                break;
        }
        cap >> frame;
        if (frame.empty())
            break;

        // Resize the frame
        //cv::resize(frame, frame, cv::Size(), scalingFactor, scalingFactor, cv::INTER_AREA);

        // Convert to grayscale
        cv::cvtColor(frame, curGray, cv::COLOR_BGR2GRAY);

        // Check if the image is valid
        if (prevGray.data) {
            // Initialize parameters for the optical flow algorithm
            float pyrScale = 0.5;
            int numLevels = 3;
            int windowSize = 15;
            int numIterations = 3;
            int neighborhoodSize = 5;
            float stdDeviation = 1.2;

            // Calculate optical flow map using Farneback algorithm
            cv::calcOpticalFlowFarneback(prevGray, curGray, flowImage, pyrScale, numLevels, windowSize, numIterations,
                                     neighborhoodSize, stdDeviation, cv::OPTFLOW_USE_INITIAL_FLOW);

            // Draw the optical flow map
            int stepSize = 16;

            // Draw the uniform grid of points on the input image along with the motion vectors
            for(int y = 0; y < frame.rows; y += stepSize)
            {
                for(int x = 0; x < frame.cols; x += stepSize)
                {
                    // Circles to indicate the uniform grid of points
                    cv::circle(frame, cv::Point(x,y), 0.5, cv::Scalar(0, 255, 0), -1, 8);

                    // Lines to indicate the motion vectors
                    cv::Point2f pt = flowImage.at<cv::Point2f>(y, x);
                    cv::arrowedLine(frame, cv::Point(x,y), cv::Point(cvRound(x+pt.x), cvRound(y+pt.y)), cv::Scalar(0,
                                                                                                                  255, 0));
                }
            }

            video_out.write(frame);
            // Display the output image
            cv::imshow("farneback", frame);
        }

        needToInit = false;
        cv::swap(next_pts, prev_pts);
        std::swap(prevGray, curGray);
    }
    video_out.release();
}

void of_lk(boost::filesystem::path dataset_path) {

    cv::Point2f point;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
    cv::Size subPixWinSize(10, 10), winSize(31, 31);

    const int MAX_COUNT = 500;
    int i = 0;

    cv::Mat pyramid1, pyramid2;
    cv::namedWindow("LK Demo", 1);

    //Zeitsteuerung über clock()
    clock_t start, start2, end;
    int LOOPtime = 100;
    start = clock();

    CvFont Font_= cvFont(0.5,1); //Vareablen zur Textausgabe
    char str[256];
    char str2[256];

    bool needToInit = true;
    std::vector<cv::Point2f> prev_pts;
    std::vector<cv::Point2f> next_pts;
    cv::Mat curGray, prevGray;
    cv::Mat frame;
    cv::VideoCapture cap;
    boost::filesystem::path video_path = dataset_path;
    video_path += "video/2011_09_28_drive_0016_sync.avi" ;
    if (boost::filesystem::exists(video_path) == 0) {
        throw("no video file");
    }
    std::cout << video_path.string() << std::endl;
    cap.open(video_path.string());
    if (!cap.isOpened()) {
        std::cout << "Could not initialize capturing...\n";
        return;
    }
    //Schreibt das Video mit
    cv::VideoWriter video_out;
    boost::filesystem::path VideoOutFile = video_path.parent_path();
    VideoOutFile += "/OpticalFlow_LK.avi";

    cv::Size frame_size;
    frame_size.height =	(int) cap.get(CV_CAP_PROP_FRAME_HEIGHT );
    frame_size.width =	(int) cap.get(CV_CAP_PROP_FRAME_WIDTH );
    video_out.open(VideoOutFile.string(),CV_FOURCC('P','I','M','1'), 30, frame_size);
    printf("Writer eingerichtet\n");

    for (;;) {
        // Break out of the loop if the user presses the Esc key
        char c = (char)cv::waitKey(10);
        switch (c) {
            case 27:
                break;
            case 'r':
                needToInit = true;
                break;
            case 'c':
                prev_pts.clear();
                next_pts.clear();
                break;
            default:
                break;
        }
        cap >> frame;
        if (frame.empty())
            break;

        pyramid1.create(frame_size, CV_8UC1);
        pyramid2.create(frame_size, CV_8UC1);

        cv::cvtColor(frame, curGray, cv::COLOR_BGR2GRAY);
        if ( prevGray.empty() ) {
            curGray.copyTo(prevGray);
        }

        if (i == -1 || needToInit) {
            // automatic initialization
            cv::goodFeaturesToTrack(curGray, next_pts, MAX_COUNT, 0.01, 10, cv::Mat(), 3, false, 0.04);
            cv::cornerSubPix(curGray, next_pts, subPixWinSize, cv::Size(-1, -1), termcrit);

            i = 0;

        } else if (!prev_pts.empty()) {
            std::vector<uchar> status;
            std::vector<float> err;
            cv::calcOpticalFlowPyrLK(prevGray, curGray, prev_pts, next_pts, status,
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

                sprintf(str, "X = %f ", (displacement_vector_x[i]));
                cv::putText(frame, str, cv::Point(frame_size.width-40, 10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));

                sprintf(str, "Y = %f ", (displacement_vector_y[i]));
                cv::putText(frame, str, cv::Point(frame_size.width-40, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));

                cv::arrowedLine(frame, prev_pts[i], next_pts[i] , cv::Scalar(0,255,0), 1, CV_AA, 0);

                end = clock();

                sprintf(str, "Alg: %i ", (int) ((end - start)));
                cv::putText(frame, str, cv::Point(10, 10), cv::FONT_HERSHEY_PLAIN, 1, cvScalar(0, 0, 0));
                cv::putText(frame, str2, cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1, cvScalar(0, 0, 0));

                //fprintf(datei, "%i;%i;%f;%f;%f\n", (int)i, (int) (clock() - start2), magnitude,
                //        displacement_vector_x[i], displacement_vector_y[i] );

            }

            //all_x.push_back(displacement_vector_x);
            //all_y.push_back(displacement_vector_y);
            next_pts.resize(k);
            video_out.write(frame);
            // Display the output image
            cv::imshow("lk", frame);
        }

        i++;
        needToInit = false;
        std::swap(next_pts, prev_pts);
        std::swap(prevGray, curGray);
    }
    video_out.release();
}

