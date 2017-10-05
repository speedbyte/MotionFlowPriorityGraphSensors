

#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv/cv.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <iostream>
#include <gnuplot-iostream/gnuplot-iostream.h>
#include <chrono>

extern void salt(cv::Mat image, int n);

using namespace std::chrono;

void canny(cv::Mat& img, cv::Mat& out) {
    // Convert to gray
    if (img.channels()==3)
        cv::cvtColor(img,out,CV_BGR2GRAY);
    // Compute Canny edges
    cv::Canny(out,out,100,200);
    // Invert the image
    cv::threshold(out,out,128,255,cv::THRESH_BINARY_INV);
}


void make_video_from_png(boost::filesystem::path dataset_path, std::string unterordner) {
    cv::VideoWriter write;
    cv::Mat temp_image;

    boost::filesystem::path dir_path = dataset_path;
    dir_path += unterordner;

    std::cout << dir_path.string();
    assert(boost::filesystem::exists(dir_path) != 0);

    std::string file_name, path;
    char file_name_char[20];
    int number = 0;

    do {
        sprintf(file_name_char, "0000000%03d", number);
        path = dir_path.string() + std::string(file_name_char) + ".png";
        temp_image = cv::imread(path, cv::IMREAD_COLOR);
        if ( number == 0 ) {
            write.open((dir_path.string()+"original_video.avi" ), CV_FOURCC('D', 'I', 'V', 'X'), 30.0,
                       cv::Size(temp_image.cols, temp_image.rows), true);
        }
        write.write(temp_image);
        number++;
    } while ( boost::filesystem::exists(path) != 0);

    write.release();


}


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

/* It is Lucas & Kanade method, modified to use pyramids.
   Also it does several iterations to get optical flow for
   every point at every pyramid level.
   Calculates optical flow between two images for certain set of points (i.e.
   it is a "sparse" optical flow, which is opposite to the previous 3 methods)

 void cv::calcOpticalFlowPyrLK(
  cv::InputArray       prevImg,            // Prior image (t-1), CV_8UC1
  cv::InputArray       nextImg,            // Next image (t), CV_8UC1
  cv::InputArray       prevPts,            // Vector of 2d start points (CV_32F)
  cv::InputOutputArray nextPts,            // Results: 2d end points (CV_32F)
  cv::OutputArray      status,             // For each point, found=1, else=0
  cv::OutputArray      err,                // Error measure for found points
  cv::Size             winSize         = Size(15,15),   // size of search window
  int                  maxLevel        = 3,             // Pyramid layers to add
  cv::TermCriteria     criteria        = TermCriteria(  // How to end search
                         cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                         30,
                         0.01
                       ),
  int                  flags           = 0,    // use guesses, and/or eigenvalues
  double               minEigThreshold = 1e-4  // for spatial gradient matrix
);

void cv::goodFeaturesToTrack(
        cv::InputArray  image,                         // Input, CV_8UC1 or CV_32FC1
        cv::OutputArray corners,                       // Output vector of corners - either Vector cv::Point2f or
                                                            cv::Mat(x,2)
        int             maxCorners,                    // Keep this many corners
        double          qualityLevel,                  // (fraction) rel to best
        double          minDistance,                   // Discard corner this close
        cv::InputArray  mask              = noArray(), // Ignore corners where mask=0
        int             blockSize         = 3,         // Neighborhood used
        bool            useHarrisDetector = false,     // false='Shi Tomasi metric'
        double          k                 = 0.04       // Used for Harris metric
);
*/



void of_algo(boost::filesystem::path dataset_path, std::string video, std::string algo) {

    std::vector<unsigned> x_pts;
    std::vector<double> y_pts;
    std::vector<unsigned> z_pts;
    std::vector<boost::tuple<std::vector<unsigned>, std::vector<double>> > pts_exectime;

    bool needToInit = true;
    std::vector<cv::Point2f> prev_pts;
    std::vector<cv::Point2f> next_pts;
    cv::Mat flowImage;
    cv::Mat curGray, prevGray;
    cv::Mat frame;
    cv::VideoCapture cap;
    boost::filesystem::path video_path = dataset_path;
    video_path += "video/"+video;
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
    VideoOutFile += "/OpticalFlow_"+algo+".avi";

    cv::Size frame_size;
    frame_size.height =	(int) cap.get(CV_CAP_PROP_FRAME_HEIGHT );
    frame_size.width =	(int) cap.get(CV_CAP_PROP_FRAME_WIDTH );
    video_out.open(VideoOutFile.string(),CV_FOURCC('D','I','V','X'), 5, frame_size);
    printf("Writer eingerichtet\n");


    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
    cv::Size subPixWinSize(10, 10), winSize(31, 31);

    const int MAX_COUNT = 500;
    cv::Mat pyramid1, pyramid2;

    pyramid1.create(frame_size, CV_8UC1);
    pyramid2.create(frame_size, CV_8UC1);

    unsigned frame_count = 0;

    cv::namedWindow(algo, CV_WINDOW_AUTOSIZE);

    // Iterate until the user presses the Esc key
    while (true) {
        // Break out of the loop if the user presses the Esc key
        char c = (char) cv::waitKey(10);
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
        frame_count++;

        // Resize the frame
        //cv::resize(frame, frame, cv::Size(), scalingFactor, scalingFactor, cv::INTER_AREA);

        // Convert to grayscale
        cv::cvtColor(frame, curGray, cv::COLOR_BGR2GRAY);

        auto tic = steady_clock::now();
        // Check if the image is valid
        if (algo.compare("FB") == 0) {
            if (prevGray.data) {
                // Initialize parameters for the optical flow algorithm
                float pyrScale = 0.5;
                int numLevels = 3;
                int windowSize = 15;
                int numIterations = 3;
                int neighborhoodSize = 5;
                float stdDeviation = 1.2;

                // Calculate optical flow map using Farneback algorithm
                cv::calcOpticalFlowFarneback(prevGray, curGray, flowImage, pyrScale, numLevels, windowSize,
                                             numIterations,
                                             neighborhoodSize, stdDeviation, cv::OPTFLOW_USE_INITIAL_FLOW);

                // Draw the optical flow map
                int stepSize = 16;

                // Draw the uniform grid of points on the input image along with the motion vectors
                for (int y = 0; y < frame.rows; y += stepSize) {
                    for (int x = 0; x < frame.cols; x += stepSize) {
                        // Circles to indicate the uniform grid of points
                        cv::circle(frame, cv::Point(x, y), 0.5, cv::Scalar(0, 255, 0), -1, 8);

                        // Lines to indicate the motion vectors
                        cv::Point2f pt = flowImage.at<cv::Point2f>(y, x);
                        cv::arrowedLine(frame, cv::Point(x, y), cv::Point(cvRound(x + pt.x), cvRound(y + pt.y)),
                                        cv::Scalar(0,
                                                   255, 0));
                    }
                }
            }
        }

        else if (algo.compare("LK") == 0) {
            // Calculate optical flow map using LK algorithm
            if (needToInit) {
                // automatic initialization
                cv::goodFeaturesToTrack(curGray, next_pts, MAX_COUNT, 0.01, 10, cv::Mat(), 3, false, 0.04);
                // Refining the location of the feature points
                if (next_pts.size() < MAX_COUNT) {
                    std::vector<cv::Point2f> currentPoint;
                    std::swap(currentPoint, next_pts);
                    for (unsigned i = 0; i < currentPoint.size(); i++) {
                        std::vector<cv::Point2f> tempPoints;
                        tempPoints.push_back(currentPoint[i]);
                        // Function to refine the location of the corners to subpixel accuracy.
                        // Here, 'pixel' refers to the image patch of size 'windowSize' and not the actual image pixel
                        cv::cornerSubPix(curGray, tempPoints, subPixWinSize, cv::Size(-1, -1), termcrit);
                        next_pts.push_back(tempPoints[0]);
                    }
                    std::swap(currentPoint, next_pts);
                }
            }

            if (!prev_pts.empty()) {
                std::vector<uchar> status;
                std::vector<float> err;
                if (prevGray.empty()) {
                    curGray.copyTo(prevGray);
                }
                cv::calcOpticalFlowPyrLK(prevGray, curGray, prev_pts, next_pts, status,
                                         err, winSize, 3, termcrit, 0, 0.001);

                unsigned count = 0;
                int minDist = 0;

                for (unsigned i = 0; i < next_pts.size(); i++) {
                    /* If the new point is within 'minDist' distance from an existing point, it will not be tracked */
                    if (cv::norm(prev_pts[i] - next_pts[i]) <= minDist) {
                        //printf("minimum distance for %i\n", i);
                        continue;
                    }

                    // Check if the status vector is good
                    if (!status[i])
                        continue;

                    next_pts[count++] = next_pts[i];
                    //cv::circle(frame, next_pts[count], 3, cv::Scalar(0, 255, 0), -1, 8);
                    cv::arrowedLine(frame, prev_pts[i], next_pts[i], cv::Scalar(0, 255, 0), 1, CV_AA, 0);
                }
                next_pts.resize(count);
                printf(" new size is %i for frame number %u\n", count, frame_count);
                z_pts.push_back(count);
            }
        }
        auto toc = steady_clock::now();

        x_pts.push_back(frame_count);
        y_pts.push_back(duration_cast<milliseconds>(toc - tic).count());

        video_out.write(frame);
        // Display the output image
        cv::imshow(algo, frame);
        needToInit = false;
        prev_pts.clear();
        std::swap(next_pts, prev_pts);
        std::swap(prevGray, curGray);

    }
    pts_exectime.push_back(boost::make_tuple(x_pts, y_pts));
    video_out.release();
    cv::destroyAllWindows();

    // gnuplot_2d
    Gnuplot gp2d;
    gp2d << "set xrange [0:200]\n";
    gp2d << "set yrange [0:100]\n";
    gp2d << "plot" << gp2d.binFile2d(pts_exectime, "record") << " with lines title 'vec of boost::tuple of vec'\n";

}

