


#include <boost/filesystem/path.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <boost/filesystem.hpp>

void disparity(boost::filesystem::path dataset_path) {


    cv::Mat img1, img2, g1, g2;
    cv::Mat disp, cDisp;

    cv::VideoWriter write;
    cv::Mat temp_image;

    boost::filesystem::path video_path = dataset_path;
    video_path += "video/" ;

    std::cout << video_path.string();

    std::string file_name, path;
    char file_name_char[10];
    int number = 0;
    std::string dir_path = dataset_path.string() + "image_02/data/";
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


    std::string left = argv[1];
    std::string right = argv[2];

    img1 = cv::imread(left);
    img2 = cv::imread(right);

    cv::namedWindow("Example1", cv::WINDOW_AUTOSIZE);
    cv::imshow("Example", img1);
    cv::waitKey(0);
    cv::destroyWindow("Example1");

    cv::namedWindow("Example2", cv::WINDOW_AUTOSIZE);
    imshow("Example2", img2);
    cv::waitKey(0);
    cv::destroyWindow("Example2");

    cv::cvtColor(img1, g1, CV_BGR2GRAY);
    cv::cvtColor(img2, g2, CV_BGR2GRAY);

    //creates disparity map
    //@param numdisparity
    //@param blocksize
    //wide camera setting or close objects -> increase numdisparity

    cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(80, 15);

    sbm->setPreFilterCap(31);
    sbm->setPreFilterSize(5);
    sbm->setTextureThreshold(500);
    sbm->setSpeckleWindowSize(0);
    sbm->setSpeckleRange(8);
    sbm->setMinDisparity(0);
    sbm->setUniquenessRatio(0);
    sbm->setDisp12MaxDiff(1);

    sbm->compute(g1, g2, disp);
    cv::normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);

    cv::namedWindow("Example3", cv::WINDOW_AUTOSIZE);
    cv::imshow("Example3", disp);
    cv::waitKey(0);
    cv::destroyWindow("Example3");

    cv::Mat cDisp;
    cv::Mat coloredDisp;
    double min;
    double max;

    cv::minMaxIdx(*disp, &min, &max);

    cv::convertScaleAbs(*disp, cDisp, 255 / (max - min));
    cv::applyColorMap(cDisp, coloredDisp, 2);

    cDisp = coloredDisp;

    cv::namedWindow("Example4", cv::WINDOW_AUTOSIZE);
    cv::imshow("Example4", cDisp);
    cv::waitKey(0);
    cv::destroyWindow("Example4");

}

