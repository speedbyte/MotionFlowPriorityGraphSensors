#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/unordered_map.hpp>
#include <boost/filesystem.hpp>
#include <iostream>

//using namespace cv;

int main ( int argc, char *argv[]) {



    cv::Mat img_rgb, img_gry, img_gauss1, img_pyr_rgb, img_pyr_gry, img_canny;
    cv::Mat tmp;
    boost::unordered::unordered_map<int,cv::Mat> m_map_input;
    boost::unordered::unordered_map<int,cv::Mat> m_map_grid;
    boost::filesystem::path fpath("../../../pics-dataset/lena.png");

    try {
        if ( !boost::filesystem::exists(fpath) ) {
            throw ("FileNotFound error");
        }
    }
    catch  ( const char * value ){
        std::cout << value;
        exit(0);
    }

    img_rgb = cv::imread(fpath.string(), CV_LOAD_IMAGE_COLOR);
    cv::cvtColor(img_rgb, img_gry, cv::COLOR_BGR2GRAY);
    std::cout << "COLOR, " << img_rgb.channels() << " ,GRAY " << img_gry.channels()<<std::endl;
    //cv::Matx<cv::Vec3b,512,512> img_gauss2;
    //img_gauss2 = cv::imread("../data/lena.png", CV_LOAD_IMAGE_COLOR);

    // Guassian blurring ( always odd kernel size )
    GaussianBlur(img_rgb, img_gauss1, cv::Size(5,5), 3, 3);
    GaussianBlur(img_rgb, img_gauss1, cv::Size(5,5), 3, 3);
    // Guassian blurring and downsampling
    cv::pyrDown(img_rgb, img_pyr_rgb);
    // Canny
    cv::pyrDown(img_gry, img_pyr_gry);
    cv::Canny(img_pyr_gry, img_canny, 10, 100, 3, true);

    cv::Mat grid(img_rgb.rows*3, img_rgb.cols*3, img_rgb.type(), cv::Scalar(0)); // rows, column in declaring.

    m_map_input[0] = img_rgb;
    m_map_input[1] = img_gry;
    m_map_input[2] = img_gauss1;
    m_map_input[3] = img_pyr_rgb;
    m_map_input[4] = img_gauss1;
    m_map_input[5] = img_pyr_rgb;

    m_map_grid[0] = cv::Mat(grid, cv::Rect(512*0,0,  512,512));
    m_map_grid[1] = cv::Mat(grid, cv::Rect(512*0,512,512,512));
    m_map_grid[2] = cv::Mat(grid, cv::Rect(512*1,0,  512,512));
    m_map_grid[3] = cv::Mat(grid, cv::Rect(512*1,512,512,512));
    m_map_grid[4] = cv::Mat(grid, cv::Rect(512*2,0,512,512));
    m_map_grid[5] = cv::Mat(grid, cv::Rect(512*2,512,512,512));

    //assert(m_map.at(count) != tmp); This wil not work, because one cannot compare vectors.
    for ( uint8_t i = 0; i < m_map_input.size(); i++) {
        m_map_input.at(i).copyTo(m_map_grid.at(i));
    }

    // Getting and setting Pixels
    int y = 16; int x = 32; // column is x and row is y.
    cv::Vec3b intensity_format1 = img_rgb.at< cv::Vec3b >(y,x);
    //cv::Vec3b intensity_format2 = (img_rgb.at< cv::Vec3b >(16,32))[0];
    for ( int i = 0; i < 3; i++) {
        std::cout << (unsigned int)intensity_format1[i] << ',' << std::endl;
        //std::cout << (unsigned int)intensity_format2[i] << ',';
    }
    std::cout << (unsigned int)img_gry.at<uchar>(y,x) << ',' << std::endl;
    std::cout << (unsigned int)img_canny.at<uchar>(y,x) << ',' << std::endl;
    cv::namedWindow("grid", CV_WINDOW_AUTOSIZE);
    imshow("grid", grid);

    cv::waitKey(0);
}