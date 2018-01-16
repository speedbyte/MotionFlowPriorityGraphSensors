#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <iostream>

void image_processing_() {
    boost::filesystem::path fpath("../../../datasets/pics_dataset/lena.png");
    try {
        if ( !boost::filesystem::exists(fpath) ) {
            throw ("FileNotFound error");
        }
    }
    catch  ( const char * value ){
        std::cout << value;
        exit(0);
    }

    // Image Processing Mat
    cv::Mat img_rgb = cv::imread(fpath.string(), CV_LOAD_IMAGE_COLOR);
    if ( img_rgb.empty() == 1 ) {
        throw ("Not an image");
    }
    try {
        assert(img_rgb.depth() == CV_8U);
        assert(img_rgb.channels() == 3);
    }
    catch (...){
        exit(0);
    }

    // Image Process Mat_
    cv::Mat_<cv::Matx<uchar,512,512>> img_rgb_; //row,column
    std::cout<<img_rgb_.total() << std::endl;
    cv::randu(img_rgb_,1,2);

    // ------------------------------------------------------
    // Range
    cv::Mat roiRange( img_rgb, cv::Range(100, 300), cv::Range(0, 512)); // Row and Column in one
    // different ways to extract rows and columns from Range
    cv::Mat roiRange2 = img_rgb.rowRange(cv::Range(100, 300));  // Row
    cv::Mat roiRange3 = img_rgb.colRange(cv::Range(100, 300));  // Column
    cv::Mat roiRange4 = img_rgb.rowRange(100,300);  // Row, another way of depicting
    cv::Mat roiRange5 = img_rgb.colRange(100,300);  // Column, another way of depicting

    // extract section of an image ( the main diaognal )
    cv::Mat roiRange6 = img_rgb.diag();
    cv::namedWindow("range", CV_WINDOW_AUTOSIZE);
    imshow("range", roiRange4);

    cv::waitKey(0);
    cv::destroyAllWindows();

    // ------------------------------------------------------
    // extract section of an image using Rect
    cv::Mat roiRect( img_rgb, cv::Rect_<int>(0,100,512,200)); // start column, start row, width, height
    cv::namedWindow("rect", CV_WINDOW_AUTOSIZE);
    imshow("rect", roiRect);

    cv::waitKey(0);
    cv::destroyAllWindows();

    // ------------------------------------------------------
    // Access
    cv::Mat img_rgb2d3c(512,512,CV_8UC3, cv::Scalar_<int>(255,0,0)); // Blue image
    img_rgb2d3c.row(0) = img_rgb2d3c.row(3) + img_rgb2d3c.row(5)*3+2;
    img_rgb2d3c.at<char>(5,1) = 10;
    // now copy the 7-th column to the 1-st column
    img_rgb2d3c.col(0) = img_rgb2d3c.col(0) + img_rgb2d3c.col(1);
    img_rgb2d3c.col(0) = img_rgb2d3c.col(1) + 10;
    img_rgb2d3c.push_back(img_rgb2d3c.row(3));
    cv::namedWindow("scalar", CV_WINDOW_AUTOSIZE);
    imshow("scalar", img_rgb2d3c);

    cv::waitKey(0);
    cv::destroyAllWindows();

}

void image_processing_bmp() {
    boost::filesystem::path fpath("../../../datasets/pics_dataset/lena.png");
    try {
        if ( !boost::filesystem::exists(fpath) ) {
            throw ("FileNotFound error");
        }
    }
    catch  ( const char * value ){
        std::cout << value;
        exit(0);
    }

    // Image Processing Mat
    cv::Mat img_rgb = cv::imread(fpath.string(), CV_LOAD_IMAGE_COLOR);
    try {
        assert(img_rgb.depth() == CV_8U);
        assert(img_rgb.channels() == 3);
    }
    catch (...){
        exit(0);
    }

    cv::Mat img_rgb_orig = cv::imread(fpath.string(), CV_LOAD_IMAGE_COLOR);
    std::cout<<img_rgb_orig.total() << std::endl;
    cv::Mat img_rgb_conv;
    img_rgb_orig.convertTo(img_rgb_conv, CV_8U);
    cv::Mat img_rgb_bmp(img_rgb.rows, img_rgb.cols, CV_8UC3, img_rgb.data); // provide different view of m1 data
    // depending on endianess of reader, you may need to swap byte order of m2 pixels
    cv::imwrite("../../../pics_dataset/lena.bmp", img_rgb_bmp);

}

void sobel() {
    cv::Mat img = cv::imread("image.jpg");
    cv::Mat grey;
    cv::cvtColor(img, grey, CV_BGR2GRAY);

    cv::Mat sobelx;
    cv::Sobel(grey, sobelx, CV_32F, 1, 0);

    double minVal, maxVal;
    cv::minMaxLoc(sobelx, &minVal, &maxVal); //find minimum and maximum intensities
    cv::Mat draw;
    sobelx.convertTo(draw, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

    cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
    cv::imshow("image", draw);
}

void absdiff(cv::Mat &kittisrc1, cv::Mat &kittisrc2) {
    cv::absdiff(kittisrc1, kittisrc2, kittisrc2); // this gives a result with just the moving objects between two
    // frames.
}

void addWeighted() {
    cv::Mat lena = cv::imread("../../../datasets/pics_dataset/lena.png", CV_LOAD_IMAGE_COLOR); //CV_LOAD_IMAGE_COLOR=1
    cv::Mat input1(lena, cv::Rect(0,0,20,370));
    cv::Mat input2(lena, cv::Rect(100,0,20,370));
    cv::Mat output(lena, cv::Rect(256,0,20,370)); // x, y, width, height
    // one to blend is the image of the guy and the second parameter.

    cv::addWeighted(input1, 1, input2, 1, 0, output); //output = input1*alpha + input2*beta + gamma
    // this gives a result with just the moving objects between two
    cv::namedWindow("result", CV_WINDOW_AUTOSIZE);
    cv::imshow("result", lena);
    cv::waitKey(0);

    cv::Mat jpgImage = cv::imread("../../../datasets/pics_dataset/lena.jpg");
    cv::namedWindow("result", CV_WINDOW_AUTOSIZE);
}

void imdecode() {
    int    nsize = 100;       // Size of buffer
    uchar pcBuffer[nsize];    // Raw buffer data
    for ( int i = 0; i < nsize; i++) {
        pcBuffer[i] = 255;
    }

// Create a Size(1, nSize) Mat object of 8-bit, single-byte elements
    cv::Mat rawData  =  cv::Mat( 1, nsize, CV_8UC1, pcBuffer );

    cv::Mat decodedImage  =  cv::imdecode( rawData, 0 /*, flags */ );
    if ( decodedImage.data == NULL )
    {
        // Error reading raw image data
    }
}

void miscallenous() {
    cv::Mat misc(50,100,CV_8UC3);
    misc = cv::Scalar::all(255);
    misc = cv::Scalar((rand()%255),(rand()%255),0);
    cv::Mat test_frame = cv::Mat::zeros(cv::Size(1242,375), CV_8UC3);
    cv::Mat test_absolute_frame = cv::Mat::zeros(cv::Size(1242,375), CV_16UC3);
}

int main ( int argc, char *argv[] ) {


    cv::Mat kittisrc1 = cv::imread("../../../datasets/kitti_dataset/raw_dataset_with_calib/2011_09_28_drive_0016_sync"
                                           "/image_02"
                                         "/data"
                              "/0000000169"".png", CV_LOAD_IMAGE_COLOR);
    cv::Mat kittisrc2 = cv::imread("../../../datasets/kitti_dataset/raw_dataset_with_calib/2011_09_28_drive_0016_sync"
                                           "/image_02/data"
                              "/0000000170.png");

    // store in binary format to be able to compare the data.
    addWeighted();
    image_processing_();
    image_processing_bmp();

    absdiff(kittisrc1, kittisrc2);
    //xml_yaml(kittisrc1, kittisrc2);

    imdecode();

    cv::waitKey(0);

}