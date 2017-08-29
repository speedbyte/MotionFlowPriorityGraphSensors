#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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

int main ( int argc, char *argv[] ) {
    cv::Mat kittisrc1 = cv::imread("../../../kitti_dataset/raw_dataset_with_calib/2011_09_28_drive_0016_sync/image_02"
                                         "/data"
                              "/0000000169"".png", CV_LOAD_IMAGE_COLOR);
    cv::Mat kittisrc2 = cv::imread("../../../kitti_dataset/raw_dataset_with_calib/2011_09_28_drive_0016_sync/image_02/data"
                              "/0000000170.png");

    // store in binary format to be able to compare the data.
    cv::absdiff(kittisrc1, kittisrc2, kittisrc2); // this gives a result with just the moving objects between two
    // frames.

    cv::Mat lena = cv::imread("../../../pics-dataset/lena.png", CV_LOAD_IMAGE_COLOR); //CV_LOAD_IMAGE_COLOR=1
    assert(kittisrc1.type() == lena.type());
    cv::Mat input1(lena, cv::Rect(0,0,20,370));
    cv::Mat input2(lena, cv::Rect(256,0,20,370));
    cv::Mat output(lena, cv::Rect(256,0,20,370));
    // one to blend is the image of the guy and the second parameter.

    cv::addWeighted(input1, 1, input2, 1, 0, output); //
    // this gives a
    // result with just the moving
    // objects
    // between
    // two
    cv::namedWindow("result", CV_WINDOW_AUTOSIZE);
    cv::imshow("result", lena);
    cv::waitKey(0);

    cv::Mat whiteImage1C(8,8,CV_8UC1,cv::Scalar(255));
    cv::Mat whiteImage3C(8,8,CV_8UC3,cv::Scalar(255,0,0));
    cv::imwrite("/local/tmp/result1C.png", whiteImage1C);
    cv::imwrite("/local/tmp/result2C.png", whiteImage3C);
    cv::Mat jpgImage = cv::imread("../../../pics-dataset/lena.jpg");
    cv::namedWindow("result", CV_WINDOW_AUTOSIZE);
    cv::imshow("result", kittisrc2);

    cv::FileStorage fs("../../../pics-dataset/test.yml", cv::FileStorage::WRITE);

    fs << "frameCount" << 5;
    time_t rawtime; time(&rawtime);
    fs << "calibrationDate" << asctime(localtime(&rawtime));
    cv::Mat cameraMatrix = cv::Mat_<double>(3,3) << (1000, 0, 320, 0, 1000, 240, 0, 0, 1);
    cv::Mat distCoeffs = cv::Mat_<double>(5,1) << (0.1, 0.01, -0.001, 0, 0);
    fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
    fs << "features" << "[";
    for( int i = 0; i < 3; i++ )
    {
        int x = rand() % 640;
        int y = rand() % 480;
        uchar lbp = rand() % 256;

        fs << "{:" << "x" << x << "y" << y << "lbp" << "[:";
        for( int j = 0; j < 8; j++ )
            fs << ((lbp >> j) & 1);
        fs << "]" << "}";
    }
    fs << "]";

    fs << "whiteImage1C" << whiteImage1C;
    fs << "whiteImage3C" << whiteImage3C;

    fs << "features" << "[";
    int row = 0;
    int col = 0;
    std::cout << kittisrc2.depth() << kittisrc2.channels();
    assert(kittisrc2.type()==CV_8UC3);
    while ( row <= kittisrc2.rows ) {
        col = 0;
        while ( col <= kittisrc2.cols ) {
            if (((kittisrc1.at<cv::Vec<char, 3>>(row, col)[0] + kittisrc1.at<cv::Vec<char, 3>>(row, col)[1] +
                        kittisrc1
                    .at<cv::Vec<char, 3>>(row, col)[2]) / 3 ) > 100) {
                fs << "{:" << "row" << row << "col" << col << "lbp" << "[:";
                fs << kittisrc1.at<cv::Vec<char, 3>>(row, col)[0] << kittisrc1.at<cv::Vec<char, 3>>(row, col)[1] <<
                   kittisrc1.at<cv::Vec<char, 3>>(row, col)[2];
                fs << "]" << "}";
            }
            col++;
        }
        row++;
    }
    fs << "]";


    fs.release();

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

    cv::waitKey(0);

}