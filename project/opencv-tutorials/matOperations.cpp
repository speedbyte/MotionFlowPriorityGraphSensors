#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <boost/filesystem.hpp>

int main ( int argc, char *argv[]) {

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
    cv::Mat img_rgb = cv::imread(fpath.string(), CV_LOAD_IMAGE_COLOR);
    cv::Mat img_rgb_orig = cv::imread(fpath.string(), CV_LOAD_IMAGE_COLOR);
    cv::Mat_<cv::Matx<uchar,512,512>> img_rgb_;
    std::cout<<img_rgb_orig.total();
    std::cout<<img_rgb_.total();
    cv::randu(img_rgb_,1,2);
    //img_rgb_orig.convertTo(img_rgb_, CV_8U);


    //cv::Point
    cv::Point3_<float> point3_1(5,1,2); // 3D accessed via x,y,z
    cv::Point3_<float> point3_2(10,1,2); // 3D accessed via x,y,z
    std::cout << " Dot " << point3_1.dot(point3_2)  << " Cross " << point3_1.cross(point3_2) << std::endl;
    cv::Point_<float> point2(10,11);
    std::cout << point2.x << point2.y << std::endl;

    //cv::Size: same as point, but the access is thru width and height.
    cv::Size_<float> size2_1(5,15);
    std::cout << "Area" << size2_1.area() << "for values " << size2_1.width << size2_1.height << std::endl;


    //cv::Vec,3. It makes by default a column vector
    cv::Vec<float,10> vec1D_1(3,4,10.0);
    cv::Vec<float,10> vec1D_2(3,3,10.0);
    cv::Vec<float,5> vec1D_3(100,10.0, 333.22);
    std::cout << "Member " << vec1D_1(0) << " Array " << vec1D_1.row(2)  << std::endl;


    //cv::Matx
    cv::Matx<float,3,4> matx2D_1;
    cv::Matx<float,3,3> matx2D_2;
    cv::Matx<float,2,8> matx1D_1(1,1,100,10.0,100,10.0,100,10.0,100,10.0,100,10.0,100,10.0,100,6767);
    cv::Matx<float,2,2> matx22;
    //matx1D_1.all(100.0);
    std::cout << matx1D_1(0,0) << matx2D_1.channels  << matx2D_1.row(1)<< std::endl;

    //cv::Mat

    cv::Mat mat34(3,4, CV_32FC(3),cv::Scalar(20,30,40));
    mat34.at<float>(2,3) = 1000;
    std::cout << mat34.at<float>(1,3) << " and " << mat34.at<cv::Vec<float,3> >(1,3)[1] << std::endl;
    std::cout << mat34 << std::endl;

    /*cv::Mat_<cv::Matx<cv::Vec2f,3,4> > mat34_;
    cv::randu(mat34_,20,30);
    mat34_(2,3) = {1000,1000};
    std::cout << mat34_(2,3) << " and " << mat34_(1,3) << std::endl;
    std::cout << mat34_ << std::endl;*/


    std::vector<float> vector_data;

    float data1[6] = {1,2,3,4,5,6};
    float data2[6] = {10,20,30,40,50,60};
    float data3[6] = {100,200,300,400,500,600};

    cv::Mat mat1_orig(3,4,CV_32FC1);
    cv::Mat_<float> mat1;
    mat1_orig.convertTo(mat1,CV_32FC1);
    mat1.setTo(cv::Scalar(33.3));

    cv::Mat mat2(3,4,CV_32FC1);
    cv::Mat_<cv::Matx<float,3,4>> mat2_;
    cv::Mat mat3(3,4,CV_32FC1,data1,sizeof(float));
    cv::Mat mat4(cv::Size(3,4),CV_32FC1);
    cv::Mat mat5(cv::Size(3,4),CV_32FC1,cv::Scalar(66.6));
    cv::Mat mat6(cv::Size(3,4),CV_32FC1,data2,sizeof(float));
    int sz[] = {100, 100, 100};
    cv::Mat bigCube1(3, sz, CV_32FC3);
    cv::Mat bigCube2(3, sz, CV_32FC1, cv::Scalar::all(99));
    //cv::Mat bigCube3(3, sz, CV_8UC1, data3, 4);
    std::cout << mat1 << std::endl << mat2 << std::endl << mat3 << std::endl << mat4 << std::endl << mat5 <<
                                                                                                          std::endl
              << mat6 << std::endl;
    std::cout << bigCube1.at<float>(10,10,10)  << std::endl << bigCube2.at<float>(10,10,10) << std::endl ;
              //<< bigCube3 << std::endl;

    std::vector<cv::Range> ranges(3, cv::Range(10,20));
    cv::Mat roiRange( img_rgb, cv::Range(100, 300), cv::Range(0, 512));
    cv::Mat roiRange2 = img_rgb.rowRange(cv::Range(100, 300));
    cv::Mat roiRange3 = img_rgb.colRange(cv::Range(100, 300));
    cv::Mat roiRange4 = img_rgb.colRange(100,300);
    cv::Mat roiRange5 = img_rgb.diag(); // display main diagonal
    cv::Mat roiRect( img_rgb, cv::Rect(0,100,512,200)); // start column, start row, width, height
    cv::Mat roiRangeMultiple( bigCube1, ranges);
    cv::namedWindow("range", CV_WINDOW_AUTOSIZE);
    imshow("range", roiRange5);
    cv::namedWindow("rect", CV_WINDOW_AUTOSIZE);
    imshow("rect", roiRect);
    std::cout << roiRangeMultiple.at<float>(0,1,1);

    cv::waitKey(0);

    // Access
    cv::Mat img_rgb1d(10,3,CV_8UC1, cv::Scalar::all(1));
    img_rgb1d.row(0) = img_rgb1d.row(3) + img_rgb1d.row(5)*3+2;
    img_rgb1d.at<char>(5,1) = 10;
    // now copy the 7-th column to the 1-st column
    img_rgb1d.col(0) = img_rgb1d.col(0) + img_rgb1d.col(1);
    img_rgb1d.col(0) = img_rgb1d.col(1) + 10;
    img_rgb1d.push_back(img_rgb1d.row(3));

    std::vector<cv::Mat> mat(592, cv::Mat(47, 47, CV_32FC1)); // allocates 592 matrices sized 47 by 47
    for(auto &m: mat) {
        // do your processsing here
        bigCube1.copyTo(m);
    }

    cv::Mat vecType(vec1D_3);
    cv::Mat matxType(matx2D_2);
    cv::Mat zeroMat = cv::Mat::zeros(3,4,CV_32FC1);
    cv::Mat oneMat = cv::Mat::ones(3,4,CV_32FC1);
    cv::Mat eyeMat = cv::Mat::eye(3,4,CV_32FC1);
    cv::Mat eyeMat2 = cv::Mat::eye(3,4,CV_32FC2);
    std::cout << oneMat.row(1);

    cv::waitKey(0);
}