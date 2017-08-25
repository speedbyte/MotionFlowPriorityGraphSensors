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


    // ------------------------------------------------------
    //cv::Point
    cv::Point_<float> point2(10,11);
    cv::Point3_<float> point3_1(5,1,2); // 3D accessed via x,y,z
    cv::Point3_<float> point3_2(10,1,2); // 3D accessed via x,y,z
    // The points are points in space and the x,y,z are distance wrt to origin.
    std::cout << " Dot " << point3_1.dot(point3_2)  << " Cross " << point3_1.cross(point3_2) << std::endl;
    std::cout << point2.x << point2.y << std::endl;

    // ------------------------------------------------------
    //cv::Size: same as point, but the access is thru width and height.
    cv::Size_<float> size2_1(5,15);
    std::cout << "Area" << size2_1.area() << "for values " << size2_1.width << size2_1.height << std::endl;

    //std::vector
    std::vector<float> vector_data;
    // ------------------------------------------------------
    //cv::Vec,3. It makes by default a column vector
    cv::Vec<float,10> vec1D_1(3,4,10.0);
    cv::Vec<float,10> vec1D_2(3,3,10.0);
    cv::Vec<float,5> vec1D_3(100,10.0, 333.33, 444.44, 555.55);
    vec1D_3.all(200.22);

    //Print
    std::cout << vec1D_1 << vec1D_1.t(); // transposing is from column vector to row vector for printing purpose
    std::cout << "Member " << vec1D_1(0) << " Array " << vec1D_1.row(2)  << " col " << vec1D_3.col(0)  << std::endl;


    //Matx ----------------------------------------------------------
    std::cout << "Begin Matx ----------------------------------" << std::endl;
    // Initialize
    cv::Matx<float,3,4> matx2D_1;
    cv::Matx<float,3,3> matx2D_2;
    cv::randu(matx2D_2,2000,3000);
    cv::Matx<float,2,8> matx1D_1(1,1,100,10.0,100,10.0,100,10.0,100,10.0,100,10.0,100,10.0,100,6767);
    cv::Matx<float,2,2> matx22;
    matx2D_2.all(100.11);
    std::cout << matx1D_1(0,0) << matx2D_1.channels  << matx2D_1.row(1)<< std::endl;
    // ------------------------------------------------------
    // Static functions in the class
    cv::Matx<float,5,1> vec1D(vec1D_3);  // Only 1 row is allowed.
    cv::Matx<float,3,3> matxType(matx2D_2);
    cv::Matx<float,3,4> zeroMatx = cv::Matx<float,3,4>::zeros();
    cv::Matx<float,3,4> oneMatx = cv::Matx<float,3,4>::ones();
    cv::Matx<float,3,4> eyeMatx = cv::Matx<float,3,4>::eye();
    std::cout << zeroMatx << std::endl;
    std::cout << oneMatx.row(1) << std::endl;;
    // ------------------------------------------------------
    // Access: Matx doesnt have .at
    std::cout << matx1D_1.row(0) << std::endl  << matx2D_2 << std::endl
            ;
    std::cout << "Vector 1D" << vec1D(3) << std::endl;
    std::cout << "Matrix 2D" << matxType(3,1) << std::endl;


    //Mat ----------------------------------------------------------
    std::cout << "Begin Mat----------------------------------" << std::endl;
    // Initialize
    float data1[6] = {1,2,3,4,5,6};
    float data2[6] = {10,20,30,40,50,60};
    /*cv::Mat_<cv::Matx<cv::Vec2f,3,4> > mat34_;
    cv::randu(mat34_,20,30);
    mat34_(2,3) = {1000,1000};
    std::cout << mat34_(2,3) << " and " << mat34_(1,3) << std::endl;
    std::cout << mat34_ << std::endl;*/

    cv::Mat mat1(3,4,CV_8UC1);
    cv::Mat mat2(3,4,CV_32FC1);
    cv::randu(mat2,0,100);
    cv::Mat mat3(3,4,CV_32FC1,data1,sizeof(float));
    cv::Mat mat34(3,4,CV_32FC3,cv::Scalar(20,30,40));
    cv::Mat mat4(cv::Size_<int>(3,4),CV_32FC1);
    cv::Mat mat5(cv::Size_<float>(3,4),CV_32FC1,cv::Scalar_<float>(66.6));
    cv::Mat mat6(cv::Size(3,4),CV_32FC1,data2,sizeof(float));
    cv::Mat eyeMat1C = cv::Mat::eye(3,4,CV_32FC1);
    cv::Mat eyeMat2C = cv::Mat::eye(3,4,CV_32FC2);

    int sz[] = {30, 30, 30};
    cv::Mat bigCube1(3, sz, CV_32FC3);
    cv::Mat bigCube2(3, sz, CV_32FC1, cv::Scalar::all(99));
    std::vector<cv::Range> ranges(3, cv::Range(10,20));
    cv::Mat roiRangeMultiple( bigCube1, ranges);
    // Access
    mat34.at<float>(2,3) = 1000;
    std::cout << mat1 << std::endl << mat2 << std::endl << mat3 << std::endl << mat34 << std::endl << mat4 << std::endl
              << mat5 << std::endl << mat6 << std::endl;
    std::cout << eyeMat1C.at<int>(2,2) << eyeMat2C.at<cv::Vec2f>(2,2)[0];
    std::cout << mat34.at<float>(1,3) << " and " << mat34.at<cv::Vec<float,3> >(1,3)[1] << std::endl;
    std::cout << bigCube1.at<float>(5,5,5)  << std::endl << bigCube2.at<float>(5,5,5) << std::endl ;
    std::cout << roiRangeMultiple.at<float>(0,1,1) << std::endl;



    //Mat_ ----------------------------------------------------------
    std::cout << "Begin Mat_----------------------------------" << std::endl;
    // Initalize
    cv::Mat_<cv::Matx<float,3,4>> mat2x_;
    cv::Mat_<char> mat1_(3,4); mat1_ << -1, 1, 2, -2, 3, 1 , 4, 0, 3, 10, 11, 12;
    cv::Mat_<float> mat2_(3,4);
    cv::randu(mat2_,0,100);
    cv::Mat_<float> mat3_(3, 4, data1, sizeof(float)); // some random data and random fill.
    cv::Mat_<cv::Vec3f> mat34_(3,4);
    cv::Mat_<float> mat4_(cv::Size_<int>(3,4));
    cv::Mat_<float> mat5_(cv::Size(3,4));
    //cv::Mat_ mat6_(cv::Size(3,4),CV_32FC1,data2,sizeof(float));
    cv::Mat_<float> eyeMat1C_ = cv::Mat_<float>::eye(3,4);
    cv::Mat_<cv::Vec2f> eyeMat2C_ = cv::Mat_<float>::eye(3,4);
    // Access : Mat_ can be accessed the same way as Mat, but can also be accessed without the template type
    mat34_(2,3) = 1000;
    // Print
    std::cout << mat1_ << std::endl << mat2_ << std::endl << mat34_ << std::endl << mat4_ <<
              std::endl << mat5_ << std::endl;
    std::cout << eyeMat1C_.at<int>(2,2) << eyeMat2C_.at<cv::Vec2f>(2,2)[0];
    std::cout << mat34_(1,3) << " and " << mat34_(1,3)[1] << std::endl;


    // Image Processing Mat
    cv::Mat img_rgb = cv::imread(fpath.string(), CV_LOAD_IMAGE_COLOR);
    cv::Mat img_rgb_orig = cv::imread(fpath.string(), CV_LOAD_IMAGE_COLOR);
    std::cout<<img_rgb_orig.total() << std::endl;
    //img_rgb_orig.convertTo(img_rgb_, CV_8U);

    // Image Process Mat_
    cv::Mat_<cv::Matx<uchar,512,512>> img_rgb_;
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

    /*
    //Very very large operation
    // allocates 1 matrices sized 100 by 100 by 100

    int sz_big[] = {100, 100, 100};
    float data3[6] = {100,200,300,400,500,600};
    cv::Mat bigCube3(3, sz_big, CV_8UC1, data3, 4);

    // allocates 592 matrices sized 47 by 47
    std::vector<cv::Mat> mat(592, cv::Mat(47, 47, CV_32FC1));
    for(auto &m: mat) {
        bigCube1.copyTo(m);
    }
    */

    cv::waitKey(0);



}

void bonus(void) {


    std::vector<cv::Point2f> sides;
    sides.push_back(cv::Point2f(10, 20));
    sides.push_back(cv::Point2f(6, 8));
    sides.push_back(cv::Point2f(5, 5));

    cv::Mat_<float> xpts(3,1), ypts(3,1);

    for ( int i=0; i< sides.size(); i++ ) {
        xpts(i) = sides[i].x;
        ypts(i) = sides[i].y;
    }


}