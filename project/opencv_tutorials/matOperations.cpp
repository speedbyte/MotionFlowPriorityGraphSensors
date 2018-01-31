#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <boost/filesystem.hpp>

void point_() {
    // ------------------------------------------------------
    //cv::Point
    cv::Point_<float> point2(10,11);
    cv::Point3_<float> point3_1(5,1,2); // 3D accessed via x,y,z
    cv::Point3_<float> point3_2(10,1,2); // 3D accessed via x,y,z
    // The points are points in space and the x,y,z are distance wrt to origin.
    std::cout << " Dot " << point3_1.dot(point3_2)  << " Cross " << point3_1.cross(point3_2) << std::endl;
    std::cout << point2.x << point2.y << std::endl;
}

void size_() {
    // ------------------------------------------------------
    //cv::Size: same as point, but the access is thru width and height.
    cv::Size_<float> size2_1(5,15);
    std::cout << "Area" << size2_1.area() << "for values " << size2_1.width << size2_1.height << std::endl;
}

void vector_() {
    //std::vector
    std::vector<float> vector_data;
    // ------------------------------------------------------
    //cv::Vec,3. It makes by default a column vector
    cv::Vec<float,10> vec1D_1(3,4,10.0);
    cv::Vec<float,10> vec1D_2(3,3,10.0);
    cv::Vec<float,5> vec1D_3(100,10.0, 333.33, 444.44, 555.55);
    cv::Matx<float,5,1> vec1D(vec1D_3);  // Only 1 row is allowed.
    vec1D_3.all(200.22);

    //Print
    std::cout << vec1D_1 << vec1D_1.t(); // transposing is from column vector to row vector for printing purpose
    std::cout << "Member " << vec1D_1(0) << " Array " << vec1D_1.row(2)  << " col " << vec1D_3.col(0)  << std::endl;
    std::cout << "Vector 1D" << vec1D(3) << std::endl;
}


void matx_() {
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
    std::cout << "Matrix 2D" << matxType(3,1) << std::endl;
}

void mat() {
    //Mat ----------------------------------------------------------
    std::cout << "Begin Mat---------------------- " << std::endl;
    // Initialize
    float data1[6] = {1,2,3,4,5,6};
    float data2[6] = {10,20,30,40,50,60};
    std::vector<int> data_vector = {1,2,3,4,5,6};
    /*cv::Mat_<cv::Matx<cv::Vec2f,3,4> > mat34_;
    cv::randu(mat34_,20,30);
    mat34_(2,3) = {1000,1000};
    std::cout << mat34_(2,3) << " and " << mat34_(1,3) << std::endl;
    std::cout << mat34_ << std::endl;*/

    cv::Mat mat1(3,4,CV_8UC1);
    cv::Mat mat2(3,4,CV_32FC1);
    cv::randu(mat2,0,100);
    cv::Mat mat3(3,4,CV_32FC1,data1,sizeof(float));
    cv::Mat mat_data_vector(3,4,CV_32FC1,data_vector.data(),sizeof(float));
    cv::Mat mat34(3,4,CV_32FC3,cv::Scalar(20,30,40));
    cv::Mat mat4(cv::Size_<int>(3,4),CV_32FC1);
    cv::Mat mat5(cv::Size_<float>(3,4),CV_32FC1,cv::Scalar_<float>(66.6));
    cv::Mat mat6(cv::Size(3,4),CV_32FC1,data2,sizeof(float));
    cv::Mat eyeMat1C = cv::Mat::eye(3,4,CV_32FC1);
    cv::Mat eyeMat2C = cv::Mat::eye(3,4,CV_32FC2);
    cv::Mat createMe;
    createMe.create(3,4,CV_8UC3);

    int sz[] = {30, 30, 30};
    cv::Mat bigCube1(3, sz, CV_32FC3);
    cv::Mat bigCube2(3, sz, CV_32FC1, cv::Scalar::all(99));
    std::vector<cv::Range> ranges(3, cv::Range(10,20));
    cv::Mat roiRangeMultiple( bigCube1, ranges);
    // Access
    mat34.at<float>(2,3) = 1000;
    std::cout << mat1 << std::endl << mat2 << std::endl << mat3 << std::endl << mat34 << std::endl << mat4 <<
              std::endl
              << mat5 << std::endl << mat6 << std::endl;
    std::cout << eyeMat1C.at<int>(2,2) << eyeMat2C.at<cv::Vec2f>(2,2)[0];
    std::cout << mat34.at<float>(1,3) << " and " << mat34.at<cv::Vec<float,3> >(1,3)[1] << std::endl;
    std::cout << bigCube1.at<float>(5,5,5)  << std::endl << bigCube2.at<float>(5,5,5) << std::endl ;
    std::cout << roiRangeMultiple.at<float>(0,1,1) << std::endl;
}

void mat_() {
    //Mat_ ----------------------------------------------------------
    std::cout << "Begin Mat_----------------------------------" << std::endl;
    // Initalize
    float data1[6] = {1,2,3,4,5,6};
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
}

void mat_multiplication() {
    std::srand(time(NULL));

    cv::Mat_<float> mat34(3,3), mat34_t(3,3), mat34_mul_t(3,3), mat34_mul_element(3,3);
    cv::Mat_<float> mat34_div_element(3,3);
    cv::Mat_<float> mat34_inverse(3,3);
    float determinant;
    mat34 << 1,2,3,4,5,6,7,8,10;
    std::cout << "original\n";
    std::cout << mat34<<std::endl;
    std::cout << "-----object multiplications\n";

    std::cout << "transpose object method\n";
    mat34_t = mat34.t();
    std::cout << mat34_t<<std::endl;

    std::cout << "determinant no object method\n";

    std::cout << "matrix element by element object method\n";
    mat34_mul_t = mat34.mul(mat34_t);
    std::cout << mat34_mul_t<<std::endl;

    std::cout << "matrix multiplication object method\n";
    mat34_mul_element = mat34*mat34_t;
    std::cout << mat34_mul_element <<std::endl;

    std::cout << "matrix inverse object method\n";
    mat34_inverse  = mat34.inv();
    std::cout << mat34_inverse <<std::endl;

    std::cout << "-----cv static functions \n";

    std::cout << "determinant static method\n";
    determinant = (float)cv::determinant(mat34);
    std::cout << determinant <<std::endl;

    std::cout << "transpose static method\n";
    cv::transpose(mat34, mat34_t);
    std::cout << mat34_t<<std::endl;

    std::cout << "matrix element by element static multiplication\n";
    cv::multiply(mat34,mat34_t,mat34_mul_element);
    std::cout << mat34_mul_element <<std::endl;

    std::cout << "matrix multiplication static method\n";
    cv::mulTransposed(mat34,mat34_mul_t,0);
    std::cout << mat34_mul_t <<std::endl;

    std::cout << "matrix element by element static division\n";
    cv::divide(mat34,mat34_t,mat34_div_element);
    std::cout << mat34_div_element <<std::endl;

    std::cout << "matrix inverse static method\n";
    cv::invert(mat34,mat34_inverse);
    std::cout << mat34_inverse <<std::endl;

    //cv::MatIterator_<float> iterator;
    //std::vector<float> vector34;
    //if ()
}

void randomize_coordinates() {
    std::srand(time(NULL));

    const int MAX = 10;

    std::vector<cv::Point2i> points(MAX);
    std::vector<cv::Point2i>::iterator it = points.begin();
    std::vector<float> x_pts, y_pts;
    std::vector<float> magnitude;

    int i = 0;
    for ( ; it < points.end(); it++ ) {
        it.operator->()->x = i; //rand()%100;
        it.operator->()->y = i; //rand()%100;
        i++;
    }

    it = points.begin();

    for ( ; it < points.end(); it++ ) {
        x_pts.push_back(it.operator->()->x);
        y_pts.push_back(it.operator->()->y);
        printf("Points x = %i, y = %i \n" ,it.operator->()->x, it.operator->()->y);
    }

    cv::magnitude(x_pts,y_pts,magnitude);
    cv::Mat mag(1,(int)magnitude.size(), CV_32FC1, magnitude.data(), sizeof(float));
    std::cout << mag;

}


void saturation_cast() {
    uchar init_m0[] = {10,10,30};
    cv::Mat m0(3,1,CV_8UC1,init_m0,sizeof(uchar));
    uchar& Vxy = m0.at<uchar>(0);
    uchar& Vxy_nocast = m0.at<uchar>(1);
    std::cout << m0 << std::endl;
    Vxy = cv::saturate_cast<uchar>((Vxy-128)*2 + 128);
    Vxy_nocast = (Vxy_nocast-128)*2 + 128;
    std::cout << m0 << std::endl;
}


void Mat_to_array() {

    int sz[3] = {3,3,3};
    cv::Mat tempMatrix_(3, sz ,CV_32FC3);;
    cv::Mat tempMatrix(3,3,CV_32FC2);
    cv::randu(tempMatrix, 1, 5);

    //assert(tempMatrix.channels() == 3);

    // TODO take all the non 0 data in a float matrix and then call FlowImage Constructor with additional data

    //cv::Vec3f *dataPtr = tempMatrix.ptr<cv::Vec3f>(0); // pointer to the first channel of the first element in the
    // first row. The r, g b  value of single pixels are not continous. all channel elements are continous. So, r is
    // continous and then g is continous and then b is continous.

    //float array[m_dataset.getFrameSize().width*m_dataset.getFrameSize().height][3];
    float *array = (float *)malloc(2*sizeof(float)*3*3);
    //cv::MatConstIterator_<cv::Vec3f> it = tempMatrix.begin<cv::Vec3f>();
    cv::MatConstIterator_<cv::Vec2f> it = tempMatrix.begin<cv::Vec2f>();
    for (unsigned i = 0; it != tempMatrix.end<cv::Vec2f>(); it++ ) {
        std::cout << (*it)[0] << std::endl;
        std::cout << (*it)[1] << std::endl;
        //std::cout << (*it)[2] << std::endl;
        for ( unsigned j = 0; j < 2; j++ ) {
            *(array + i ) = (*it)[j];
            i++;
        }
    }
    std::cout << tempMatrix << std::endl;
    for ( unsigned i = 0; i < 18 ; i++ ) {
        std::cout << *(array+i) << std::endl;
    }
}


int main ( int argc, char *argv[]) {

//    point_();
//    size_();
//    vector_();
//    matx_();
//    mat_();
    //mat();
//    mat_multiplication();
    //randomize_coordinates();
    Mat_to_array();

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