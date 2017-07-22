#include<iostream>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<boost/filesystem.hpp>
#include"hello.h"

int main ( int argc, char *argv[] ) {
    cv::Mat image;
    boost::filesystem::path path = "./data/image.png";
    image = cv::imread(path.string());
    std::cout << "Hello World " << path.string();
    cv::namedWindow("Hello", CV_WINDOW_AUTOSIZE);
    cv::imshow("Hello", image);
    int colSize = image.cols;
    return 0;
}



Error& Error::operator<<(const fun_ptr fun) {
    if(this->m_valid)
        (this->*fun)();
    return *this;
}


template<typename _type>
Error& Error<_type>::Error operator<<(const _type& value) {
    if(this->m_valid)
        this->m_ss << value;
    return *this;
}
