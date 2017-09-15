
#include <iostream>
#include <opencv2/core/persistence.hpp>
#include <fstream>
#include <boost/filesystem/path.hpp>

void read_kitti_calibration(boost::filesystem::path calib_path) {


    //cv::FileStorage fs(calib_path.string(), cv::FileStorage::READ);
    std::string name, head;
    std::ifstream is(calib_path.string());
    std::getline(is,head);

}