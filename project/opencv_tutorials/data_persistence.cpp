
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <iostream>
#include <boost/filesystem/operations.hpp>
#include <opencv/highgui.h>
#include <opencv/cv.hpp>


void xml_yaml_write() {

    //std::srand(time(NULL));
    boost::filesystem::path fpath("../../../pics_dataset/lena.png");
    if ( !boost::filesystem::exists(fpath) ) {
        throw ("FileNotFound error");
    }
    cv::Mat lena(cv::imread("../../../pics_dataset/lena.png", CV_LOAD_IMAGE_GRAYSCALE));
    // XML and YAML START
    cv::Mat whiteImage1C(8,8,CV_8UC1,cv::Scalar(255));
    cv::Mat whiteImage3C(8,8,CV_8UC3,cv::Scalar(255,0,0));

    cv::FileStorage fs("../../../pics_dataset/test.yml", cv::FileStorage::WRITE);

    fs << "frameCount" << 5;
    cv::Mat_<double> cameraMatrix(3,3);
    cameraMatrix << (1000, 0, 320, 0, 1000, 240, 0, 0, 1);
    //cv::Mat cameraMatrix =
    cv::Mat_<double> distCoeffs(5,1);
    distCoeffs << (0.1, 0.01, -0.001, 0, 0);
    fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
    fs << "features" << "[";
    for( int i = 0; i < 3; i++ )
    {
        int x = rand() % 100;
        int y = rand() % 100;
        uchar lbp = rand() % 100;

        fs << "{:" << "x" << x << "y" << y << "lbp" << "[:";
        for( int j = 0; j < 8; j++ )
            fs << ((lbp >> j) & 1);
        fs << "]" << "}";
    }
    fs << "]";

    fs << "whiteImage1C" << whiteImage1C;
    fs << "whiteImage3C" << whiteImage3C;

    //fs << "#lena features";
    fs << "darkfeatures" << "[";
    int row = 0;
    int col = 0;
    assert(lena.type()==CV_8UC1);
    while ( row < lena.rows ) {
        col = 0;
        // save only grayscale value less than 5
        while ( col < lena.cols ) {
            if (lena.at<uchar>(row, col) < 5) {
                fs << "{:" << "row" << row << "col" << col << "lbp" << "[:";
                fs << lena.at<uchar>(row, col);
                fs << "]" << "}";
            }
            col++;
        }
        row++;
    }
    fs << "]";
    fs.release();
    try {
        cv::FileStorage yaml_check("../../../pics_dataset/test.yml", cv::FileStorage::READ);
    }
    catch(...) {
        std::cout << "problem in yml file";
    }
    // XML and YAML END
}


void xml_yaml_read() {
    cv::FileStorage fs("../../../pics_dataset/test.yml", cv::FileStorage::READ);
    cv::Mat cameraMatrix;

    cv::FileNode file_node, file_node_temp;
    cv::FileNodeIterator file_node_iterator_begin, file_node_iterator_end, file_node_iterator;

    file_node = fs["cameraMatrix"];
    file_node_iterator_begin = file_node.begin();
    file_node_iterator_end = file_node.end();
    for ( file_node_iterator = file_node_iterator_begin; file_node_iterator != file_node_iterator_end;
          file_node_iterator++) {
        file_node_temp = *file_node_iterator;
        std::cout << file_node_temp.type() << file_node.size() << file_node_temp.name() << std::endl;
        file_node_temp >> cameraMatrix;
        std::cout << cameraMatrix;
    }

    file_node = fs["darkfeatures"];
    file_node_iterator_begin = file_node.begin();
    file_node_iterator_end = file_node.end();
    cv::Mat create_image(512,512,CV_8UC1,cv::Scalar(255));

    for ( file_node_iterator = file_node_iterator_begin; file_node_iterator != file_node_iterator_end;
          file_node_iterator++) {
        size_t size = (*file_node_iterator)["lbp"].size();
        cv::Vec<uchar,1> data; (*file_node_iterator)["lbp"] >> data;
        create_image.at<uchar>((int)(*file_node_iterator)["row"] ,(int)(*file_node_iterator)["col"]) = data[0];
    }
    cv::namedWindow("yaml image", CV_WINDOW_KEEPRATIO);
    cv::imshow("yaml image", create_image);
    cv::waitKey(0);
}

int main (int argc, char *argv[]) {
    xml_yaml_write();
    xml_yaml_read();
}