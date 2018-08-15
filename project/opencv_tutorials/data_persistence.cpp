
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <iostream>
#include <boost/filesystem/operations.hpp>
#include <opencv/highgui.h>
#include <opencv/cv.hpp>


void xml_yaml_write() {

    //std::srand(time(NULL));
    boost::filesystem::path fpath("../../../datasets/pics_dataset/lena.png");
    if ( !boost::filesystem::exists(fpath) ) {
        std::cerr << "PathNotFound error" << std::endl;
        throw ("FileNotFound error");
    }
    cv::Mat lena(cv::imread("../../../datasets/pics_dataset/lena.png", CV_LOAD_IMAGE_GRAYSCALE));
    if ( lena.data == NULL ) {
        std::cerr << "FileNotFound error" << std::endl;
        throw ("FileNotFound error");
    }
    // XML and YAML START
    cv::Mat whiteImage1C(8,8,CV_8UC1,cv::Scalar(255));
    cv::Mat whiteImage3C(8,8,CV_8UC3,cv::Scalar(255,0,0));

    cv::FileStorage fs("../../../datasets/pics_dataset/test.yml", cv::FileStorage::WRITE);
    if ( !fs.isOpened() ) {
        throw;
    }

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
        cv::FileStorage yaml_check("../../../datasets/pics_dataset/test.yml", cv::FileStorage::READ);
    }
    catch(...) {
        std::cout << "problem in yml file";
    }
    // XML and YAML END
}



void xml_yaml_read() {
    cv::FileStorage fs("../../../datasets/pics_dataset/test.yml", cv::FileStorage::READ);
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


    cv::FileNode main_node = fs["CPP_DATASET"];
    for ( cv::FileNodeIterator iterator_dictNode = main_node.begin(); iterator_dictNode!= main_node.end(); iterator_dictNode++) {

        std::cout << (*iterator_dictNode).name();
        if ( (*iterator_dictNode).name() == "DATAPROCESSING")
            std::cout << "hurra" << (int)(*iterator_dictNode)["val"];
    }
    //cv::imshow("yaml image", create_image);
    //cv::waitKey(0);
}


void nested_yaml() {
    cv::FileStorage write_fs;
    write_fs.open("../trajectory.yml", cv::FileStorage::WRITE);

    for ( unsigned i = 1; i < 3 ; i++ ) {

        char temp_str_fs[20];
        sprintf(temp_str_fs, "i_%03d", i);
        write_fs << temp_str_fs << "[";

        for (ushort j = 0; j < 4; j++) {
            char temp_str_fc[20];
            sprintf(temp_str_fc, "j_%03d", j);
            write_fs << temp_str_fc << "[";
            write_fs << "{:" << "name" << "hello" << "x" << 1 << "y" << 2 << "}";
            write_fs << "]";
        }
        write_fs << "]";
    }
    write_fs.release();
}

void read_nested() {

    cv::FileStorage fs("../trajectory.yml", cv::FileStorage::READ);
    std::vector<cv::Point2f> traj_points;

    cv::FileNode file_node, file_node_temp;
    cv::FileNodeIterator file_node_iterator_begin, file_node_iterator_end, file_node_iterator;

    char temp_str_fc[20];
    sprintf(temp_str_fc, "j_%03d", 1);
    file_node = fs[temp_str_fc];
    if ( file_node.isNone() || file_node.empty() ) {
        std::cout << temp_str_fc << " cannot be found" << std::endl;
    }
    else {

        file_node_iterator_begin = file_node.begin();
        file_node_iterator_end = file_node.end();

        for ( auto i = 0; i < file_node.size() ; i ++ ) {

            std::cout << (*file_node_iterator_begin).name() << file_node.size() << " found" << std::endl;

        }
    }
    fs.release();

}

int main (int argc, char *argv[]) {
    //xml_yaml_write();
    xml_yaml_read();
    //nested_yaml();
    //read_nested();
}

/*
 features:
   - { x:83, y:86, lbp:[ 1, 0, 1, 1, 0, 0, 1, 0 ] }
   - { x:15, y:93, lbp:[ 1, 1, 0, 0, 0, 1, 0, 0 ] }
   - { x:86, y:92, lbp:[ 1, 0, 0, 0, 1, 1, 0, 0 ] }

 */