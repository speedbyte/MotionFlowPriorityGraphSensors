//
// Created by veikas on 02.02.18.
//

#include <opencv/cv.hpp>
#include <iostream>
#include "RobustnessIndex.h"
#include "Dataset.h"


void RobustnessIndex::generatePixelRobustness(const std::string &resultOrdner) {


    cv::Mat temp_image;
    boost::filesystem::directory_iterator end_iter;

    boost::filesystem::path dir_path = Dataset::getResultTrajectoryPath();
    dir_path += "/";
    dir_path += resultOrdner;
    dir_path += "/flow_occ_01";

    std::cout << dir_path.string() << std::endl;
    assert(boost::filesystem::exists(dir_path) != 0);

    std::string file_name, path;
    boost::filesystem::path temp;



    for (boost::filesystem::directory_iterator dir_iter(dir_path); dir_iter != end_iter; ++dir_iter) {
        if (boost::filesystem::is_regular_file(dir_iter->status())) {

            std::string extension = boost::filesystem::extension(*dir_iter);
            if (extension == ".png") {
                std::cout << *dir_iter << std::endl;
                temp = *dir_iter;
                temp_image = cv::imread(temp.string(), cv::IMREAD_COLOR);
                cv::imshow("video", temp_image);
                cv::waitKey(1000);

            } else {

                std::cout << "ignoring extension : " << extension << " path " << *dir_iter << std::endl;

            }
        }
        cv::destroyAllWindows();
    }
    //calcCovarMatrix();
}

void RobustnessIndex::generateVectorRobustness() {

}
