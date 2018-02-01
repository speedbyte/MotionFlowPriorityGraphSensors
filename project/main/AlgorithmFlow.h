//
// Created by veikas on 26.01.18.
//

#ifndef MAIN_FLOW_H
#define MAIN_FLOW_H

#include "datasets.h"
#include <iostream>
#include <boost/filesystem/path.hpp>
#include "Dataset.h"
#include "PlotFlow.h"

class AlgorithmFlow {

    std::vector<std::pair<cv::Point2i, cv::Point2i> > m_obj_flow_vector_resultframe;

public:

    void prepare_directories(std::string resultordner);

    void calculate_flow(ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise);

    void storeData(const std::vector<cv::Point2f> &prev_pts, std::vector<cv::Point2f> &next_pts, const
    std::vector<uchar> status);

    void store_in_yaml(cv::FileStorage &fs, const cv::Point2i &l_pixelposition, const cv::Point2i
    &l_pixelmovement );


};


#endif //MAIN_FLOW_H
