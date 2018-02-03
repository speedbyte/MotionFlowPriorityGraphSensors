//
// Created by veikas on 26.01.18.
//

#ifndef MAIN_FLOW_H
#define MAIN_FLOW_H

#include "datasets.h"
#include <iostream>
#include <boost/filesystem/path.hpp>
#include <opencv2/core/persistence.hpp>
#include "Dataset.h"
#include "PlotFlow.h"

class AlgorithmFlow {

    // Each point on GroundTruthFlow is a vector of points in AlgorithmFlow. Hence both the base and fast movement
    // consists of an additional vector wrappper.

    std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2i> > > > m_algo_frame_pixel_point_pixel_displacement;
    std::string m_resultordner;

public:

    void prepare_directories(ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise);

    void calculate_flow(ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise);

    void storeData(const std::vector<cv::Point2f> &prev_pts, std::vector<cv::Point2f> &next_pts, const
    std::vector<uchar> status, std::vector<std::vector<std::pair<cv::Point2i,
            cv::Point2i> > > &frame_pixel_point_pixel_displacement);

    void store_in_yaml(cv::FileStorage &fs, const cv::Point2i &l_pixelposition, const cv::Point2i
    &l_pixelmovement );

    void setResultOrdner(ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise);

    const std::string getResultOrdner() const {
        return m_resultordner;
    }

};


#endif //MAIN_FLOW_H
