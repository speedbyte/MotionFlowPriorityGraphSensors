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
#include "OpticalFlow.h"




class AlgorithmFlow : public OpticalFlow {

    // Each point on GroundTruthFlow is a vector of points in AlgorithmFlow. Hence both the base and fast movement
    // consists of an additional vector wrappper.

public:

    const vector<vector<vector<pair<cv::Point2f, cv::Point2f>>>> &
    get_algo_extrapolated_frame_pixel_point_pixel_displacement() const;

private:
    std::string m_resultordner;
    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > m_algo_extrapolated_frame_pixel_point_pixel_displacement;

public:

    AlgorithmFlow( const std::vector<Objects> &list_objects ) : OpticalFlow(list_objects) {}

    void prepare_directories(ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise);

    void calculate_flow(ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise);

    void store_in_yaml(cv::FileStorage &fs, const cv::Point2f &l_pixelposition, const cv::Point2f
    &l_pixelmovement );

    void setResultOrdner(ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise);

    const std::string getResultOrdner() const {
        return m_resultordner;
    }

    void generate_gt_scenepixel_displacement();

};


#endif //MAIN_FLOW_H
