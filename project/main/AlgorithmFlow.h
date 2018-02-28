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

private:

    boost::filesystem::path mImageabholOrt;


public:

    AlgorithmFlow( std::string environment, std::vector<Objects*> &list_gt_objects, std::vector<Objects*> &list_simulated_objects ) :
    OpticalFlow(list_gt_objects, list_simulated_objects) {

        mImageabholOrt = Dataset::getGroundTruthPath().string() + "/" + environment + "/";

    }


    void prepare_directories(ALGO_TYPES algo, FRAME_TYPES frame_types, std::string noise);

    void generate_flow_frame(ALGO_TYPES algo, FRAME_TYPES frame_types, std::string  noise,
                             const std::vector<SimulatedObjects>& groundtruthobjects);

    void store_in_yaml(cv::FileStorage &fs, const cv::Point2f &l_pixelposition, const cv::Point2f
    &l_pixelmovement );

    const std::string getResultOrdner() const {
        return m_resultordner;
    }

    void generate_collision_points_mean() {
        OpticalFlow::generate_collision_points_mean();
    };

    std::string getImageAbholOrt() const {
        return mImageabholOrt.string();
    }

};


#endif //MAIN_FLOW_H
