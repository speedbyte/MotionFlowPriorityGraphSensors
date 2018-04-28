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



public:

    AlgorithmFlow( std::vector<Objects*> &list_gt_objects, std::vector<Objects*> &list_simulated_base_objects, std::vector<Objects*> &list_simulated_objects, ushort stepSize ) :
    OpticalFlow(list_gt_objects, list_simulated_base_objects, list_simulated_objects, stepSize) {

    }


    void prepare_directories(ALGO_TYPES algo, FRAME_TYPES frame_types, std::string noise, ushort fps, ushort stepSize);

    void generate_flow_frame(ALGO_TYPES algo, FRAME_TYPES frame_types, std::string  noise, ushort fps);

    void store_in_yaml(cv::FileStorage &fs, const cv::Point2f &l_pixelposition, const cv::Point2f
    &l_pixelmovement );

    std::string getImageAbholOrt() const {
        return mImageabholOrt.string();
    }

    void visualiseStencil(void);

};


#endif //MAIN_FLOW_H
