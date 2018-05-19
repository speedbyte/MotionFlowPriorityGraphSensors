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

    AlgorithmFlow( std::string opticalFlowName, std::vector<Objects*> &ptr_list_gt_objects, std::vector<Objects*> &ptr_list_simulated_base_objects, std::vector<Objects*> &ptr_list_simulated_objects, ushort stepSize ) :
    OpticalFlow(opticalFlowName, ptr_list_gt_objects, ptr_list_simulated_base_objects, ptr_list_simulated_objects, stepSize) {

    }

    void prepare_directories(ALGO_TYPES algo, std::string noise, ushort fps, ushort stepSize);

    void generate_flow_frame(ALGO_TYPES algo, FRAME_TYPES frame_types, std::string  noise, ushort fps);

    std::string getGroundTruthImageLocation() const {
        return m_GroundTruthImageLocation.string();
    }

};


class Farneback: public AlgorithmFlow {

public:

    Farneback(std::string opticalFlowName, std::vector<Objects*> &ptr_list_gt_objects, std::vector<Objects*> &ptr_list_simulated_base_objects, std::vector<Objects*> &ptr_list_simulated_objects, ushort stepSize ) : AlgorithmFlow( opticalFlowName, ptr_list_gt_objects, ptr_list_simulated_base_objects, ptr_list_simulated_objects, stepSize ) {

    }

};


#endif //MAIN_FLOW_H
