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
    ALGO_TYPES mAlgo;

public:

    AlgorithmFlow( ALGO_TYPES algo, std::string opticalFlowName, std::vector<Objects*> &ptr_list_gt_objects, std::vector<Objects*> &ptr_list_simulated_base_objects, std::vector<Objects*> &ptr_list_simulated_objects, ushort stepSize ) : mAlgo(algo),
    OpticalFlow(opticalFlowName, ptr_list_gt_objects, ptr_list_simulated_base_objects, ptr_list_simulated_objects, stepSize) {

    }

    void prepare_directories(std::string noise, ushort fps, ushort stepSize) override ;

    void run_optical_flow_algorithm(FRAME_TYPES frame_types, std::string  noise, ushort fps);

    std::string getGroundTruthImageLocation() const {
        return m_GroundTruthImageLocation.string();
    }

};


class Farneback : public AlgorithmFlow {


public:

    Farneback(ALGO_TYPES algo, std::string opticalFlowName, std::vector<Objects*> &ptr_list_gt_objects, std::vector<Objects*> &ptr_list_simulated_base_objects, std::vector<Objects*> &ptr_list_simulated_objects, ushort stepSize ) : AlgorithmFlow( algo, opticalFlowName, ptr_list_gt_objects, ptr_list_simulated_base_objects, ptr_list_simulated_objects, stepSize ) {

    }

};

class LukasKanade : public AlgorithmFlow {

private:
    ALGO_TYPES mAlgo;

public:

    LukasKanade(ALGO_TYPES algo, std::string opticalFlowName, std::vector<Objects*> &ptr_list_gt_objects, std::vector<Objects*> &ptr_list_simulated_base_objects, std::vector<Objects*> &ptr_list_simulated_objects, ushort stepSize ) : AlgorithmFlow( algo, opticalFlowName, ptr_list_gt_objects, ptr_list_simulated_base_objects, ptr_list_simulated_objects, stepSize ) {

    }


};


#endif //MAIN_FLOW_H
