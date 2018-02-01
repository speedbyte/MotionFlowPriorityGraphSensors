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
#include "ObjectFlow.h"

class AlgorithmFlow : public ObjectFlow {

public:

    void prepare_directories(std::string resultordner);

    void calculate_flow(ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise);
};


#endif //MAIN_FLOW_H
