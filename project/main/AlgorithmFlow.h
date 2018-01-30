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

class AlgorithmFlow: public ObjectFlow, public PlotFlow  {

    Dataset m_dataset;

public:

    AlgorithmFlow( Dataset dataset );

    void prepare_result_directories(std::string resultordner);

    void calculate_flow(const boost::filesystem::path dataset_path, ALGO_TYPES algo,
                        FRAME_TYPES frame_types, NOISE_TYPES noise);

    void plot(std::string resultordner);

};


#endif //MAIN_FLOW_H
