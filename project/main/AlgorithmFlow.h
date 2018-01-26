//
// Created by veikas on 26.01.18.
//

#ifndef MAIN_FLOW_H
#define MAIN_FLOW_H

#include "datasets.h"
#include <iostream>
#include <boost/filesystem/path.hpp>
#include "Dataset.h"

class AlgorithmFlow  {

    Dataset m_dataset;

public:

    AlgorithmFlow( Dataset dataset );

    void prepare_result_directories(std::string resultordner);

    void calculate_flow(const boost::filesystem::path dataset_path, const std::string image_input_sha, ALGO_TYPES algo,
                        FRAME_TYPES frame_types, NOISE_TYPES noise);

};


#endif //MAIN_FLOW_H
