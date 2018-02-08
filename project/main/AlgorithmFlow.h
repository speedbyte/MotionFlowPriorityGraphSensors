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
    std::string m_resultordner;

    std::vector<std::pair<SimulatedObjects, SimulatedObjects> > m_list_objects_combination;

    const std::vector<Objects> &m_list_objects;

    std::vector<SimulatedObjects> &m_list_simulated_objects;

public:

    AlgorithmFlow( const std::vector<Objects> &list_objects, std::vector<SimulatedObjects>
    &list_simulated_objects ) : m_list_objects(list_objects), m_list_simulated_objects(list_simulated_objects) {}

    void prepare_directories(ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise);

    void generate_flow_frame(ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise);

    void store_in_yaml(cv::FileStorage &fs, const cv::Point2f &l_pixelposition, const cv::Point2f
    &l_pixelmovement );

    void setResultOrdner(ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise);

    const std::string getResultOrdner() const {
        return m_resultordner;
    }

    void generate_collision_points();

    std::vector<SimulatedObjects> getSimulatedObjects() {
        return m_list_simulated_objects;
    }

};


#endif //MAIN_FLOW_H
