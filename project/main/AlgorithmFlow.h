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
    std::vector<std::pair<SimulatedObjects, SimulatedObjects> > m_list_objects_combination;

    std::vector<SimulatedObjects> &m_list_simulated_objects;

    std::vector< Objects*> m_list_simulated_objects_ptr;


public:

    AlgorithmFlow( std::vector<SimulatedObjects> &list_simulated_objects ) : m_list_simulated_objects(list_simulated_objects)  {

        m_basepath = Dataset::getResultPath();

        for ( unsigned i = 0; i < m_list_simulated_objects.size(); i++ ) {

            m_list_simulated_objects_ptr.push_back(&m_list_simulated_objects.at(i));
        }
    }

    void prepare_directories(ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise);

    void generate_flow_frame(ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise,
                             const std::vector<GroundTruthObjects>& groundtruthobjects);

    void store_in_yaml(cv::FileStorage &fs, const cv::Point2f &l_pixelposition, const cv::Point2f
    &l_pixelmovement );

    const std::string getResultOrdner() const {
        return m_resultordner;
    }

    std::vector<SimulatedObjects> getSimulatedObjects() {
        return m_list_simulated_objects;
    }

    void generate_collision_points() {
        OpticalFlow::generate_collision_points(m_list_simulated_objects_ptr);
    };

};


#endif //MAIN_FLOW_H
