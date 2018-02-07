//
// Created by veikas on 06.02.18.
//

#ifndef MAIN_OPTICALFLOW_H
#define MAIN_OPTICALFLOW_H

#include <opencv2/core/types.hpp>
#include "Objects.h"
#include "SimulatedObjects.h"

class OpticalFlow {

protected:

    const std::vector<Objects> &m_list_objects;

    std::vector<SimulatedObjects> m_list_simulated_objects;


    std::vector<std::vector<cv::Point2f> >  m_frame_collision_points;

    std::vector<std::vector<std::vector<cv::Point2f> > > m_frame_skip_collision_points;

public:

    OpticalFlow(const std::vector<Objects> &list_objects): m_list_objects(list_objects) {};

    std::vector<std::vector<std::vector<cv::Point2f> > > getCollisionPoints () const {
        return m_frame_skip_collision_points;
    }

    std::vector<SimulatedObjects> getSimulatedObjects() {
        return m_list_simulated_objects;
    }




};

#endif //MAIN_OPTICALFLOW_H
