//
// Created by veikas on 06.02.18.
//

#ifndef MAIN_OPTICALFLOW_H
#define MAIN_OPTICALFLOW_H

#include <opencv2/core/types.hpp>
#include "GroundTruthObjects.h"
#include "SimulatedObjects.h"

class OpticalFlow {

protected:

    std::string m_resultordner;

    boost::filesystem::path  m_basepath;

    boost::filesystem::path  m_generatepath;

    boost::filesystem::path  m_collision_obj_path;

    boost::filesystem::path  m_flow_occ_path;

    boost::filesystem::path  m_trajectory_occ_path;

    boost::filesystem::path  m_plots_path;

    std::vector<std::vector<cv::Point2f> >  m_frame_collision_points;

    std::vector<std::vector<std::vector<cv::Point2f> > > m_frame_skip_collision_points;

    std::vector<std::pair<Objects*, Objects* > > m_list_objects_combination;

public:

    std::vector<std::vector<std::vector<cv::Point2f> > > getCollisionPoints () const {
        return m_frame_skip_collision_points;
    }

    void prepare_directories();

    void generate_collision_points(const std::vector<Objects* > & m_list_objects_ptr);

};

#endif //MAIN_OPTICALFLOW_H
