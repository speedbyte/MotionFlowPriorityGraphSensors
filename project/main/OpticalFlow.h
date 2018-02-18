//
// Created by veikas on 06.02.18.
//

#ifndef MAIN_OPTICALFLOW_H
#define MAIN_OPTICALFLOW_H

#include <opencv2/core/types.hpp>
#include "GroundTruthObjects.h"
#include "SimulatedObjects.h"

class OpticalFlow {

private:

protected:

    std::string m_resultordner;

    boost::filesystem::path  m_generatepath;

    boost::filesystem::path  m_collision_obj_path;

    boost::filesystem::path  m_flow_occ_path;

    boost::filesystem::path  m_trajectory_occ_path;

    boost::filesystem::path  m_plots_path;

    std::vector<GroundTruthObjects> &m_list_gt_objects;

    std::vector<SimulatedObjects> &m_list_simulated_objects;

    std::vector<std::vector<std::vector<cv::Point2f> > > m_frame_skip_collision_points;

    std::vector<std::vector<std::vector<cv::Point2f> > > m_frame_skip_shape_points;


public:



    OpticalFlow( std::vector<GroundTruthObjects> &list_gt_objects, std::vector<SimulatedObjects> &list_simulated_objects ) :
    m_list_gt_objects(list_gt_objects), m_list_simulated_objects(list_simulated_objects) { };

    std::vector<std::vector<std::vector<cv::Point2f> > > getCollisionPoints () const {
        return m_frame_skip_collision_points;
    }

    void prepare_directories();

    void generate_collision_points();

    void generate_shape_points();

    void generate_collision_points_mean();

    std::string getGeneratePath() const {
        return m_generatepath.string();
    }

    std::string getResultOrdner() const {
        return m_resultordner;
    }

private:
    void find_collision_points_given_two_line_parameters(const cv::Point2f lineparameters1, const cv::Point2f lineparameters2,
                                                                      cv::Mat &tempMatrix, std::vector<cv::Point2f> &collision_points);

    void getCombination( std::vector<std::pair<GroundTruthObjects, GroundTruthObjects > > &list_of_gt_objects_combination,
    std::vector<std::pair<SimulatedObjects, SimulatedObjects > > &list_of_simulated_objects_combination);

};

#endif //MAIN_OPTICALFLOW_H
