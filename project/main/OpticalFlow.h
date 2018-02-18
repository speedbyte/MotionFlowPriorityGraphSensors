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

    boost::filesystem::path  m_generatepath;

    boost::filesystem::path  m_collision_obj_path;

    boost::filesystem::path  m_flow_occ_path;

    boost::filesystem::path  m_trajectory_occ_path;

    boost::filesystem::path  m_plots_path;

    std::vector<std::vector<std::vector<cv::Point2f> > > m_frame_skip_collision_points;

    std::vector<std::pair<Objects*, Objects* > > m_list_simulated_objects_combination;

    std::vector<std::pair<Objects*, Objects* > > m_list_gt_objects_combination;

public:

    std::vector<std::vector<std::vector<cv::Point2f> > > getCollisionPoints () const {
        return m_frame_skip_collision_points;
    }

    void prepare_directories();

    void generate_collision_points(const std::vector<Objects* > & m_list_objects_ptr);

    void generate_shape_points(const std::vector<Objects* > & m_list_objects_ptr);

    void generate_collision_points_mean(const std::vector<Objects* > & m_list_gt_objects_ptr, const std::vector<Objects* > & m_list_objects_ptr);

    std::string getGeneratePath() const {
        return m_generatepath.string();
    }

    std::string getResultOrdner() const {
        return m_resultordner;
    }

private:
    void find_collision_points_given_two_line_parameters(const cv::Point2f lineparameters1, const cv::Point2f lineparameters2,
                                                                      cv::Mat &tempMatrix, std::vector<cv::Point2f> &collision_points);

};

#endif //MAIN_OPTICALFLOW_H
