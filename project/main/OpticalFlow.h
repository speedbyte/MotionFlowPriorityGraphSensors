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

    boost::filesystem::path mImageabholOrt;

    boost::filesystem::path  m_generatepath;

    boost::filesystem::path  m_collision_obj_path;

    boost::filesystem::path  m_flow_occ_path;

    boost::filesystem::path  m_position_occ_path;

    boost::filesystem::path  m_plots_path;

    std::vector<Objects *> &m_list_gt_objects;

    std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > > m_list_frame_skip_collision_points;

    std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > > m_frame_skip_shape_points;

    void getCombination( const std::vector<Objects *> &m_list_objects, std::vector<std::pair<Objects*, Objects*> > &list_of_objects_combination);

    void find_collision_points_given_two_line_parameters(const cv::Point2f lineparameters1, const cv::Point2f lineparameters2,
                                                         cv::Mat &tempMatrix, std::vector<cv::Point2f> &collision_points);

public:

    OpticalFlow( std::vector<Objects *> &list_gt_objects ) :
    m_list_gt_objects(list_gt_objects) { };

    std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > > getCollisionPoints () const {
        return m_list_frame_skip_collision_points;
    }

    std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > > getShapePoints () const {
        return m_frame_skip_shape_points;
    }


    void prepare_directories();

    void generate_collision_points();

    void generate_shape_points();

    std::string getGeneratePath() const {
        return m_generatepath.string();
    }

    std::string getResultOrdner() const {
        return m_resultordner;
    }

    void CannyEdgeDetection(std::string temp_result_flow_path, std::string temp_result_edge_path);


private:

};

#endif //MAIN_OPTICALFLOW_H
