//
// Created by veikas on 06.02.18.
//

#ifndef MAIN_OPTICALFLOW_H
#define MAIN_OPTICALFLOW_H

#include <opencv2/core/types.hpp>
#include "GroundTruthObjects.h"
#include "SimulatedObjects.h"
#include "FlowImageExtended.h"


typedef struct {

    int current_frame_index;
    int obj_index;
    cv::Point2f object_dimension;

    int groundTruthPixels;
    int visiblePixels;

    double allPixels_l1_error;
    double allPixels_l2_error;
    double allPixels_ma_error;

    int goodPixels_l1_error;
    int goodPixels_l2_error;
    int goodPixels_ma_error;

    cv::Point2f gt_mean_displacement;

    // Mean
    cv::Point2f mean_pts;
    cv::Point2f mean_displacement;

    // Stddev
    cv::Point2f stddev_pts;
    cv::Point2f stddev_displacement;

    // Covariance
    cv::Mat covar_pts;
    cv::Mat covar_displacement;

} OPTICAL_FLOW_EVALUATION_METRICS;

typedef struct {

    int current_frame_index;
    int obj_index;
    cv::Point2f collisionpoints;

} OPTICAL_FLOW_COLLISION_METRICS;


class OpticalFlow {

private:

    void frame_stencil_displacement_region_of_interest_method(ushort sensor_index, ushort current_frame_index, const std::vector<cv::Point2f> &frame_next_pts_array, const std::vector<cv::Point2f>  &displacement_array, ushort obj_index, std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_stencil_displacement, std::vector<bool> &frame_stencil_visibility, const std::vector<cv::Point2f>& all_moving_objects_in_frame, const cv::Mat& depth_02_frame);

protected:

    std::string m_resultordner;

    std::string m_opticalFlowName;

    std::vector<ushort> m_evaluation_list;

    std::string m_weather;

    boost::filesystem::path m_GroundTruthImageLocation;

    boost::filesystem::path  m_generatepath;

    boost::filesystem::path  m_collision_object_path;

    boost::filesystem::path  m_flow_occ_path;

    boost::filesystem::path  m_position_occ_path;

    boost::filesystem::path  m_plots_path;

    boost::filesystem::path  m_gnuplots_path;

    std::vector<GroundTruthObjects *> &m_ptr_list_gt_objects;

    std::vector<Objects *> &m_ptr_list_simulated_objects_base;

    std::vector<Objects *> &m_ptr_list_simulated_objects;

    ushort mStepSize;

    std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > > > m_sensor_mean_displacement_points;

    std::vector<std::vector<std::vector<std::vector<OPTICAL_FLOW_COLLISION_METRICS> > > > m_list_sensor_collision_points;

    std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > >m_list_sensor_line_angles;

    std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > > > m_sensor_shape_points;

    std::vector<std::vector<std::vector<std::vector< OPTICAL_FLOW_EVALUATION_METRICS > > > > m_sensor_multiframe_evaluation_data;

    std::vector<std::map<std::pair<float, float>, int> >  m_sensor_scenario_displacement_occurence;

    void getCombination( const std::vector<Objects *> &m_list_objects, std::vector<std::pair<Objects*, Objects*> > &list_of_objects_combination);

    void find_collision_points_given_two_line_parameters(const cv::Point2f lineparameters1, const cv::Point2f lineparameters2,
                                                         std::vector<cv::Point2f> &collision_points);

public:

    OpticalFlow( std::vector<ushort> evaluation_list, std::string weather, std::string opticalFlowName, std::vector<GroundTruthObjects *> &ptr_list_gt_objects, std::vector<Objects *> &ptr_list_simulated_objects_base, std::vector<Objects*>  &ptr_list_simulated_objects, ushort stepSize ) : m_evaluation_list(evaluation_list), m_weather(weather), m_opticalFlowName(opticalFlowName), m_ptr_list_gt_objects(ptr_list_gt_objects), m_ptr_list_simulated_objects_base(ptr_list_simulated_objects_base), m_ptr_list_simulated_objects(ptr_list_simulated_objects), mStepSize(stepSize)  { };

    const std::vector<std::vector<std::vector<std::vector<OPTICAL_FLOW_COLLISION_METRICS > > > > &getCollisionPoints () const {
        return m_list_sensor_collision_points;
    }

    const std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > > &getLineAngles () const  {
        return m_list_sensor_line_angles;
    }

    const std::vector<std::vector<std::vector<std::vector<OPTICAL_FLOW_EVALUATION_METRICS> > > > &get_sensor_multiframe_evaluation_data() const {
        return m_sensor_multiframe_evaluation_data;
    }

    const std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f> > > > > &getMeanDisplacementPoints () const {
        return m_sensor_mean_displacement_points;
    }

    const std::vector<std::map<std::pair<float, float>, int> > &getScenarioDisplacementOccurence() const {
        return m_sensor_scenario_displacement_occurence;
    };

    void prepare_directories_common(ushort SENSOR_COUNT);

    virtual void prepare_directories(ushort SENSOR_COUNT, std::string noise, ushort fps, ushort stepSize) {};

    void generate_collision_points(ushort SENSOR_COUNT);


    void common_flow_frame(ushort sensor_index, ushort current_frame_index,  const std::vector<cv::Point2f> &next_pts_array, const std::vector<cv::Point2f>  &displacement_array,std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &multiframe_stencil_displacement, std::vector<std::vector<std::vector<bool> >  > &multiframe_visibility,
                           std::vector<cv::Point2f> all_moving_objects_in_frame = {}) ;

    void plot_stencil(ushort SENSOR_COUNT);

    void save_flow_vector(ushort SENSOR_COUNT);

    void generate_metrics_optical_flow_algorithm(ushort SENSOR_COUNT);

    void generate_shape_points_sensor_fusion(const ushort &datafilter_index, std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > >  &sensor_shape_points);

    std::string getGeneratePath() const {
        return m_generatepath.string();
    }

    std::string getResultOrdner() const {
        return m_resultordner;
    }

    void generate_pdf_correct_displacement_x();

private:

};

#endif //MAIN_OPTICALFLOW_H
