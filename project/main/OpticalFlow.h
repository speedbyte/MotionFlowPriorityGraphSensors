//
// Created by veikas on 06.02.18.
//

#ifndef MAIN_OPTICALFLOW_H
#define MAIN_OPTICALFLOW_H

#include <opencv2/core/types.hpp>
#include "GroundTruthObjects.h"
#include "SimulatedObjects.h"
#include "FlowImageExtended.h"
#include "Sensors.h"
#include "MotionFlowTypes.h"



typedef struct {

    ushort all_pixels;

    ushort l1_good_pixels;
    ushort l2_good_pixels;
    ushort ma_good_pixels;

    float l1_cumulative_error_all_pixels;
    float l2_cumulative_error_all_pixels;
    float ma_cumulative_error_all_pixels;

    float l1_cumulative_error_good_pixels;
    float l2_cumulative_error_good_pixels;
    float ma_cumulative_error_good_pixels;

    char sync_point[1];


} COUNT_METRICS;


typedef struct {

    int    frame_number;
    int    obj_index;
    bool   visiblity;

    COUNT_METRICS entire_metrics;
    COUNT_METRICS entire_interpolated_metrics;
    COUNT_METRICS sroi_metrics;
    COUNT_METRICS sroi_interpolated_metrics;


    // reliability
    cv::Mat distribution_matrix;
    ushort reliability;

    // Mean
    cv::Point2f mean_pts;
    cv::Point2f mean_displacement;

    // Stddev
    cv::Point2f stddev_pts;
    cv::Point2f stddev_displacement;

    // Covariance
    cv::Mat covar_pts;
    cv::Mat covar_displacement;

    // Regressin line
    cv::Vec4f regression_line;

} OPTICAL_FLOW_EVALUATION_METRICS;


typedef struct {

    int frame_number;
    cv::Point2f collision_points;

} OPTICAL_FLOW_COLLISION_METRICS;


class OpticalFlow {

private:

    void frame_stencil_displacement_region_of_interest_method(ushort sensor_index, ushort current_frame_index,
            const std::vector<cv::Point2f> &frame_next_pts_array, const std::vector<cv::Point2f>  &displacement_array,
            ushort obj_index, std::vector<std::pair<cv::Point2f, cv::Point2f> > &object_stencil_displacement,
            std::vector< GROUND_TRUTH_CONTOURS > &object_contour_stencil_displacement,
            std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_stencil_disjoint_displacement,
            std::vector<bool> &frame_stencil_visibility, const std::vector<cv::Point2f> &all_moving_objects_in_frame, const cv::Mat& depth_02_frame);

protected:

    std::string m_resultordner;

    boost::filesystem::path m_GroundTruthImageLocation;

    boost::filesystem::path  m_generatepath;

    boost::filesystem::path  m_collision_object_path;

    boost::filesystem::path  m_flow_occ_path;

    boost::filesystem::path  m_position_occ_path;

    boost::filesystem::path  m_plots_path;

    boost::filesystem::path  m_gnuplots_path;

    const std::string m_opticalFlowName;

    const std::vector<ushort> m_evaluation_list;

    const std::string m_noise;

    const ushort mStepSize;

    const std::vector<GroundTruthObjects *> &m_ptr_list_gt_objects;

    const std::vector<Sensors> &m_list_of_gt_sensors;

    const std::shared_ptr<OpticalFlow> m_ptr_gt_flow;


    std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > > > m_sensor_mean_displacement_points;

    std::vector<std::vector<std::vector<std::vector<OPTICAL_FLOW_COLLISION_METRICS> > > > m_list_sensor_collision_points;

    std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > >m_list_sensor_line_angles;

    std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > > > m_sensor_shape_points;

    std::vector<std::vector<std::vector<std::vector< OPTICAL_FLOW_EVALUATION_METRICS > > > > m_sensor_multiframe_evaluation_data;

    std::vector<std::map<std::pair<float, float>, int> >  m_sensor_scenario_displacement_occurence;

    void getCombination( const std::vector<Objects *> &m_list_objects, std::vector<std::pair<Objects*, Objects*> > &list_of_objects_combination);

    void find_collision_points_given_two_line_parameters(const cv::Point2f lineparameters1, const cv::Point2f lineparameters2,
                                                         std::vector<cv::Point2f> &collision_points);

    void common_flow_frame(ushort sensor_index, ushort current_frame_index,  const std::vector<cv::Point2f> &next_pts_array,
            const std::vector<cv::Point2f>  &displacement_array,
            std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &multiframe_stencil_displacement,
            std::vector<std::vector<std::vector< GROUND_TRUTH_CONTOURS > > > &multiframe_contour_stencil_displacement,
            std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &multiframe_stencil_disjoint_displacement,
            std::vector<std::vector<std::vector<bool> >  > &multiframe_visibility,
            std::vector<cv::Point2f> all_moving_objects_in_frame = {}) ;


public:

    OpticalFlow( const std::vector<ushort> evaluation_list, const std::string noise, const std::string opticalFlowName, const std::vector<Sensors> &list_of_gt_sensors_base, const std::vector<GroundTruthObjects *> &ptr_list_gt_objects, const ushort stepSize, const std::shared_ptr<OpticalFlow> ptr_gt_flow ) : m_evaluation_list(evaluation_list), m_noise(noise), m_opticalFlowName(opticalFlowName), m_list_of_gt_sensors(list_of_gt_sensors_base), m_ptr_list_gt_objects(ptr_list_gt_objects), mStepSize(stepSize), m_ptr_gt_flow(ptr_gt_flow)  { };

    OpticalFlow( const std::vector<ushort> evaluation_list, const std::string noise, const std::string opticalFlowName, const std::vector<Sensors> &list_of_gt_sensors_base, const std::vector<GroundTruthObjects *> &ptr_list_gt_objects, const ushort stepSize ) : m_evaluation_list(evaluation_list), m_noise(noise), m_opticalFlowName(opticalFlowName), m_list_of_gt_sensors(list_of_gt_sensors_base), m_ptr_list_gt_objects(ptr_list_gt_objects), mStepSize(stepSize)  { };

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

    void prepare_directories_common();

    void generate_collision_points();

    virtual const std::vector<SimulatedObjects *>& get_simulated_objects_ptr_list() {
        throw;
    }

    void plot_stencil();

    void save_flow_vector();

    void rerun_optical_flow_algorithm_interpolated();

    void generate_metrics_optical_flow_algorithm();

    void generate_shape_points_sensor_fusion(const ushort &datafilter_index, std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > >  &sensor_shape_points);

    std::string getGeneratePath() const {
        return m_generatepath.string();
    }

    std::string getResultOrdner() const {
        return m_resultordner;
    }

    void generate_pdf_correct_displacement_x();

    std::vector<std::pair<float, float>> generate_count_metrics_data(std::string, const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &entire_roi_object, const ushort sensor_index, const ushort current_frame_index,  const ushort obj_index, const std::vector<OPTICAL_FLOW_EVALUATION_METRICS> &evaluationData, COUNT_METRICS &count_anaylsis, cv::Mat &icovar);

    void generate_sroi_intersections();

    void stich_gnuplots();

    std::string getOpticalFlowName() {
        return m_opticalFlowName;
    }

private:

    void show_gnuplot(std::string, const std::vector<std::pair<float, float>>  &xy_pts, const ushort sensor_index, const ushort current_frame_index, const ushort obj_index, const std::vector<OPTICAL_FLOW_EVALUATION_METRICS> &evaluationData, COUNT_METRICS &count_metrics, cv::Mat &icovar);

};

#endif //MAIN_OPTICALFLOW_H
