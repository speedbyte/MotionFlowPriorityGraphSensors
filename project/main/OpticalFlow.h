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

    unsigned frame_count;
    unsigned visiblePixels;  // pixels that are visible according to the algorithm. In case of ground truth, all pixels are visible
    unsigned goodPixels;  // subset of visible pixels, and are within a tolerance. In case of ground truth, this is equsl to visiblePixel

} OPTICAL_FLOW_EVALUATION_METRICS;


class OpticalFlow {

private:

protected:

    std::string m_resultordner;

    std::string m_opticalFlowName;

    boost::filesystem::path m_GroundTruthImageLocation;

    boost::filesystem::path  m_generatepath;

    boost::filesystem::path  m_collision_object_path;

    boost::filesystem::path  m_flow_occ_path;

    boost::filesystem::path  m_edge_path;

    boost::filesystem::path  m_position_occ_path;

    boost::filesystem::path  m_plots_path;

    std::vector<Objects *> &m_ptr_list_gt_objects;

    std::vector<Objects *> &m_ptr_list_simulated_objects_base;

    std::vector<Objects *> &m_ptr_list_simulated_objects;

    ushort mStepSize;

    std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > > > m_sensor_mean_displacement_points;

    std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > > > m_list_sensor_collision_points;

    std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > >m_list_sensor_line_angles;

    std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > > > m_sensor_shape_points;

    std::vector<std::vector< OPTICAL_FLOW_EVALUATION_METRICS > > m_sensor_multiframe_evaluation_data;

    std::vector<std::map<std::pair<float, float>, int> >  m_sensor_scenario_displacement_occurence;

    void getCombination( const std::vector<Objects *> &m_list_objects, std::vector<std::pair<Objects*, Objects*> > &list_of_objects_combination);

    void find_collision_points_given_two_line_parameters(const cv::Point2f lineparameters1, const cv::Point2f lineparameters2,
                                                         cv::Mat &tempMatrix, std::vector<cv::Point2f> &collision_points);

public:

    OpticalFlow( std::string opticalFlowName, std::vector<Objects *> &ptr_list_gt_objects, std::vector<Objects *> &ptr_list_simulated_objects_base, std::vector<Objects *> &ptr_list_simulated_objects, ushort stepSize ) :
    m_opticalFlowName(opticalFlowName), m_ptr_list_gt_objects(ptr_list_gt_objects), m_ptr_list_simulated_objects_base(ptr_list_simulated_objects_base), m_ptr_list_simulated_objects(ptr_list_simulated_objects), mStepSize(stepSize)  { };

    const std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f> > > > > & getCollisionPoints () const {
        return m_list_sensor_collision_points;
    }

    const std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > > &getLineAngles () const  {
        return m_list_sensor_line_angles;
    }

    const std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > > > &getShapePoints() const {
        return m_sensor_shape_points;
    }

    const std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f> > > > > &getMeanDisplacementPoints () const {
        return m_sensor_mean_displacement_points;
    }

    const std::vector<std::map<std::pair<float, float>, int> > &getScenarioDisplacementOccurence() const {
        return m_sensor_scenario_displacement_occurence;
    };


    void prepare_directories_common();

    virtual void prepare_directories(ALGO_TYPES algo, std::string noise, ushort fps, ushort stepSize) {};

    void generate_collision_points();

    void save_flow_frame_from_displacement();



    void common_flow_frame(std::string sensor_index_folder_suffix, std::string file_name_input_image, ushort sensor_index, ushort frame_count, cv::Mat &flowFrame, std::vector<cv::Point2f> &next_pts_array, std::vector<cv::Point2f>  &displacement_array,FlowImageExtended &F_png_write, std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &multiframe_stencil_displacement, std::vector<std::vector<std::vector<bool> >  > &multiframe_visibility) ;

    void visualiseStencilAlgorithms(void);

    void generate_metrics_optical_flow_algorithm();

    void generate_metrics_data_processing_algorithm();

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
