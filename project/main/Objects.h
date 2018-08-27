//
// Created by veikas on 08.02.18.
//

#ifndef MAIN_OBJECTS_H
#define MAIN_OBJECTS_H


#include <opencv2/core/types.hpp>
#include "ObjectMetaData.h"


typedef struct {

    cv::Point2f mean_pts;
    cv::Point2f mean_displacement;
    cv::Point2f stddev_pts;
    cv::Point2f stddev_displacement;
    cv::Mat covar_pts;
    cv::Mat covar_displacement;
    cv::Vec4f regression_line;
    cv::Mat_<float> ellipse;

} OBJECTS_MEAN_STDDEV;

class Objects {

private:

protected:

    std::vector<std::vector<STRUCT_GT_OBJECTS_ALL> >
            m_object_extrapolated_all;

    std::vector<std::pair<cv::Point2f, cv::Point2f> >
            m_object_base_point_displacement;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            m_object_extrapolated_point_displacement;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            m_object_stencil_point_displacement;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            m_object_edge_point_displacement;

    std::vector<std::vector<std::vector<OBJECTS_MEAN_STDDEV> > >
            m_list_object_dataprocessing_mean_centroid_displacement;

    std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > >
            m_list_object_dataprocessing_stencil_points_displacement;

    std::vector<std::vector<bool> >
            m_object_extrapolated_visibility;

    std::vector<std::vector<std::vector<bool> > >
            m_object_stencil_visibility;

    std::vector<std::vector<std::vector<cv::Point2f > > >
            m_list_object_line_parameters;

    unsigned m_objectId;

    std::string m_objectName;

    std::vector<std::vector<std::vector<cv::Point2f> > > m_special_region_of_interest;

    void generate_object_mean_lineparameters( ushort SENSOR_COUNT, std::string post_processing_algorithm);

public:


    Objects() {}

    Objects( std::string objectName) : m_objectName(objectName) {}

    Objects( std::string objectName, std::vector<std::vector<bool> >  extrapolated_visibility) : m_objectName(objectName), m_object_extrapolated_visibility(extrapolated_visibility) {}

    void generate_object_mean_centroid_displacement(ushort SENSOR_COUNT, std::string post_processing_algorithm);

    void generate_updated_mean_from_multiple_sensors( std::string post_processing_algorithm,
            const std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > &multi_sensor_input_flow_vector,
            std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > &sensor_multiframe_centroid_displacement_sensor_fusion_mean,
            const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &multi_sensor_input_shape,
            std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &sensor_multiframe_dataprocessing_stencil_points_displacement_sensor_fusion_mean
    );

    virtual void setSpecialRegionOfInterest(std::vector<std::vector<std::vector<cv::Point2f> > > all_object_combination_sensor_special_region_of_interest) {
        throw;
    };


    const std::string getObjectName() const {
        return m_objectName;
    }

    const std::vector<std::vector<std::vector<OBJECTS_MEAN_STDDEV > > > &get_list_object_dataprocessing_mean_centroid_displacement() const {
        return m_list_object_dataprocessing_mean_centroid_displacement;
    }

    const std::vector<std::vector<std::vector<cv::Point2f > > > &get_list_object_line_parameters() const {
        return m_list_object_line_parameters;
    }

    const std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > > &get_list_object_dataprocessing_stencil_points_displacement() const {
        return m_list_object_dataprocessing_stencil_points_displacement;
    }

    unsigned getObjectId() const {
        return m_objectId;
    }

    const std::vector<std::vector<STRUCT_GT_OBJECTS_ALL> > &getExtrapolatedGroundTruthDetails() const {
        return m_object_extrapolated_all;
    }

    const std::vector<std::vector<bool> >  &get_object_extrapolated_visibility() const {
        return m_object_extrapolated_visibility;
    }

    const std::vector<std::vector<std::vector<bool> > > &get_object_stencil_visibility() const {
        return m_object_stencil_visibility;
    };

    const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &get_object_stencil_point_displacement() const {
        return m_object_stencil_point_displacement;
    };

    const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &get_object_edge_point_displacement() const {
        return m_object_edge_point_displacement;
    };

    const std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >  &get_object_extrapolated_point_displacement() const {
        return m_object_extrapolated_point_displacement;
    }

    void push_back_object_stencil_point_displacement_pixel_visibility( std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > sensor_base_movement,  std::vector<std::vector<bool> > sensor_base_visibility);

    void generate_edge_contour(ushort SENSOR_COUNT, std::string post_processing_algorithm);

    const std::vector<std::vector<std::vector<cv::Point2f> > > &get_object_special_region_of_interest() const {
        return m_special_region_of_interest;
    }

};


#endif //MAIN_OBJECTS_H
