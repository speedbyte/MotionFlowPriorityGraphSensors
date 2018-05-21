//
// Created by veikas on 08.02.18.
//

#ifndef MAIN_OBJECTS_H
#define MAIN_OBJECTS_H


#include <opencv2/core/types.hpp>
#include "ObjectMetaData.h"

class Objects {

private:

protected:

    std::vector<STRUCT_GT_OBJECTS_ALL>
            m_object_base_all;

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

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            m_list_object_mean_centroid_displacement;

    std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > >
            m_list_object_shapepoints_displacement;

    std::vector<bool>
            m_object_base_visibility;

    std::vector<std::vector<bool> >
            m_object_extrapolated_visibility;

    std::vector<std::vector<std::vector<bool> > >
            m_object_stencil_visibility;

    std::vector<std::vector<std::vector<cv::Point2f > > >
            m_list_object_line_parameters;

    int m_ObjectInertialWidth;

    int m_ObjectInertialHeight;

    unsigned m_objectId;

    std::string m_objectName;

    void generate_object_mean_lineparameters( std::string post_processing_algorithm);

public:


    Objects() {}

    Objects( std::string objectName) : m_objectName(objectName) {}

    Objects( std::string objectName, std::vector<std::vector<bool> >  extrapolated_visibility) : m_objectName(objectName), m_object_extrapolated_visibility(extrapolated_visibility) {}

    void generate_object_mean_centroid_displacement(std::string post_processing_algorithm);

    void generate_updated_mean_from_multiple_sensors( std::string post_processing_algorithm,
            const std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > &multi_sensor_input_flow_vector,
            std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > &sensor_multiframe_centroid_displacement_sensor_fusion_mean,
            const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &multi_sensor_input_shape,
            std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &sensor_multiframe_shapepoints_displacement_sensor_fusion_mean
    );


    const std::string getObjectName() const {
        return m_objectName;
    }

    const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &get_list_object_mean_centroid_displacement() const {
        return m_list_object_mean_centroid_displacement;
    }

    const std::vector<std::vector<std::vector<cv::Point2f > > > &get_list_object_line_parameters() const {
        return m_list_object_line_parameters;
    }

    const std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > > &get_list_object_shapepoints_displacement() const {
        return m_list_object_shapepoints_displacement;
    }


    unsigned getObjectId() const {
        return m_objectId;
    }

    const std::vector<std::vector<STRUCT_GT_OBJECTS_ALL> > &getExtrapolatedGroundTruthDetails() const {
        return m_object_extrapolated_all;
    }

    const std::vector<bool>  &get_object_base_visibility()    const {
        return m_object_base_visibility;
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

    void generate_object_stencil_point_displacement_pixel_visibility(std::string post_processing_algorithm);

    void set_object_stencil_point_displacement_pixel_visibility(std::string post_processing_algorithm, std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > sensor_base_movement,  std::vector<std::vector<bool> > sensor_base_visibility);

    void generate_edge_contour(std::string post_processing_algorithm);

};


#endif //MAIN_OBJECTS_H
