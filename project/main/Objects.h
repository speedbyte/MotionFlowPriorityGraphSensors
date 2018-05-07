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

    std::vector<STRUCT_GT_OBJECTS_ALL> m_obj_base_all;

    std::vector<std::vector<STRUCT_GT_OBJECTS_ALL> > m_obj_extrapolated_all;

    std::vector<std::pair<cv::Point2f, cv::Point2f> > m_obj_base_pixel_position_pixel_displacement;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > m_obj_extrapolated_pixel_position_pixel_displacement;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            m_obj_extrapolated_shape_pixel_point_pixel_displacement;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            m_obj_extrapolated_stencil_pixel_point_pixel_displacement;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            m_obj_extrapolated_edge_pixel_point_pixel_displacement;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            m_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement;

    std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > >
            m_list_obj_extrapolated_shape_parameters;

    std::vector<bool> m_obj_base_visibility;

    std::vector<std::vector<bool> >  m_obj_extrapolated_visibility;

    std::vector<std::vector<std::vector<bool> > > m_obj_extrapolated_shape_visibility;

    std::vector<std::vector<bool> > m_obj_extrapolated_mean_visibility;

    std::vector<std::vector<std::vector<cv::Point2f > > >
            m_list_obj_line_parameters;


    int m_ObjectInertialWidth;

    int m_ObjectInertialHeight;

    unsigned m_objectId;

    std::string m_objectName;



public:

    Objects() {}

    Objects( std::string objectName) : m_objectName(objectName) {}

    Objects( std::string objectName, int width, int height, std::vector<std::vector<bool> >  extrapolated_visibility) : m_objectName(objectName) , m_ObjectInertialWidth(width), m_ObjectInertialHeight
            (height), m_obj_extrapolated_visibility(extrapolated_visibility) {}

    void generate_obj_extrapolated_mean_pixel_centroid_pixel_displacement(const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &obj_extrapolated_stencil_pixel_point_pixel_displacement, const std::vector<std::vector<std::vector<bool> > > &obj_extrapolated_blob_visibility, const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &obj_extrapolated_edge_pixel_point_pixel_displacement, std::string post_processing_algorithm);

    void generate_obj_line_parameters( std::string post_processing_algorithm);

    void generate_updated_mean_from_multiple_sensors( std::string post_processing_algorithm,
            const std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > &multi_sensor_input_flow_vector,
            std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > &outer_multiframe_flowvector_sensor_fusion_mean,
            const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &multi_sensor_input_shape,
            std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &outer_multiframe_shape_parameters_sensor_fusion_mean
    );


    const std::string getObjectName() const {
        return m_objectName;
    }

    const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &get_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement() const {
        return m_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement;
    }

const int &getInertialWidth() const {
        return m_ObjectInertialWidth;
    }

    const int &getInertialHeight() const {
        return m_ObjectInertialHeight;
    }

    const std::vector<std::vector<std::vector<cv::Point2f > > > &get_line_parameters() const {
        return m_list_obj_line_parameters;
    }

    const std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > > &get_list_obj_shape_parameters() const {
        return m_list_obj_extrapolated_shape_parameters;
    }

    const std::vector<std::vector<bool> >  &get_obj_extrapolated_visibility() const {
        return m_obj_extrapolated_visibility;
    }

    const std::vector<bool>  &get_obj_base_visibility()    const {
        return m_obj_base_visibility;
    }

    unsigned getObjectId() const {
        return m_objectId;
    }

    const std::vector<std::vector<STRUCT_GT_OBJECTS_ALL> > &getExtrapolatedGroundTruthDetails() const {
        return m_obj_extrapolated_all;
    }

    const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &get_obj_extrapolated_shape_pixel_point_pixel_displacement() const {
        return m_obj_extrapolated_shape_pixel_point_pixel_displacement;
    };

    const std::vector<std::vector<std::vector<bool> > > &get_obj_extrapolated_shape_visibility() const {
        return m_obj_extrapolated_shape_visibility;
    };

    const std::vector<std::vector<bool> > &get_obj_extrapolated_mean_visibility() const {
        return m_obj_extrapolated_mean_visibility;
    };

    const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &get_obj_extrapolated_stencil_pixel_point_pixel_displacement() const {
        return m_obj_extrapolated_stencil_pixel_point_pixel_displacement;
    };

    const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &get_obj_extrapolated_edge_pixel_point_pixel_displacement() const {
        return m_obj_extrapolated_edge_pixel_point_pixel_displacement;
    };


    const std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >  &get_obj_extrapolated_pixel_position_pixel_displacement() const {
        return m_obj_extrapolated_pixel_position_pixel_displacement;
    }

    virtual void generate_obj_extrapolated_shape_pixel_point_pixel_displacement_pixel_visibility(std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_base_movement,  std::vector<std::vector<bool> > outer_base_visibility)  {};

    virtual void generate_obj_extrapolated_stencil_pixel_point_pixel_displacement(std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_stencil_movement ) {};

    void generate_obj_extrapolated_edge_pixel_point_pixel_displacement(std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_edge_movement ) {
        m_obj_extrapolated_edge_pixel_point_pixel_displacement.push_back(outer_edge_movement);
    };


};


#endif //MAIN_OBJECTS_H
