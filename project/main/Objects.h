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

    std::vector<STRUCT_GT_OBJECTS_ALL> m_object_base_all;

    std::vector<std::vector<STRUCT_GT_OBJECTS_ALL> > m_object_all;

    std::vector<std::pair<cv::Point2f, cv::Point2f> > m_object_base_point_displacement;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > m_object_pixel_position_pixel_displacement;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            m_object_stencil_point_displacement;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            m_object_edge_point_displacement;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            m_list_object_mean_centroid_displacement;

    std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > >
            m_list_object_shapepoints_displacement;

    std::vector<bool> m_object_base_visibility;

    std::vector<std::vector<bool> >  m_object_visibility;

    std::vector<std::vector<std::vector<bool> > > m_object_stencil_visibility;

    std::vector<std::vector<bool> > m_object_mean_visibility;

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

    Objects( std::string objectName, int width, int height, std::vector<std::vector<bool> >  extrapolated_visibility) : m_objectName(objectName) , m_ObjectInertialWidth(width), m_ObjectInertialHeight
            (height), m_object_visibility(extrapolated_visibility) {}

    void generate_object_mean_centroid_displacement(const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &obj__stencil_point_displacement, const std::vector<std::vector<std::vector<bool> > > &obj__blob_visibility, const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &obj__edge_point_displacement, std::string post_processing_algorithm);

    void generate_updated_mean_from_multiple_sensors( std::string post_processing_algorithm,
            const std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > &multi_sensor_input_flow_vector,
            std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > &outer_multiframe_flowvector_sensor_fusion_mean,
            const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &multi_sensor_input_shape,
            std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &outer_multiframe_shapepoints_displacement_sensor_fusion_mean
    );


    const std::string getObjectName() const {
        return m_objectName;
    }

    const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &get_list_object_mean_centroid_displacement() const {
        return m_list_object_mean_centroid_displacement;
    }

const int &getInertialWidth() const {
        return m_ObjectInertialWidth;
    }

    const int &getInertialHeight() const {
        return m_ObjectInertialHeight;
    }

    const std::vector<std::vector<std::vector<cv::Point2f > > > &get_list_object_line_parameters() const {
        return m_list_object_line_parameters;
    }

    const std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > > &get_list_object_shapepoints_displacement() const {
        return m_list_object_shapepoints_displacement;
    }

    const std::vector<std::vector<bool> >  &get_object_visibility() const {
        return m_object_visibility;
    }

    const std::vector<bool>  &get_object_base_visibility()    const {
        return m_object_base_visibility;
    }

    unsigned getObjectId() const {
        return m_objectId;
    }

    const std::vector<std::vector<STRUCT_GT_OBJECTS_ALL> > &getExtrapolatedGroundTruthDetails() const {
        return m_object_all;
    }

    const std::vector<std::vector<std::vector<bool> > > &get_object_shape_visibility() const {
        return m_object_stencil_visibility;
    };

    const std::vector<std::vector<bool> > &get_object_mean_visibility() const {
        return m_object_mean_visibility;
    };

    const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &get_object_stencil_point_displacement() const {
        return m_object_stencil_point_displacement;
    };

    const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &get_object_edge_point_displacement() const {
        return m_object_edge_point_displacement;
    };

    const std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >  &get_object_pixel_position_pixel_displacement() const {
        return m_object_pixel_position_pixel_displacement;
    }

    void generate_object_stencil_point_displacement_pixel_visibility(std::string post_processing_algorithm, std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_base_movement,  std::vector<std::vector<bool> > outer_base_visibility);

    void generate_edge_contour(std::string post_processing_algorithm);

};


#endif //MAIN_OBJECTS_H
