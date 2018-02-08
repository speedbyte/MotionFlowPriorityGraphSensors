//
// Created by veikas on 08.02.18.
//

#ifndef MAIN_OBJECTS_H
#define MAIN_OBJECTS_H


#include <opencv2/core/types.hpp>

class Objects {

protected:

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            m_obj_extrapolated_shape_pixel_point_pixel_displacement;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            m_obj_extrapolated_pixel_centroid_pixel_displacement_mean;

    std::vector<std::vector<bool> > m_obj_extrapolated_mean_visibility;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            m_obj_line_parameters;

    int m_ObjectWidth;

    int m_ObjectHeight;

    unsigned m_objectId;

    std::vector<std::vector<bool> >  m_obj_extrapolated_visibility;

    const std::string m_objectName;



public:

    Objects( std::string objectName) : m_objectName(objectName) {}

    Objects( std::string objectName, int width, int height, std::vector<std::vector<bool> >  extrapolated_visibility) : m_objectName(objectName) , m_ObjectWidth(width), m_ObjectHeight
            (height), m_obj_extrapolated_visibility(extrapolated_visibility) {}

    std::string getObjectName() const {
        return m_objectName;
    }

    void generate_obj_extrapolated_pixel_centroid_pixel_displacement_mean(const unsigned &max_skips);

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >    get_obj_extrapolated_pixel_centroid_pixel_displacement_mean()
    const {
        return m_obj_extrapolated_pixel_centroid_pixel_displacement_mean;
    }

    const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f>>>> &
    get_obj_extrapolated_shape_pixel_point_pixel_displacement() const {
        return m_obj_extrapolated_shape_pixel_point_pixel_displacement;
    }

    int getWidth() const {
        return m_ObjectWidth;
    }

    int getHeight() const {
        return m_ObjectHeight;
    }

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >  get_line_parameters() const {
        return m_obj_line_parameters;
    }

    std::vector<std::vector<bool> >  get_obj_extrapolated_visibility()
    const {
        return m_obj_extrapolated_visibility;
    }

    unsigned getObjectId() const {
        return m_objectId;
    }


};


#endif //MAIN_OBJECTS_H
