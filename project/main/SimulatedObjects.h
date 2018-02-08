//
// Created by veikas on 07.02.18.
//

#ifndef MAIN_SIMULATEDOBJECTS_H
#define MAIN_SIMULATEDOBJECTS_H


#include <opencv2/core/types.hpp>
#include "Objects.h"

class SimulatedObjects : public Objects {
private:

    static unsigned SimulatedobjectCurrentCount; // assingn object id

    const unsigned m_objectId;

    const std::string m_objectName;

    int m_ObjectWidth;

    int m_ObjectHeight;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
             m_algo_frame_pixel_point_pixel_displacement;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            m_obj_extrapolated_shape_pixel_point_pixel_displacement;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            m_obj_extrapolated_pixel_centroid_pixel_displacement_mean;

    std::vector<std::vector<bool> > m_obj_extrapolated_mean_visibility;


    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            m_obj_line_parameters;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_base_movement;

    std::vector<std::vector<bool> >  m_obj_extrapolated_visibility;

public:

    SimulatedObjects(unsigned objectId, std::string objectName, int width, int height, std::vector<std::vector<bool> >  extrapolated_visibility) :
            m_objectId(objectId), m_objectName(objectName), m_ObjectWidth(width), m_ObjectHeight
            (height), m_obj_extrapolated_visibility(extrapolated_visibility) {
        SimulatedobjectCurrentCount += 1;
    }

    void generate_simulated_obj_extrapolated_pixel_centroid_pixel_displacement_mean(const unsigned &max_skips);

    const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f>>>> &
    get_obj_extrapolated_shape_pixel_point_pixel_displacement() const {
        return m_obj_extrapolated_shape_pixel_point_pixel_displacement;
    }

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >  get_line_parameters() const {
        return m_obj_line_parameters;
    }

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
    get_obj_extrapolated_pixel_centroid_pixel_displacement_mean()
    const {
        return m_obj_extrapolated_pixel_centroid_pixel_displacement_mean;
    }

    unsigned getObjectId() const {
        return m_objectId;
    }

    std::string getObjectName() const {
        return m_objectName;
    }

    void set_outer_base_movement(std::vector<std::pair<cv::Point2f, cv::Point2f> > base_movement) {
        outer_base_movement.push_back(base_movement);
    }

    void set_m_obj_extrapolated_shape_pixel_point_pixel_displacement() {
        m_obj_extrapolated_shape_pixel_point_pixel_displacement.push_back(outer_base_movement);
        outer_base_movement.clear();
    }

    int getWidth() const {
        return m_ObjectWidth;
    }

    int getHeight() const {
        return m_ObjectHeight;
    }

    std::vector<std::vector<bool> >  get_obj_extrapolated_visibility()
    const {
        return m_obj_extrapolated_visibility;
    }

};


#endif //MAIN_SIMULATEDOBJECTS_H
