//
// Created by veikas on 07.02.18.
//

#ifndef MAIN_SIMULATEDOBJECTS_H
#define MAIN_SIMULATEDOBJECTS_H


#include <opencv2/core/types.hpp>
#include "Objects.h"

class SimulatedObjects : public Objects {
public:

    static unsigned SimulatedobjectCurrentCount; // assingn object id

private:

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            m_obj_extrapolated_stencil_pixel_point_pixel_displacement;


public:

    SimulatedObjects(std::string objectName, int width, int height, std::vector<std::vector<bool> >  extrapolated_visibility) :
            Objects(objectName, width, height, extrapolated_visibility ) {

        m_objectId = SimulatedobjectCurrentCount ;
        SimulatedobjectCurrentCount += 1;
    }


    void generate_obj_extrapolated_shape_pixel_point_pixel_displacement(std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_base_movement ) ;

    void generate_obj_extrapolated_stencil_pixel_point_pixel_displacement(std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_stencil_movement ) ;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > get_obj_extrapolated_stencil_pixel_point_pixel_displacement() const {
        return m_obj_extrapolated_stencil_pixel_point_pixel_displacement;
    };

};


#endif //MAIN_SIMULATEDOBJECTS_H
