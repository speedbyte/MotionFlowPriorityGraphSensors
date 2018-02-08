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

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
             m_algo_frame_pixel_point_pixel_displacement;


    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_base_movement;


public:

    SimulatedObjects(unsigned objectId, std::string objectName, int width, int height, std::vector<std::vector<bool> >  extrapolated_visibility) :
            Objects(objectName, width, height, extrapolated_visibility ) {

        m_objectId = SimulatedobjectCurrentCount ;
        SimulatedobjectCurrentCount += 1;
    }


    void set_outer_base_movement(std::vector<std::pair<cv::Point2f, cv::Point2f> > base_movement) {
        outer_base_movement.push_back(base_movement);
    }

    void set_m_obj_extrapolated_shape_pixel_point_pixel_displacement() {
        m_obj_extrapolated_shape_pixel_point_pixel_displacement.push_back(outer_base_movement);
        outer_base_movement.clear();
    }



};


#endif //MAIN_SIMULATEDOBJECTS_H
