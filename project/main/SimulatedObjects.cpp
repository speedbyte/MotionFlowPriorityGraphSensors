//
// Created by veikas on 07.02.18.
//

#include "SimulatedObjects.h"
#include "Dataset.h"

unsigned SimulatedObjects::SimulatedobjectCurrentCount = 0;



void SimulatedObjects::generate_obj_extrapolated_shape_pixel_point_pixel_displacement_pixel_visibility(std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_base_movement,  std::vector<std::vector<bool> > outer_base_visibility)  {
    m_obj_extrapolated_shape_pixel_point_pixel_displacement.push_back(outer_base_movement);
    m_obj_extrapolated_shape_visibility.push_back(outer_base_visibility);

}

void SimulatedObjects::generate_obj_extrapolated_stencil_pixel_point_pixel_displacement(std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_stencil_movement) {

    m_obj_extrapolated_stencil_pixel_point_pixel_displacement.push_back(outer_stencil_movement);

}

