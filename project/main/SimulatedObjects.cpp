//
// Created by veikas on 07.02.18.
//

#include "SimulatedObjects.h"
#include "Dataset.h"

unsigned SimulatedObjects::SimulatedobjectCurrentCount = 0;



void SimulatedObjects::generate_obj_extrapolated_shape_pixel_point_pixel_displacement(std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_base_movement) {
    m_obj_extrapolated_shape_pixel_point_pixel_displacement.push_back(outer_base_movement);
}

void SimulatedObjects::generate_obj_extrapolated_stencil_pixel_point_pixel_displacement(std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_stencil_movement) {
    m_obj_extrapolated_stencil_pixel_point_pixel_displacement.push_back(outer_stencil_movement);
}


void SimulatedObjects::generate_obj_extrapolated_pixel_centroid_pixel_displacement_mean( const unsigned &max_skips) {

    for (unsigned frame_skip = 1; frame_skip < max_skips; frame_skip++) {
        std::vector<std::pair<cv::Point2f, cv::Point2f> > multiframe_flowvector;
        std::vector<bool> multiframe_visibility;
        unsigned long FRAME_COUNT = m_obj_extrapolated_stencil_pixel_point_pixel_displacement.at(frame_skip - 1)
                .size();

        for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement
            float mean_pts_x = 0.0f;
            float mean_pts_y = 0.0f;
            float mean_displacement_vector_x = 0.0f;
            float mean_displacement_vector_y = 0.0f;

            const unsigned CLUSTER_SIZE = (unsigned)m_obj_extrapolated_stencil_pixel_point_pixel_displacement.at
                    (frame_skip - 1).at(frame_count).size();

            for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {
                cv::Point2f pts = m_obj_extrapolated_stencil_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).first;
                cv::Point2f gt_displacement = m_obj_extrapolated_stencil_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).second;
                mean_pts_x += pts.x ;
                mean_pts_y += pts.y ;
                mean_displacement_vector_x += gt_displacement.x ;
                mean_displacement_vector_y += gt_displacement.y ;
            }
            mean_pts_x /= (float)CLUSTER_SIZE;
            mean_pts_y /= (float)CLUSTER_SIZE;
            mean_displacement_vector_x =  mean_displacement_vector_x / (float) CLUSTER_SIZE;
            mean_displacement_vector_y = mean_displacement_vector_y / (float) CLUSTER_SIZE;

            // I should return the vector instead of points and then normalize it.
            multiframe_flowvector.push_back(std::make_pair(cv::Point2f(mean_pts_x, mean_pts_y), cv::Point2f
                    (mean_displacement_vector_x, mean_displacement_vector_y)));
            multiframe_visibility.push_back(true);
        }
        m_obj_extrapolated_pixel_centroid_pixel_displacement_mean.push_back(multiframe_flowvector);
        m_obj_extrapolated_mean_visibility.push_back(multiframe_visibility);

    }

}
