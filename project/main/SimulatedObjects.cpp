//
// Created by veikas on 07.02.18.
//

#include "SimulatedObjects.h"
#include "Dataset.h"

unsigned SimulatedObjects::SimulatedobjectCurrentCount = 0;

void SimulatedObjects::generate_simulated_obj_extrapolated_pixel_centroid_pixel_displacement_mean( const unsigned
                                                                                             &max_skips) {


    for (unsigned frame_skip = 1; frame_skip < max_skips; frame_skip++) {
        std::vector<std::pair<cv::Point2f, cv::Point2f> > multiframe_flowvector;
        std::vector<bool> multiframe_visibility;
        unsigned long FRAME_COUNT = m_simulated_obj_extrapolated_shape_pixel_point_pixel_displacement.at(frame_skip - 1)
                .size();
        for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement
            int prev_pts_x = 0;
            int prev_pts_y = 0;
            float next_pts_x = 0.0f;
            float next_pts_y = 0.0f;
            float displacement_vector_x = 0.0f;
            float displacement_vector_y = 0.0f;

            const unsigned CLUSTER_SIZE = (unsigned)m_simulated_obj_extrapolated_shape_pixel_point_pixel_displacement.at
                    (frame_skip - 1).at(frame_count).size();

            for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {
                cv::Point2f pts = m_simulated_obj_extrapolated_shape_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).first;
                cv::Point2f gt_displacement = m_simulated_obj_extrapolated_shape_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).second;
                next_pts_x += pts.x ;
                next_pts_y += pts.y ;
                displacement_vector_x += gt_displacement.x ;
                displacement_vector_y += gt_displacement.y ;
            }
            next_pts_x /= (float)CLUSTER_SIZE;
            next_pts_y /= (float)CLUSTER_SIZE;
            displacement_vector_x =  displacement_vector_x / (float) CLUSTER_SIZE;
            displacement_vector_y = displacement_vector_y / (float) CLUSTER_SIZE;
            //prev_pts_x = next_pts_x - displacement_vector_x;
            //prev_pts_y = next_pts_y - displacement_vector_y;

            // I should return the vector instead of points and then normalize it.
            multiframe_flowvector.push_back(std::make_pair(cv::Point2f(next_pts_x, next_pts_y), cv::Point2f
                    (displacement_vector_x, displacement_vector_y)));
            multiframe_visibility.push_back(true);
        }
        m_simulated_obj_extrapolated_pixel_centroid_pixel_displacement_mean.push_back(multiframe_flowvector);
    }


    //TODO - clean up this section.
    for (unsigned frame_skip = 1; frame_skip < max_skips; frame_skip++) {
        std::vector<std::pair<cv::Point2f, cv::Point2f> > line_parameters;
        const unsigned long FRAME_COUNT = m_simulated_obj_extrapolated_pixel_centroid_pixel_displacement_mean.at
                        (frame_skip
                                                                                                          - 1)
                .size();
        for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement

//            if ( m_obj_extrapolated_mean_visibility.at(frame_skip-1).at(frame_count) == true ) {
            if ( 1 ) {
                cv::Point2f next_pts = m_simulated_obj_extrapolated_pixel_centroid_pixel_displacement_mean.at
                                (frame_skip - 1)
                        .at(frame_count).first;
                cv::Point2f  displacement_vector =
                        m_simulated_obj_extrapolated_pixel_centroid_pixel_displacement_mean.at
                        (frame_skip - 1).at(frame_count).second;

                float m, c;
                m = displacement_vector.y / displacement_vector.x;
                c = next_pts.y - m * next_pts.x;  // c = y - mx

                //float d = (float) sqrt((double) displacement_vector.x * displacement_vector.x +
                //                       (double) displacement_vector.y * displacement_vector.y);
                //displacement_vector.x /= d; // normalized vector in x
                //displacement_vector.y /= d; // normalized vector in y

                cv::Point2f pt2;

                if (std::isinf(m)) {
                    if (displacement_vector.y >  0.0f) {  // going up
                        pt2.x = next_pts.x;
                        pt2.y = Dataset::getFrameSize().height;
                    } else {  // going down
                        pt2.x = next_pts.x;
                        pt2.y = 0;
                    }
                } else if (m == 0) {
                    //std::cout << frame_count << " " << next_pts<<  " " << m << " " << displacement_vector << std::endl;
                    if (std::signbit(m)) { //  going left
                        pt2.x = 0;
                        pt2.y = next_pts.y;
                    } else {  // going right
                        pt2.x = Dataset::getFrameSize().width;
                        pt2.y = next_pts.y;
                    }
                } else {
                    //std::cout << frame_count << " " << next_pts <<  " " << m << " " << pt2<< std::endl;
                    if (displacement_vector.y >  0.0f) {
                        pt2.x = (Dataset::getFrameSize().height - c) / m; //
                        pt2.y = Dataset::getFrameSize().height;
                    } else if (displacement_vector.y < 0.0f) {
                        pt2.x = (-c/m); //
                        pt2.y = 0;
                    }
                }

                line_parameters.push_back(std::make_pair(cv::Point2f(m, c), pt2));
            }
            else {
                line_parameters.push_back(std::make_pair(cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f)));
            }

        }
        m_simulated_obj_line_parameters.push_back(line_parameters);

    }
}

