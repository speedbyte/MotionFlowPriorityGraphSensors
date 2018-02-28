//
// Created by veikas on 08.02.18.
//

#include <iostream>
#include "Objects.h"
#include "Dataset.h"


void Objects::generate_obj_extrapolated_pixel_centroid_pixel_displacement_mean( const unsigned &max_skips,
                                                                                         const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &obj_extrapolated_blob_pixel_point_pixel_displacement) {

    // A blob can be either a stencil or a shape

    for (unsigned frame_skip = 1; frame_skip < max_skips; frame_skip++) {
        std::vector<std::pair<cv::Point2f, cv::Point2f> > multiframe_flowvector;
        std::vector<bool> multiframe_visibility;

        std::cout << "generate_obj_extrapolated_pixel_centroid_pixel_displacement_mean for frame_skip " << frame_skip << std::endl;

        unsigned long FRAME_COUNT = obj_extrapolated_blob_pixel_point_pixel_displacement.at(frame_skip - 1)
                .size();

        for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement
            float mean_pts_x = 0.0f;
            float mean_pts_y = 0.0f;
            float mean_displacement_vector_x = 0.0f;
            float mean_displacement_vector_y = 0.0f;

            const unsigned CLUSTER_SIZE = (unsigned)obj_extrapolated_blob_pixel_point_pixel_displacement.at
                    (frame_skip - 1).at(frame_count).size();
            assert(CLUSTER_SIZE>0);

            for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {
                cv::Point2f pts = obj_extrapolated_blob_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).first;
                cv::Point2f gt_displacement = obj_extrapolated_blob_pixel_point_pixel_displacement.at(frame_skip - 1)
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

            if ( frame_count > 0 ) {
                if ( mean_displacement_vector_x == 0 ) {
                    mean_displacement_vector_x+=0.001;
                }
                //assert(std::abs(mean_displacement_vector_y )>0);
            }
            // I should return the vector instead of points and then normalize it.
            multiframe_flowvector.push_back(std::make_pair(cv::Point2f(mean_pts_x, mean_pts_y), cv::Point2f
                    (mean_displacement_vector_x, mean_displacement_vector_y)));
            multiframe_visibility.push_back(true);
        }
        m_obj_extrapolated_pixel_centroid_pixel_displacement_mean.push_back(multiframe_flowvector);
        m_obj_extrapolated_mean_visibility.push_back(multiframe_visibility);

    }

}


void Objects::generate_obj_line_parameters( const unsigned &max_skips) {

    // starting from here, we reduce the size of the frame_count by 1. This is because, from here we start the calculation of the line.
    // collision points should also be reduced by 1 hence forth.

    //TODO - clean up this section.
    for (unsigned frame_skip = 1; frame_skip < max_skips; frame_skip++) {
        std::vector<std::pair<cv::Point2f, cv::Point2f> > line_parameters;
        const unsigned long FRAME_COUNT = m_obj_extrapolated_pixel_centroid_pixel_displacement_mean.at
                (frame_skip - 1).size() - 1;
        for (unsigned frame_count = 1; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement

            if ( m_obj_extrapolated_mean_visibility.at(frame_skip-1).at(frame_count) == true ) {
                cv::Point2f next_pts = m_obj_extrapolated_pixel_centroid_pixel_displacement_mean.at
                        (frame_skip - 1).at(frame_count).first;
                cv::Point2f  displacement_vector =
                        m_obj_extrapolated_pixel_centroid_pixel_displacement_mean.at
                                (frame_skip - 1).at(frame_count).second;

            //assert(std::abs(mean_displacement_vector_y )>0);


                float m, c;
                m = displacement_vector.y / displacement_vector.x;
                c = next_pts.y - m * next_pts.x;  // c = y - mx

                if ( (int)c == 0 ) {
                    c += 0.001;
                }

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
        m_obj_line_parameters.push_back(line_parameters);
    }
}

