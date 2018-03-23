//
// Created by veikas on 08.02.18.
//

#include <iostream>
#include "Objects.h"
#include "Dataset.h"


void Objects::post_processing_obj_extrapolated_stencil_pixel_points_pixel_displacement( ) {

    // A blob can be either a stencil or a shape

    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {
        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> >  > multiframe_method_mean_outer;

        std::cout << "generate_obj_extrapolated_mean_pixel_centroid_pixel_displacement for frame_skip " << frame_skip << " for object name " << m_objectName << " " << std::endl;

        unsigned long FRAME_COUNT = m_obj_extrapolated_stencil_pixel_point_pixel_displacement.at(frame_skip - 1)
                .size();

        for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement
            std::vector<std::pair<cv::Point2f, cv::Point2f> >  multiframe_method_mean_inner;

            bool visibility = m_obj_extrapolated_shape_visibility.at(frame_skip-1).at(frame_count).at(0);

            const unsigned CLUSTER_SIZE = (unsigned)m_obj_extrapolated_stencil_pixel_point_pixel_displacement.at
                    (frame_skip - 1).at(frame_count).size();
            if ( visibility == true ) {
                assert(CLUSTER_SIZE>0);
            }

            for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {
                cv::Point2f pts = m_obj_extrapolated_stencil_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).first;
                cv::Point2f displacement = m_obj_extrapolated_stencil_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).second;

                auto distance = cv::norm(displacement);
                /*if ( dist_algo < 0.1 ) {
                    continue;
                }
                auto dist_err = std::abs(dist_gt - dist_algo);
                if (dist_err < DISTANCE_ERROR_TOLERANCE) {
                    auto angle_err = std::cosh(
                            algo_displacement.dot(displacement) / (dist_gt * dist_algo));
                    if (((std::abs(angle_err)) < ANGLE_ERROR_TOLERANCE)) {

                        multiframe_method_mean_inner.push_back(std::make_pair(pts,displacement));

                    }
                }*/
                // 1. MEAN - add all displacement and points and divide by total size.
                // 2. THRESHOLD MEAN - add all displacement and points that are within a boundary and divide by total found size
                // 3. VOTED MEAN - create vote of the displacement and points and only accept values that have the highest number of occurence
                // 4. RANKED MEAN - create ranks of the displacment and points by accepting only those values that are on the boundary and edges and ignoring the rest.
                // 1 st method

                // 2nd method

                // 3rd method

                // 4th method
            }
            multiframe_method_mean_outer.push_back(multiframe_method_mean_inner);
        }
        m_stencil_postprocessing_method_mean.push_back(multiframe_method_mean_outer);
    }
}


void Objects::generate_obj_extrapolated_mean_pixel_centroid_pixel_displacement( const unsigned &max_skips,
                                                                                         const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &obj_extrapolated_blob_pixel_point_pixel_displacement, const std::vector<std::vector<std::vector<bool> > > &obj_extrapolated_blob_visibility, std::string post_processing_algorithm) {

    // A blob can be either a stencil or a shape
    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> >  > outer_multiframe_flowvector_centroid_mean,
            outer_multiframe_flowvector_threshold_mean,
            outer_multiframe_flowvector_voted_mean,
            outer_multiframe_flowvector_ranked_mean;

    for (unsigned frame_skip = 1; frame_skip < max_skips; frame_skip++) {
        std::vector<std::pair<cv::Point2f, cv::Point2f> > multiframe_flowvector_centroid_mean,
                multiframe_flowvector_threshold_mean,
                multiframe_flowvector_voted_mean,
                multiframe_flowvector_ranked_mean;

        std::vector<bool> multiframe_visibility;

        std::cout << "generate_obj_extrapolated_mean_pixel_centroid_pixel_displacement for frame_skip " << frame_skip << " for object name " << m_objectName << " " << std::endl;

        unsigned long FRAME_COUNT = obj_extrapolated_blob_pixel_point_pixel_displacement.at(frame_skip - 1)
                .size();

        for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement
            float mean_pts_centroid_mean_x = 0.0f;
            float mean_pts_centroid_mean_y = 0.0f;
            float mean_displacement_vector_centroid_mean_x = 0.0f;
            float mean_displacement_vector_centroid_mean_y = 0.0f;

            float mean_pts_threshold_mean_x = 0.0f;
            float mean_pts_threshold_mean_y = 0.0f;
            float mean_displacement_vector_threshold_mean_x = 0.0f;
            float mean_displacement_vector_threshold_mean_y = 0.0f;

            float mean_pts_voted_mean_x = 0.0f;
            float mean_pts_voted_mean_y = 0.0f;
            float mean_displacement_vector_voted_mean_x = 0.0f;
            float mean_displacement_vector_voted_mean_y = 0.0f;

            float mean_pts_ranked_mean_x = 0.0f;
            float mean_pts_ranked_mean_y = 0.0f;
            float mean_displacement_vector_ranked_mean_x = 0.0f;
            float mean_displacement_vector_ranked_mean_y = 0.0f;

            bool mean_visibility = false;

            bool visibility = obj_extrapolated_blob_visibility.at(frame_skip-1).at(frame_count).at(0);

            const unsigned CLUSTER_SIZE = (unsigned)obj_extrapolated_blob_pixel_point_pixel_displacement.at
                    (frame_skip - 1).at(frame_count).size();
            if ( visibility == true ) {
                assert(CLUSTER_SIZE>0);
            }

            cv::Point2f gt_displacement_prev_min, gt_displacement_prev_max;

            for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {
                cv::Point2f pts = obj_extrapolated_blob_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).first;
                cv::Point2f gt_displacement = obj_extrapolated_blob_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).second;

                // 1. CENTROID MEAN - add all displacement and points and divide by total size.
                // 2. THRESHOLD MEAN - add all displacement and points that are within a boundary and divide by total found size
                // 3. VOTED MEAN - create vote of the displacement and points and only accept values that have the highest number of occurence
                // 4. RANKED MEAN - create ranks of the displacment and points by accepting only those values that are on the boundary and edges and ignoring the rest.

                // preprocessing 1 st method
                // Nothing to do

                // preprocesing 2nd method
                gt_displacement_prev_min.x = std::min(gt_displacement_prev_min.x, gt_displacement.x );
                gt_displacement_prev_min.y = std::min(gt_displacement_prev_min.y, gt_displacement.y );
                gt_displacement_prev_max.x = std::max(gt_displacement_prev_max.x, gt_displacement.x );
                gt_displacement_prev_max.y = std::max(gt_displacement_prev_max.y, gt_displacement.y );

                // 3rd method
                // create histogram of values.

                mean_pts_voted_mean_x += pts.x ;
                mean_pts_voted_mean_y += pts.y ;
                mean_displacement_vector_voted_mean_x += gt_displacement.x ;
                mean_displacement_vector_voted_mean_y += gt_displacement.y ;

                // 4th method
                mean_pts_ranked_mean_x += pts.x ;
                mean_pts_ranked_mean_y += pts.y ;
                mean_displacement_vector_ranked_mean_x += gt_displacement.x ;
                mean_displacement_vector_ranked_mean_y += gt_displacement.y ;

                mean_visibility = visibility;
            }

            for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {
                cv::Point2f pts = obj_extrapolated_blob_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).first;
                cv::Point2f gt_displacement = obj_extrapolated_blob_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).second;

                // 1. MEAN - add all displacement and points and divide by total size.
                // 2. THRESHOLD MEAN - add all displacement and points that are within a boundary and divide by total found size
                // 3. VOTED MEAN - create vote of the displacement and points and only accept values that have the highest number of occurence
                // 4. RANKED MEAN - create ranks of the displacment and points by accepting only those values that are on the boundary and edges and ignoring the rest.
                // 1 st method
                mean_pts_centroid_mean_x += pts.x ;
                mean_pts_centroid_mean_y += pts.y ;
                mean_displacement_vector_centroid_mean_x += gt_displacement.x ;
                mean_displacement_vector_centroid_mean_y += gt_displacement.y ;

                // 2nd method
                if ( gt_displacement.x >  (gt_displacement_prev_max.x-gt_displacement_prev_max.x/10 ) &&
                     gt_displacement.x >  (gt_displacement_prev_max.y-gt_displacement_prev_max.y/10 ) &&
                     gt_displacement.x <  (gt_displacement_prev_min.x+gt_displacement_prev_min.x/10 )&&
                     gt_displacement.x <  (gt_displacement_prev_min.x+gt_displacement_prev_min.x/10 ))
                {
                    mean_pts_threshold_mean_x += pts.x ;
                    mean_pts_threshold_mean_y += pts.y ;
                    mean_displacement_vector_threshold_mean_x += gt_displacement.x ;
                    mean_displacement_vector_threshold_mean_y += gt_displacement.y ;
                }

                // 3rd method
                mean_pts_voted_mean_x += pts.x ;
                mean_pts_voted_mean_y += pts.y ;
                mean_displacement_vector_voted_mean_x += gt_displacement.x ;
                mean_displacement_vector_voted_mean_y += gt_displacement.y ;

                // 4th method
                mean_pts_ranked_mean_x += pts.x ;
                mean_pts_ranked_mean_y += pts.y ;
                mean_displacement_vector_ranked_mean_x += gt_displacement.x ;
                mean_displacement_vector_ranked_mean_y += gt_displacement.y ;

                mean_visibility = visibility;
            }


            mean_pts_centroid_mean_x /= (float)CLUSTER_SIZE;
            mean_pts_centroid_mean_y /= (float)CLUSTER_SIZE;
            mean_displacement_vector_centroid_mean_x =  mean_displacement_vector_centroid_mean_x / (float) CLUSTER_SIZE;
            mean_displacement_vector_centroid_mean_y = mean_displacement_vector_centroid_mean_y / (float) CLUSTER_SIZE;

            if ( frame_count > 0 ) {
                assert(std::abs(mean_displacement_vector_centroid_mean_x )>0 || std::abs(mean_displacement_vector_centroid_mean_y )>0);
            }

            multiframe_flowvector_centroid_mean.push_back(std::make_pair(cv::Point2f(mean_pts_centroid_mean_x, mean_pts_centroid_mean_y), cv::Point2f
                    (mean_displacement_vector_centroid_mean_x, mean_displacement_vector_centroid_mean_y)));
            multiframe_flowvector_threshold_mean.push_back(std::make_pair(cv::Point2f(mean_pts_centroid_mean_x, mean_pts_centroid_mean_y), cv::Point2f
                    (mean_displacement_vector_centroid_mean_x, mean_displacement_vector_centroid_mean_y)));
            multiframe_flowvector_voted_mean.push_back(std::make_pair(cv::Point2f(mean_pts_centroid_mean_x, mean_pts_centroid_mean_y), cv::Point2f
                    (mean_displacement_vector_centroid_mean_x, mean_displacement_vector_centroid_mean_y)));
            multiframe_flowvector_ranked_mean.push_back(std::make_pair(cv::Point2f(mean_pts_centroid_mean_x, mean_pts_centroid_mean_y), cv::Point2f
                    (mean_displacement_vector_centroid_mean_x, mean_displacement_vector_centroid_mean_y)));


            multiframe_visibility.push_back(mean_visibility);
        }

        outer_multiframe_flowvector_centroid_mean.push_back(multiframe_flowvector_centroid_mean);
        outer_multiframe_flowvector_threshold_mean.push_back(multiframe_flowvector_centroid_mean);
        outer_multiframe_flowvector_voted_mean.push_back(multiframe_flowvector_centroid_mean);
        outer_multiframe_flowvector_ranked_mean.push_back(multiframe_flowvector_centroid_mean);

        if ( post_processing_algorithm == "ground_truth") {
            m_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement.push_back(outer_multiframe_flowvector_centroid_mean);
        }
        else {
            m_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement.push_back(outer_multiframe_flowvector_centroid_mean);
            m_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement.push_back(outer_multiframe_flowvector_threshold_mean);
            m_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement.push_back(outer_multiframe_flowvector_voted_mean);
            m_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement.push_back(outer_multiframe_flowvector_ranked_mean);
        }
        m_obj_extrapolated_mean_visibility.push_back(multiframe_visibility);
    }
}


void Objects::generate_obj_line_parameters( const unsigned &max_skips, std::string post_processing_algorithm) {

    // starting from here, we reduce the size of the frame_count by 1. This is because, from here we start the calculation of the line.
    // collision points should also be reduced by 1 hence forth.

    unsigned COUNT;
    if ( post_processing_algorithm == "ground_truth") {
        COUNT = 1;
    }
    else {
        COUNT = 4;
    }

    for ( unsigned i = 0; i < COUNT; i++ ) {
        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_line_parameters;

        for (unsigned frame_skip = 1; frame_skip < max_skips; frame_skip++) {
            std::vector<std::pair<cv::Point2f, cv::Point2f> > line_parameters;
            const unsigned long FRAME_COUNT =
                    m_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement.at(i).at
                            (frame_skip - 1).size() - 1;

            for (unsigned frame_count = 1; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement

                if (m_obj_extrapolated_mean_visibility.at(frame_skip - 1).at(frame_count) == true) {
                    cv::Point2f next_pts = m_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement.at(i).at
                            (frame_skip - 1).at(frame_count).first;
                    cv::Point2f displacement_vector =
                            m_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement.at(i).at
                                    (frame_skip - 1).at(frame_count).second;

                    //assert(std::abs(mean_displacement_vector_y )>0);

                    float m, c;
                    m = displacement_vector.y / displacement_vector.x;
                    c = next_pts.y - m * next_pts.x;  // c = y - mx

                    if ((int) c == 0) {
                        c += 0.001;
                    }

                    //float d = (float) sqrt((double) displacement_vector.x * displacement_vector.x +
                    //                       (double) displacement_vector.y * displacement_vector.y);
                    //displacement_vector.x /= d; // normalized vector in x
                    //displacement_vector.y /= d; // normalized vector in y

                    cv::Point2f pt2;

                    if (std::isinf(m)) {
                        if (displacement_vector.y > 0.0f) {  // going up
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
                        if (displacement_vector.y > 0.0f) {
                            pt2.x = (Dataset::getFrameSize().height - c) / m; //
                            pt2.y = Dataset::getFrameSize().height;
                        } else if (displacement_vector.y < 0.0f) {
                            pt2.x = (-c / m); //
                            pt2.y = 0;
                        }
                    }
                    line_parameters.push_back(std::make_pair(cv::Point2f(m, c), pt2));
                } else {
                    // we want to maintain the size of the array.
                    line_parameters.push_back(std::make_pair(cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f)));
                }
            }
            outer_line_parameters.push_back(line_parameters);
        }
        m_list_obj_line_parameters.push_back(outer_line_parameters);
    }
}

