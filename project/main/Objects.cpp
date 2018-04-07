//
// Created by veikas on 08.02.18.
//

#include <iostream>
#include <map>
#include "Objects.h"
#include "Dataset.h"


void Objects::generate_obj_extrapolated_mean_pixel_centroid_pixel_displacement( const unsigned &max_skips,
                                                                                         const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &obj_extrapolated_blob_pixel_point_pixel_displacement, const std::vector<std::vector<std::vector<bool> > > &obj_extrapolated_blob_visibility, const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &obj_extrapolated_edge_pixel_point_pixel_displacement, std::string post_processing_algorithm) {

    // A blob can be either a stencil or a shape
    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> >  > outer_multiframe_mean_pixel_centroid_pixel_displacement;
    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> >  >  > outer_multiframe_shape_parameters;

    for (unsigned frame_skip = 1; frame_skip < max_skips; frame_skip++) {

        std::vector<std::pair<cv::Point2f, cv::Point2f> >
                multiframe_flowvector_centroid_mean,
                multiframe_flowvector_threshold_mean,
                multiframe_flowvector_voted_mean,
                multiframe_flowvector_ranked_mean;

        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
                outer_multiframe_shape_parameters_centroid_mean,
                outer_multiframe_shape_parameters_threshold_mean,
                outer_multiframe_shape_parameters_voted_mean,
                outer_multiframe_shape_parameters_ranked_mean;

        std::vector<bool> multiframe_visibility;

        std::cout << "generate_obj_extrapolated_mean_pixel_centroid_pixel_displacement for frame_skip " << frame_skip << " for object name " << m_objectName << " " << std::endl;

        unsigned long FRAME_COUNT = obj_extrapolated_blob_pixel_point_pixel_displacement.at(frame_skip - 1)
                .size();

        for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
            // gt_displacement

            std::vector<std::pair<cv::Point2f, cv::Point2f> >
                    multiframe_shape_parameters_centroid_mean,
                    multiframe_shape_parameters_threshold_mean,
                    multiframe_shape_parameters_voted_mean,
                    multiframe_shape_parameters_ranked_mean;

            std::cout << frame_count << std::endl;

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

            std::vector<float> data_x, data_y;
            std::vector<float> data_x_pts, data_y_pts;

            bool mean_visibility = false;

            bool visibility = obj_extrapolated_blob_visibility.at(frame_skip - 1).at(frame_count).at(0);

            const unsigned CLUSTER_SIZE = (unsigned) obj_extrapolated_blob_pixel_point_pixel_displacement.at
                    (frame_skip - 1).at(frame_count).size();

            if (visibility == true) {
                assert(CLUSTER_SIZE > 0);
            }

            cv::Point2f gt_displacement_threshold_min, gt_displacement_threshold_max;

            cv::Point2f gt_pts_compare;
            cv::Point2f gt_displacement_compare;

            for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {
                cv::Point2f pts = obj_extrapolated_blob_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).first;
                cv::Point2f gt_displacement = obj_extrapolated_blob_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).second;

                // 1. CENTROID MEAN - add all displacement and points and divide by total size.
                // 2. THRESHOLD MEAN - add all displacement and points that are within a boundary and divide by total found size
                // 3. VOTED MEAN - create vote of the displacement and points and only accept values that have the highest number of occurence
                // 4. RANKED MEAN - create ranks of the displacment and points by accepting only those values that are on the boundary and edges and ignoring the rest.

                // preprocessing 1st method
                // Nothing to do

                // preprocesing 2nd method
                // Nothing to do

                // 3rd method
                data_x.push_back(std::round(gt_displacement.x * 100) / 100);
                data_y.push_back(std::round(gt_displacement.y * 100) / 100);

                // preprocesing 4th method
                data_x_pts.push_back(pts.x);
                data_y_pts.push_back(pts.y);

                mean_visibility = visibility;
            }

            gt_displacement_threshold_min.x = *std::min_element(data_x.begin(), data_x.end());
            gt_displacement_threshold_min.y = *std::min_element(data_y.begin(), data_y.end());
            gt_displacement_threshold_max.x = *std::max_element(data_x.begin(), data_x.end());
            gt_displacement_threshold_max.y = *std::max_element(data_y.begin(), data_y.end());

            // create histogram of values for displacement.
            std::map<float, int> histogram_x, histogram_y;
            for (const auto &e : data_x) {
                ++histogram_x[e];
            }
            for (const auto &e : data_y) {
                ++histogram_y[e];
            }
            cv::Point2f max_voted = {};

            for (const auto &x : histogram_x) {
                if (max_voted.x <= x.second) {
                    max_voted.x = x.second;
                    gt_displacement_compare.x = x.first;
                    std::cout << x.first << " histogram " << x.second << "endx\n";
                }
            }
            std::cout << gt_displacement_compare.x << " histogram " << max_voted.x << "endx\n";
            for (const auto &y : histogram_y) {
                if (max_voted.y <= y.second) {
                    max_voted.y = y.second;
                    gt_displacement_compare.y = y.first;
                    //std::cout << y.first << " histogram " << y.second << "endy\n";
                }
            }
            std::cout << gt_displacement_compare.y << " histogram " << max_voted.y << "endy\n";

            // create histogram of values for displacement.
            std::map<float, int> histogram_x_pts, histogram_y_pts;
            for (const auto &e : data_x_pts) {
                ++histogram_x_pts[e];
            }
            for (const auto &e : data_y_pts) {
                ++histogram_y_pts[e];
            }
            cv::Point2f max_voted_pts = {};

            for (const auto &x : histogram_x_pts) {
                if (max_voted_pts.x <= x.second) {
                    max_voted_pts.x = x.second;
                    gt_pts_compare.x = x.first;
                    //std::cout << x.first << " histogram_pts " << x.second <<"endx\n";
                }
            }
            for (const auto &y : histogram_y_pts) {
                if (max_voted_pts.y <= y.second) {
                    max_voted_pts.y = y.second;
                    gt_pts_compare.y = y.first;
                    //std::cout << y.first << " histogram_pts " << y.second <<"endy\n";
                }
            }

            unsigned cluster_size_centroid_mean_x = 0,
                    cluster_size_centroid_mean_y = 0,
                    cluster_size_threshold_mean_x = 0,
                    cluster_size_threshold_mean_y = 0,
                    cluster_size_voted_mean_x = 0,
                    cluster_size_voted_mean_y = 0,
                    cluster_size_ranked_mean_x = 0,
                    cluster_size_ranked_mean_y = 0;

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
                mean_pts_centroid_mean_x += pts.x;
                mean_pts_centroid_mean_y += pts.y;
                mean_displacement_vector_centroid_mean_x += gt_displacement.x;
                mean_displacement_vector_centroid_mean_y += gt_displacement.y;
                cluster_size_centroid_mean_x++;
                cluster_size_centroid_mean_y++;
                multiframe_shape_parameters_centroid_mean.push_back(
                        std::make_pair(cv::Point2f(pts.x, pts.y), cv::Point2f
                                (gt_displacement.x, gt_displacement.y)));


                // 2nd method

                cv::Point2f temp_pts, temp_displacement;

                if (gt_displacement.x > ((gt_displacement_threshold_min.x + gt_displacement_threshold_max.x) / 2)) {
                    mean_displacement_vector_threshold_mean_x += gt_displacement_threshold_max.x;
                    temp_displacement.x = gt_displacement_threshold_max.x;
                } else if (gt_displacement.x <=
                        ((gt_displacement_threshold_min.x + gt_displacement_threshold_max.x) / 2)) {
                    mean_displacement_vector_threshold_mean_x += gt_displacement_threshold_min.x;
                    temp_displacement.x = gt_displacement_threshold_min.x;
                }

                if (gt_displacement.y > ((gt_displacement_threshold_min.x + gt_displacement_threshold_max.x) / 2)) {
                    mean_displacement_vector_threshold_mean_x += gt_displacement_threshold_max.x;
                    temp_displacement.y = gt_displacement_threshold_max.y;
                } else if (gt_displacement.y <=
                        ((gt_displacement_threshold_min.x + gt_displacement_threshold_max.x) / 2)) {
                    mean_displacement_vector_threshold_mean_x += gt_displacement_threshold_min.x;
                    temp_displacement.y = gt_displacement_threshold_min.y;
                }
                mean_pts_threshold_mean_x += pts.x;
                cluster_size_threshold_mean_x++;
                mean_pts_threshold_mean_y += pts.y;
                cluster_size_threshold_mean_y++;
                multiframe_shape_parameters_threshold_mean.push_back(
                        std::make_pair(cv::Point2f(pts.x, pts.y), cv::Point2f
                                (temp_displacement.x, temp_displacement.y)));


                // 3rd method
                ushort WEIGHT = 100;
                if (((std::round(gt_displacement.x * 100) / 100) < ((gt_displacement_compare.x) + (gt_displacement_compare.x)/10.0f) &&
                        (std::round(gt_displacement.x * 100) / 100) >= ((gt_displacement_compare.x) - (gt_displacement_compare.x)/10.0f)) ||
                        ((std::round(gt_displacement.y * 100) / 100) < ((gt_displacement_compare.y) + (gt_displacement_compare.y)/10.0f) &&
                         (std::round(gt_displacement.y * 100) / 100) >= ((gt_displacement_compare.y) - (gt_displacement_compare.y)/10.0f))
                        ) {
                    mean_pts_voted_mean_x +=  WEIGHT*pts.x;
                    mean_pts_voted_mean_y +=  WEIGHT*pts.y;
                    mean_displacement_vector_voted_mean_x += WEIGHT*gt_displacement_compare.x;
                    mean_displacement_vector_voted_mean_y += WEIGHT*gt_displacement_compare.y;
                    cluster_size_voted_mean_x += WEIGHT;
                    cluster_size_voted_mean_y += WEIGHT;
                }

                // 4th method


                mean_visibility = visibility;
            }

            if ( post_processing_algorithm != "ground_truth") {

                const unsigned CLUSTER_EDGE_SIZE = (unsigned) obj_extrapolated_edge_pixel_point_pixel_displacement.at
                        (frame_skip - 1).at(frame_count).size();

                ushort WEIGHT = 100;

                for (unsigned cluster_point = 0; cluster_point < CLUSTER_EDGE_SIZE; cluster_point++) {

                    cv::Point2f pts_edge = obj_extrapolated_edge_pixel_point_pixel_displacement.at(
                                    frame_skip - 1)
                            .at(frame_count).at(cluster_point).first;
                    cv::Point2f gt_displacement = obj_extrapolated_edge_pixel_point_pixel_displacement.at(
                                    frame_skip - 1)
                            .at(frame_count).at(cluster_point).second;

                    mean_pts_ranked_mean_x += WEIGHT * pts_edge.x;
                    mean_pts_ranked_mean_y += WEIGHT * pts_edge.y;

                    mean_displacement_vector_ranked_mean_x += WEIGHT * gt_displacement.x;
                    mean_displacement_vector_ranked_mean_y += WEIGHT * gt_displacement.y;
                    cluster_size_ranked_mean_x += WEIGHT;
                    cluster_size_ranked_mean_y += WEIGHT;

                }
            }

            //cluster_size_ranked_mean_x = 0;
            //cluster_size_ranked_mean_y = 0;

            for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {

                cv::Point2f pts = obj_extrapolated_blob_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).first;
                cv::Point2f gt_displacement = obj_extrapolated_blob_pixel_point_pixel_displacement.at(frame_skip - 1)
                        .at(frame_count).at(cluster_point).second;

                mean_pts_ranked_mean_x += pts.x;
                mean_pts_ranked_mean_y += pts.y;
                mean_displacement_vector_ranked_mean_x += gt_displacement.x;
                mean_displacement_vector_ranked_mean_y += gt_displacement.y;
                cluster_size_ranked_mean_x++;
                cluster_size_ranked_mean_y++;
                // 4th method
                multiframe_shape_parameters_ranked_mean.push_back(std::make_pair(cv::Point2f(pts.x, pts.y), cv::Point2f
                        (mean_displacement_vector_ranked_mean_x, mean_displacement_vector_ranked_mean_y)));


                mean_pts_voted_mean_x +=  pts.x;
                mean_pts_voted_mean_y +=  pts.y;
                mean_displacement_vector_voted_mean_x += gt_displacement.x;
                mean_displacement_vector_voted_mean_y += gt_displacement.y;
                cluster_size_voted_mean_x++;
                cluster_size_voted_mean_y++;
                multiframe_shape_parameters_voted_mean.push_back(
                        std::make_pair(cv::Point2f(pts.x, pts.y), cv::Point2f
                                (gt_displacement_compare.x, gt_displacement_compare.y)));
            }

            if ( frame_count > 0 ) {
                assert(cluster_size_centroid_mean_x > 0);
                assert(cluster_size_centroid_mean_y > 0);
                assert(cluster_size_threshold_mean_x > 0);
                assert(cluster_size_threshold_mean_y > 0);
                assert(cluster_size_voted_mean_x > 0);
                assert(cluster_size_voted_mean_y > 0);
                assert(cluster_size_ranked_mean_x > 0);
                assert(cluster_size_ranked_mean_y > 0);
            }

            mean_pts_centroid_mean_x /= (float)cluster_size_centroid_mean_x;
            mean_pts_centroid_mean_y /= (float)cluster_size_centroid_mean_y;
            mean_displacement_vector_centroid_mean_x /= (float) cluster_size_centroid_mean_x;
            mean_displacement_vector_centroid_mean_y /= (float) cluster_size_centroid_mean_y;

            mean_pts_threshold_mean_x /= (float)cluster_size_threshold_mean_x;
            mean_pts_threshold_mean_y /= (float)cluster_size_threshold_mean_y;
            mean_displacement_vector_threshold_mean_x /= (float) cluster_size_threshold_mean_x;
            mean_displacement_vector_threshold_mean_y /= (float) cluster_size_threshold_mean_y;

            mean_pts_voted_mean_x /= (float)cluster_size_voted_mean_x;
            mean_pts_voted_mean_y /= (float)cluster_size_voted_mean_y;
            mean_displacement_vector_voted_mean_x /= (float) cluster_size_voted_mean_x;
            mean_displacement_vector_voted_mean_y /= (float) cluster_size_voted_mean_y;

            mean_pts_ranked_mean_x /= (float)cluster_size_ranked_mean_x;
            mean_pts_ranked_mean_y /= (float)cluster_size_ranked_mean_y;
            mean_displacement_vector_ranked_mean_x /= (float) cluster_size_ranked_mean_x;
            mean_displacement_vector_ranked_mean_y /= (float) cluster_size_ranked_mean_y;

            if ( frame_count > 0 ) {
                assert(std::abs(mean_displacement_vector_centroid_mean_x )>0 || std::abs(mean_displacement_vector_centroid_mean_y )>0);
            }

            multiframe_flowvector_centroid_mean.push_back(std::make_pair(cv::Point2f(mean_pts_centroid_mean_x, mean_pts_centroid_mean_y), cv::Point2f
                    (mean_displacement_vector_centroid_mean_x, mean_displacement_vector_centroid_mean_y)));

            multiframe_flowvector_threshold_mean.push_back(std::make_pair(cv::Point2f(mean_pts_threshold_mean_x, mean_pts_threshold_mean_y), cv::Point2f
                    (mean_displacement_vector_threshold_mean_x, mean_displacement_vector_threshold_mean_y)));

            multiframe_flowvector_voted_mean.push_back(std::make_pair(cv::Point2f(mean_pts_voted_mean_x, mean_pts_voted_mean_y), cv::Point2f
                    (mean_displacement_vector_voted_mean_x, mean_displacement_vector_voted_mean_y)));

            multiframe_flowvector_ranked_mean.push_back(std::make_pair(cv::Point2f(mean_pts_ranked_mean_x, mean_pts_ranked_mean_y), cv::Point2f
                    (mean_displacement_vector_ranked_mean_x, mean_displacement_vector_ranked_mean_y)));


            outer_multiframe_shape_parameters_centroid_mean.push_back(multiframe_shape_parameters_centroid_mean);
            outer_multiframe_shape_parameters_threshold_mean.push_back(multiframe_shape_parameters_threshold_mean);
            outer_multiframe_shape_parameters_voted_mean.push_back(multiframe_shape_parameters_voted_mean);
            outer_multiframe_shape_parameters_ranked_mean.push_back(multiframe_shape_parameters_ranked_mean);
            multiframe_visibility.push_back(mean_visibility);

        }

        if ( post_processing_algorithm == "ground_truth") {
            outer_multiframe_mean_pixel_centroid_pixel_displacement.push_back(multiframe_flowvector_centroid_mean);
            outer_multiframe_shape_parameters.push_back(outer_multiframe_shape_parameters_centroid_mean);
        }
        else {
            outer_multiframe_mean_pixel_centroid_pixel_displacement.push_back(multiframe_flowvector_centroid_mean);
            outer_multiframe_mean_pixel_centroid_pixel_displacement.push_back(multiframe_flowvector_threshold_mean);
            outer_multiframe_mean_pixel_centroid_pixel_displacement.push_back(multiframe_flowvector_voted_mean);
            outer_multiframe_mean_pixel_centroid_pixel_displacement.push_back(multiframe_flowvector_ranked_mean);

            outer_multiframe_shape_parameters.push_back(outer_multiframe_shape_parameters_centroid_mean);
            outer_multiframe_shape_parameters.push_back(outer_multiframe_shape_parameters_threshold_mean);
            outer_multiframe_shape_parameters.push_back(outer_multiframe_shape_parameters_voted_mean);
            outer_multiframe_shape_parameters.push_back(outer_multiframe_shape_parameters_ranked_mean);
        }

        m_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement.push_back(outer_multiframe_mean_pixel_centroid_pixel_displacement);

        m_list_obj_extrapolated_shape_parameters.push_back(outer_multiframe_shape_parameters);

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

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > list_obj_line_parameters;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_line_parameters;

    for (unsigned frame_skip = 1; frame_skip < max_skips; frame_skip++) {

        std::cout << "generate_obj_line_parameters for frame_skip " << frame_skip << " for object name " << m_objectName << " " << std::endl;


        for ( unsigned i = 0; i < COUNT; i++ ) {

            std::vector<std::pair<cv::Point2f, cv::Point2f> > frame_line_parameters;

            const unsigned long FRAME_COUNT =
                    m_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement.at(frame_skip - 1).at
                            (i).size();

            for (unsigned frame_count = 1; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement

                if (m_obj_extrapolated_mean_visibility.at(frame_skip - 1).at(frame_count) == true) {
                    cv::Point2f next_pts = m_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement.at(frame_skip-1).at
                            (i).at(frame_count).first;
                    cv::Point2f displacement_vector =
                            m_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement.at(frame_skip-1).at
                                    (i).at(frame_count).second;

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
                    frame_line_parameters.push_back(std::make_pair(cv::Point2f(m, c), pt2));
                } else {
                    // we want to maintain the size of the array.
                    frame_line_parameters.push_back(std::make_pair(cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f)));
                }
            }
            outer_line_parameters.push_back(frame_line_parameters);
        }
        list_obj_line_parameters.push_back(outer_line_parameters);
    }
    m_list_obj_line_parameters = list_obj_line_parameters;
    std::cout << "line done" << std::endl;
}

