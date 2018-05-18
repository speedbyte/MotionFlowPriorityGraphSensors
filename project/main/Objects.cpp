//
// Created by veikas on 08.02.18.
//

#include <iostream>
#include <map>
#include "Objects.h"
#include "Utils.h"
#include <chrono>

using namespace std::chrono;


void Objects::generate_object_stencil_point_displacement_pixel_visibility( std::string post_processing_algorithm, std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_base_movement_external,  std::vector<std::vector<bool> > outer_base_visibility_external ) {

// object image_data_and_shape

    if ( post_processing_algorithm == "ground_truth") {

        for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

            std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_base_movement;
            std::vector<std::vector<bool> > outer_base_visiblity;

            std::cout << "generate_object_stencil_point_displacement_pixel_visibility for sensor_index "
                      << sensor_index << std::endl;
            unsigned long FRAME_COUNT = m_object_pixel_position_pixel_displacement.at(sensor_index).size();
            assert(FRAME_COUNT > 0);
            for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement

                cv::Point2f gt_roi_pts = cv::Point2f(
                        m_object_all.at(sensor_index).at(frame_count).m_region_of_interest_px.x,
                        m_object_all.at(sensor_index).at(frame_count).m_region_of_interest_px.y);

                cv::Point2f gt_displacement = m_object_pixel_position_pixel_displacement.at(sensor_index).at(
                        frame_count).second;

                bool visibility = m_object_visibility.at(sensor_index).at(frame_count);

                std::vector<std::pair<cv::Point2f, cv::Point2f> > base_movement;
                std::vector<bool> base_visibility;

                int ObjectWidth = cvRound(
                        m_object_all.at(sensor_index).at(frame_count).m_object_dimensions_px.dim_width_m);
                int ObjectHeight = cvRound(
                        m_object_all.at(sensor_index).at(frame_count).m_object_dimensions_px.dim_height_m);

                for (unsigned j = 0; j < ObjectWidth; j += 1) {
                    for (unsigned k = 0; k < ObjectHeight; k += 1) {
                        if (j % STENCIL_GRID_COMPRESSOR == 0 &&
                            k % STENCIL_GRID_COMPRESSOR == 0) { // only entertain multiple of x pixels to reduce data
                            base_movement.push_back(std::make_pair(cv::Point2f(gt_roi_pts.x + j, gt_roi_pts.y +
                                                                                                 k), gt_displacement));
                            base_visibility.push_back(visibility);
                        }
                    }
                }
                if (!visibility) {
                    base_movement.push_back(std::make_pair(cv::Point2f(0, 0), cv::Point2f(0, 0)));
                    base_visibility.push_back(visibility);
                }
                outer_base_movement.push_back(base_movement);
                outer_base_visiblity.push_back(base_visibility);
            }

            m_object_stencil_point_displacement.push_back(outer_base_movement);
            m_object_stencil_visibility.push_back(outer_base_visiblity);
        }
    }
    else {
        m_object_stencil_point_displacement.push_back(outer_base_movement_external);
        m_object_stencil_visibility.push_back(outer_base_visibility_external);
    }


}


void Objects::generate_edge_contour(std::string post_processing_algorithm) {

    for ( int sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++ ) {

        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_edge_movement;

        char sensor_index_folder_suffix[50];
        char file_name_input_image[50];

        std::cout << "edge counter results will be stored in " << post_processing_algorithm << std::endl;

        sprintf(sensor_index_folder_suffix, "%02d", sensor_index);

        std::string temp_result_flow_path;

        std::cout << "creating edge files for sensor_index " << sensor_index << std::endl;
        std::vector<std::pair<cv::Point2f, cv::Point2f> > next_pts_array;

        for (ushort frame_count=0; frame_count < MAX_ITERATION_RESULTS; frame_count++) {
            //draw new ground truth flow.


            cv::Mat objectEdgeFrame( Dataset::getFrameSize(), CV_8UC1 );
            objectEdgeFrame = cv::Scalar_<char>(0);

            sprintf(file_name_input_image, "000%03d_10.png", frame_count);

            temp_result_flow_path = Dataset::getGroundTruthPath().string() + "ground_truth/edge_" + sensor_index_folder_suffix + "/" + file_name_input_image;

            cv::Mat edge_02_frame = cv::imread(temp_result_flow_path, CV_LOAD_IMAGE_COLOR);
            if ( edge_02_frame.data == NULL ) {
                std::cerr << temp_result_flow_path << " not found" << std::endl;
                throw ("No image file found error");
            }

            cv::Mat edge_02_frame_gray;
            cv::cvtColor(edge_02_frame, edge_02_frame_gray, CV_RGB2GRAY);

            // Calculate optical generate_flow_frame map using LK algorithm
            std::cout << "frame_count " << frame_count << std::endl;

            if ( frame_count > 0 ) {

                    objectEdgeFrame = cv::Scalar_<char>(0);
                    bool visibility = m_object_visibility.at(sensor_index).at(frame_count);
                    if ( visibility ) {

                        // This is for the base model
                        std::vector<std::pair<cv::Point2f, cv::Point2f> >  edge_movement;

                        next_pts_array = m_object_stencil_point_displacement.at(sensor_index).at(frame_count);

                        //std::cout << roi_offset.x + col_index << std::endl;
                        auto COUNT = next_pts_array.size();
                        std::cout << "making a edge contour on the basis of " << post_processing_algorithm << " " << m_objectId << std::endl;
                        std::cout << "base count " << COUNT << std::endl;
                        for ( ushort next_pts_index = 0; next_pts_index < COUNT; next_pts_index++ ) {

                            //printf("gray %u\n", edge_02_frame_gray.at<char>(cvRound(next_pts_array.at(next_pts_index).first.y), cvRound(next_pts_array.at(next_pts_index).first.x)));
                            if ( edge_02_frame_gray.at<char>(cvRound(next_pts_array.at(next_pts_index).first.y), cvRound(next_pts_array.at(next_pts_index).first.x)) != 0 ) {

                                objectEdgeFrame.at<char>(cvRound(next_pts_array.at(next_pts_index).first.y), cvRound(next_pts_array.at(next_pts_index).first.x)) = 255;
                                edge_movement.push_back(
                                        std::make_pair(next_pts_array.at(next_pts_index).first,
                                                       next_pts_array.at(next_pts_index).second));
                                //std::cout << "jayy " << next_pts_array.at(next_pts_index).first.x << " " << next_pts_array.at(next_pts_index).first.y << std::endl;

                            }
                            else {
                                //std::cout << "nopes " << next_pts_array.at(next_pts_index).first.x << " " << next_pts_array.at(next_pts_index).first.y << std::endl;
                            }
                        }

                        //cv::namedWindow("edge", CV_WINDOW_AUTOSIZE);
                        //cv::imshow("edge", objectEdgeFrame);
                        //cv::waitKey(0);
                        cv::destroyAllWindows();


                        auto edge_points = edge_movement.size();
                        std::cout << edge_points << std::endl;
                        //assert(edge_points != 0);

                        outer_edge_movement.push_back(edge_movement);

                    }
                    else {
                            outer_edge_movement.push_back(
                                    {{std::make_pair(cv::Point2f(0, 0), cv::Point2f(0, 0))}});
                    }
            }
            else {
                    outer_edge_movement.push_back(
                            {{std::make_pair(cv::Point2f(0, 0), cv::Point2f(0, 0))}});
            }
        }

        m_object_edge_point_displacement.push_back(outer_edge_movement);
    }
}


void Objects::generate_object_mean_centroid_displacement(const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &obj__blob_point_displacement, const std::vector<std::vector<std::vector<bool> > > &obj__blob_visibility, const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &obj__edge_point_displacement, std::string post_processing_algorithm) {

    // A blob can be either a stencil or a shape

    generate_edge_contour(post_processing_algorithm);

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            list_object_mean_centroid_displacement;

    std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > >
            list_object_shapepoints_displacement;


    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            outer_multiframe_flowvector_simple_avg_mean,
            outer_multiframe_flowvector_moving_avg_mean,
            outer_multiframe_flowvector_voted_mean,
            outer_multiframe_flowvector_ranked_mean,

            outer_multiframe_flowvector_sensor_fusion_mean;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            outer_multiframe_shapepoints_displacement_simple_avg_mean,
            outer_multiframe_shapepoints_displacement_moving_avg_mean,
            outer_multiframe_shapepoints_displacement_voted_mean,
            outer_multiframe_shapepoints_displacement_ranked_mean,

            outer_multiframe_shapepoints_displacement_sensor_fusion_mean;



    for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

        std::vector<std::pair<cv::Point2f, cv::Point2f> >
                multiframe_flowvector_simple_avg_mean,
                multiframe_flowvector_moving_avg_mean,
                multiframe_flowvector_voted_mean,
                multiframe_flowvector_ranked_mean;

        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
                multiframe_shapepoints_displacement_simple_avg_mean,
                multiframe_shapepoints_displacement_moving_avg_mean,
                multiframe_shapepoints_displacement_voted_mean,
                multiframe_shapepoints_displacement_ranked_mean;

        std::vector<bool> multiframe_visibility;

        std::cout << "generate_object_mean_centroid_displacement for sensor_index "
                << sensor_index
                << " for object name " << m_objectName << " " << std::endl;

        unsigned long FRAME_COUNT = obj__blob_point_displacement.at(sensor_index)
                .size();

        for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
            // gt_displacement

            std::vector<std::pair<cv::Point2f, cv::Point2f> >
                    frame_shapepoints_displacement_simple_avg_mean,
                    frame_shapepoints_displacement_moving_avg_mean,
                    frame_shapepoints_displacement_voted_mean,
                    frame_shapepoints_displacement_ranked_mean;

            std::cout << "frame_count " << frame_count << std::endl;

            float mean_pts_simple_avg_mean_x = 0.0f;
            float mean_pts_simple_avg_mean_y = 0.0f;
            float mean_displacement_vector_simple_avg_mean_x = 0.0f;
            float mean_displacement_vector_simple_avg_mean_y = 0.0f;

            float mean_pts_moving_avg_mean_x = 0.0f;
            float mean_pts_moving_avg_mean_y = 0.0f;
            float mean_displacement_vector_moving_avg_mean_x = 0.0f;
            float mean_displacement_vector_moving_avg_mean_y = 0.0f;

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

            bool visibility = obj__blob_visibility.at(sensor_index).at(frame_count).at(0);

            const unsigned CLUSTER_SIZE = (unsigned) obj__blob_point_displacement.at
                    (sensor_index).at(frame_count).size();


            if (visibility) {

                assert(CLUSTER_SIZE > 0);

                float max_voted_x;
                float max_voted_y;

                cv::Mat_<cv::Vec4f> samples(1, CLUSTER_SIZE, CV_32FC4);

                for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                    cv::Point2f pts = obj__blob_point_displacement.at(sensor_index)
                            .at(frame_count).at(cluster_index).first;
                    cv::Point2f gt_displacement = obj__blob_point_displacement.at(
                                    sensor_index)
                            .at(frame_count).at(cluster_index).second;

                    // preprocessing 1st method
                    // Nothing to do

                    // preprocesing 2nd method
                    // Nothing to do

                    // 3rd method
                    data_x.push_back(std::round(gt_displacement.x * 1000) / 1000);
                    data_y.push_back(std::round(gt_displacement.y * 1000) / 1000);

                    // preprocesing 4th method
                    data_x_pts.push_back(pts.x);
                    data_y_pts.push_back(pts.y);

                    mean_visibility = visibility;

                    samples.at<cv::Vec4f>(0, cluster_index) = {pts.x, pts.y, gt_displacement.x, gt_displacement.y};

                }

                cv::Mat_<float> covar_new, mean_new, corr;
                cv::Scalar mean;
                cv::Scalar stddev;

                cv::meanStdDev(samples, mean, stddev);

                mean_pts_simple_avg_mean_x = mean(0);
                mean_pts_simple_avg_mean_y = mean(1);
                mean_displacement_vector_simple_avg_mean_x = mean(2);
                mean_displacement_vector_simple_avg_mean_y = mean(3);

                //cv::calcCovarMatrix(samples, covar, mean, cv::COVAR_NORMAL | cv::COVAR_COLS | cv::COVAR_SCALE, CV_32FC1 );

                std::cout << "\nMean\n" << mean << "\nCovar\n" << covar_new <<
                        "\nstddev_x\n" << stddev <<
                        "\ncorr\n" << corr << std::endl;


                std::vector<cv::Mat> histogram_x, histogram_y;
                if (frame_count > 0) {
                    //Utils::drawHistogramLegacy(data_x, max_voted_x);
                    Utils::drawHistogram(data_x, histogram_x, false);
                    max_voted_x = Utils::getHistogramPeak(histogram_x);
                    Utils::drawHistogram(data_y, histogram_y, false);
                    max_voted_y = Utils::getHistogramPeak(histogram_y);
                } else {
                    max_voted_x = 0;
                    max_voted_y = 0;
                }

                std::cout << "max_voted_x " << max_voted_x << " max_voted_y " << max_voted_y << std::endl;

                unsigned
                        cluster_size_moving_avg_mean_x = 0,
                        cluster_size_moving_avg_mean_y = 0,
                        cluster_size_voted_mean_x = 0,
                        cluster_size_voted_mean_y = 0,
                        cluster_size_ranked_mean_x = 0,
                        cluster_size_ranked_mean_y = 0;


                for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {
                    cv::Point2f pts = obj__blob_point_displacement.at(sensor_index)
                            .at(frame_count).at(cluster_index).first;
                    cv::Point2f gt_displacement = obj__blob_point_displacement.at(
                                    sensor_index)
                            .at(frame_count).at(cluster_index).second;

                    // 1. MEAN - add all displacement and points and divide by total size.
                    // 2. VOTED MEAN - create vote of the displacement and points and only accept values that have the highest number of occurence
                    // 3. RANKED MEAN - create ranks of the displacment and points by accepting only those values that are on the boundary and edges and ignoring the rest.
                    // 1 st method

                    frame_shapepoints_displacement_simple_avg_mean.push_back(
                            std::make_pair(cv::Point2f(pts.x, pts.y), cv::Point2f
                                    (gt_displacement.x, gt_displacement.y)));

                    mean_pts_moving_avg_mean_x += pts.x;
                    mean_pts_moving_avg_mean_y += pts.y;
                    mean_displacement_vector_moving_avg_mean_x += gt_displacement.x;
                    mean_displacement_vector_moving_avg_mean_y += gt_displacement.y;

                    if (cluster_index > 0) {
                        mean_displacement_vector_moving_avg_mean_x /= 2;
                        mean_displacement_vector_moving_avg_mean_y /= 2;
                        frame_shapepoints_displacement_moving_avg_mean.push_back(
                                std::make_pair(cv::Point2f(pts.x, pts.y), cv::Point2f
                                        // I will anyways overwrite the whole thing later.. So this is not required, but still keeping for future references.
                                        (mean_displacement_vector_moving_avg_mean_x,
                                                mean_displacement_vector_moving_avg_mean_y)));
                    } else {
                        frame_shapepoints_displacement_moving_avg_mean.push_back(
                                std::make_pair(cv::Point2f(pts.x, pts.y), cv::Point2f
                                        (gt_displacement.x, gt_displacement.y)));

                    }
                    cluster_size_moving_avg_mean_x++;
                    cluster_size_moving_avg_mean_y++;

                    // 2nd method

                    // 3rd method
                    mean_pts_voted_mean_x += pts.x;
                    mean_pts_voted_mean_y += pts.y;

                    mean_displacement_vector_voted_mean_x += max_voted_x;
                    cluster_size_voted_mean_x += 1;

                    mean_displacement_vector_voted_mean_y += max_voted_y;
                    cluster_size_voted_mean_y += 1;

                    // 4th method

                    mean_visibility = visibility;
                }

                if (post_processing_algorithm != "ground_truth") {

                    const unsigned CLUSTER_EDGE_SIZE = (unsigned) obj__edge_point_displacement.at
                            (sensor_index).at(frame_count).size();

                    ushort WEIGHT = 100;

                    for (unsigned cluster_edge_index = 0;
                            cluster_edge_index < CLUSTER_EDGE_SIZE; cluster_edge_index++) {

                        cv::Point2f pts_edge = obj__edge_point_displacement.at(
                                        sensor_index)
                                .at(frame_count).at(cluster_edge_index).first;
                        cv::Point2f gt_displacement = obj__edge_point_displacement.at(
                                        sensor_index)
                                .at(frame_count).at(cluster_edge_index).second;

                        mean_pts_ranked_mean_x += WEIGHT * pts_edge.x;
                        mean_pts_ranked_mean_y += WEIGHT * pts_edge.y;

                        mean_displacement_vector_ranked_mean_x += WEIGHT * gt_displacement.x;
                        mean_displacement_vector_ranked_mean_y += WEIGHT * gt_displacement.y;
                        cluster_size_ranked_mean_x += WEIGHT;
                        cluster_size_ranked_mean_y += WEIGHT;

                    }
                }


                for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                    cv::Point2f pts = obj__blob_point_displacement.at(sensor_index)
                            .at(frame_count).at(cluster_index).first;
                    cv::Point2f gt_displacement = obj__blob_point_displacement.at(
                                    sensor_index)
                            .at(frame_count).at(cluster_index).second;

                    mean_pts_ranked_mean_x += pts.x;
                    mean_pts_ranked_mean_y += pts.y;
                    mean_displacement_vector_ranked_mean_x += gt_displacement.x;
                    mean_displacement_vector_ranked_mean_y += gt_displacement.y;
                    cluster_size_ranked_mean_x++;
                    cluster_size_ranked_mean_y++;

                    frame_shapepoints_displacement_ranked_mean.push_back(
                            std::make_pair(cv::Point2f(pts.x, pts.y), cv::Point2f(0, 0)));

                    frame_shapepoints_displacement_voted_mean.push_back(
                            std::make_pair(cv::Point2f(pts.x, pts.y), cv::Point2f(0, 0)));
                }

                if (frame_count > 0) {
                    assert(cluster_size_moving_avg_mean_x > 0);
                    assert(cluster_size_moving_avg_mean_y > 0);
                    assert(cluster_size_voted_mean_x > 0);
                    assert(cluster_size_voted_mean_y > 0);
                    assert(cluster_size_ranked_mean_x > 0);
                    assert(cluster_size_ranked_mean_y > 0);
                }

                // moving average mean is done on the fly.
                mean_pts_moving_avg_mean_x /= (float) cluster_size_moving_avg_mean_x;
                mean_pts_moving_avg_mean_y /= (float) cluster_size_moving_avg_mean_y;

                mean_pts_voted_mean_x /= (float) cluster_size_voted_mean_x;
                mean_pts_voted_mean_y /= (float) cluster_size_voted_mean_y;
                mean_displacement_vector_voted_mean_x /= (float) cluster_size_voted_mean_x;
                mean_displacement_vector_voted_mean_y /= (float) cluster_size_voted_mean_y;

                mean_pts_ranked_mean_x /= (float) cluster_size_ranked_mean_x;
                mean_pts_ranked_mean_y /= (float) cluster_size_ranked_mean_y;
                mean_displacement_vector_ranked_mean_x /= (float) cluster_size_ranked_mean_x;
                mean_displacement_vector_ranked_mean_y /= (float) cluster_size_ranked_mean_y;


                for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                    frame_shapepoints_displacement_simple_avg_mean.at(cluster_index).second = cv::Point2f(
                            mean_displacement_vector_simple_avg_mean_x, mean_displacement_vector_simple_avg_mean_y);

                    frame_shapepoints_displacement_moving_avg_mean.at(cluster_index).second = cv::Point2f(
                            mean_displacement_vector_moving_avg_mean_x, mean_displacement_vector_moving_avg_mean_y);

                    frame_shapepoints_displacement_voted_mean.at(cluster_index).second = cv::Point2f(
                            mean_displacement_vector_voted_mean_x, mean_displacement_vector_voted_mean_y);

                    frame_shapepoints_displacement_ranked_mean.at(cluster_index).second = cv::Point2f(
                            mean_displacement_vector_ranked_mean_x, mean_displacement_vector_ranked_mean_y);


                }

                std::cout << "mean_displacement_vector_simple_avg_mean "
                        << cv::Point2f(mean_displacement_vector_simple_avg_mean_x,
                                mean_displacement_vector_simple_avg_mean_y) << std::endl;

                std::cout << "mean_displacement_vector_moving_avg_mean "
                        << cv::Point2f(mean_displacement_vector_moving_avg_mean_x,
                                mean_displacement_vector_moving_avg_mean_y) << std::endl;

                std::cout << "mean_displacement_vector_voted_mean "
                        << cv::Point2f(mean_displacement_vector_voted_mean_x, mean_displacement_vector_voted_mean_y)
                        << std::endl;

                std::cout << "mean_displacement_vector_ranked_mean "
                        << cv::Point2f(mean_displacement_vector_ranked_mean_x,
                                mean_displacement_vector_ranked_mean_y) << std::endl;


                if (frame_count > 0) {
                    assert(mean_displacement_vector_moving_avg_mean_x != 0);
                    assert(mean_displacement_vector_simple_avg_mean_x != 0);
                    assert(mean_displacement_vector_ranked_mean_x != 0);
                    assert(mean_displacement_vector_voted_mean_x != 0);
                }

                multiframe_flowvector_simple_avg_mean.push_back(
                        std::make_pair(cv::Point2f(mean_pts_simple_avg_mean_x, mean_pts_simple_avg_mean_y),
                                cv::Point2f(mean_displacement_vector_simple_avg_mean_x,
                                        mean_displacement_vector_simple_avg_mean_y)));

                multiframe_flowvector_moving_avg_mean.push_back(
                        std::make_pair(cv::Point2f(mean_pts_moving_avg_mean_x, mean_pts_moving_avg_mean_y),
                                cv::Point2f(mean_displacement_vector_moving_avg_mean_x,
                                        mean_displacement_vector_moving_avg_mean_y)));

                multiframe_flowvector_voted_mean.push_back(
                        std::make_pair(cv::Point2f(mean_pts_voted_mean_x, mean_pts_voted_mean_y),
                                cv::Point2f(mean_displacement_vector_voted_mean_x,
                                        mean_displacement_vector_voted_mean_y)));

                multiframe_flowvector_ranked_mean.push_back(
                        std::make_pair(cv::Point2f(mean_pts_ranked_mean_x, mean_pts_ranked_mean_y),
                                cv::Point2f(mean_displacement_vector_ranked_mean_x,
                                        mean_displacement_vector_ranked_mean_y)));

                multiframe_shapepoints_displacement_simple_avg_mean.push_back(frame_shapepoints_displacement_simple_avg_mean);
                multiframe_shapepoints_displacement_moving_avg_mean.push_back(frame_shapepoints_displacement_moving_avg_mean);
                multiframe_shapepoints_displacement_voted_mean.push_back(frame_shapepoints_displacement_voted_mean);
                multiframe_shapepoints_displacement_ranked_mean.push_back(frame_shapepoints_displacement_ranked_mean);

                multiframe_visibility.push_back(mean_visibility);

            }

            else {

                std::cout << "not visible" << std::endl;

                multiframe_flowvector_simple_avg_mean.push_back(
                        std::make_pair(cv::Point2f(mean_pts_simple_avg_mean_x, mean_pts_simple_avg_mean_y),
                                       cv::Point2f(mean_displacement_vector_simple_avg_mean_x,
                                                   mean_displacement_vector_simple_avg_mean_y)));

                multiframe_flowvector_moving_avg_mean.push_back(
                        std::make_pair(cv::Point2f(mean_pts_moving_avg_mean_x, mean_pts_moving_avg_mean_y),
                                       cv::Point2f(mean_displacement_vector_moving_avg_mean_x,
                                                   mean_displacement_vector_moving_avg_mean_y)));

                multiframe_flowvector_voted_mean.push_back(
                        std::make_pair(cv::Point2f(mean_pts_voted_mean_x, mean_pts_voted_mean_y),
                                       cv::Point2f(mean_displacement_vector_voted_mean_x,
                                                   mean_displacement_vector_voted_mean_y)));

                multiframe_flowvector_ranked_mean.push_back(
                        std::make_pair(cv::Point2f(mean_pts_ranked_mean_x, mean_pts_ranked_mean_y),
                                       cv::Point2f(mean_displacement_vector_ranked_mean_x,
                                                   mean_displacement_vector_ranked_mean_y)));


                multiframe_shapepoints_displacement_simple_avg_mean.push_back({std::make_pair(cv::Point2f(0, 0), cv::Point2f
                        (0, 0))});
                multiframe_shapepoints_displacement_moving_avg_mean.push_back({std::make_pair(cv::Point2f(0, 0), cv::Point2f
                        (0, 0))});
                multiframe_shapepoints_displacement_voted_mean.push_back({std::make_pair(cv::Point2f(0, 0), cv::Point2f
                        (0, 0))});
                multiframe_shapepoints_displacement_ranked_mean.push_back({std::make_pair(cv::Point2f(0, 0), cv::Point2f
                        (0, 0))});


                multiframe_visibility.push_back(mean_visibility);

            }
        }

        outer_multiframe_flowvector_simple_avg_mean.push_back(multiframe_flowvector_simple_avg_mean);
        outer_multiframe_flowvector_moving_avg_mean.push_back(multiframe_flowvector_moving_avg_mean);
        outer_multiframe_flowvector_voted_mean.push_back(multiframe_flowvector_voted_mean);
        outer_multiframe_flowvector_ranked_mean.push_back(multiframe_flowvector_ranked_mean);

        outer_multiframe_shapepoints_displacement_simple_avg_mean.push_back(multiframe_shapepoints_displacement_simple_avg_mean);
        outer_multiframe_shapepoints_displacement_moving_avg_mean.push_back(multiframe_shapepoints_displacement_moving_avg_mean);
        outer_multiframe_shapepoints_displacement_voted_mean.push_back(multiframe_shapepoints_displacement_voted_mean);
        outer_multiframe_shapepoints_displacement_ranked_mean.push_back(multiframe_shapepoints_displacement_ranked_mean);

        // This needs to be transferred to array too -- TODO
        m_object_mean_visibility.push_back(multiframe_visibility);
    }

    if (post_processing_algorithm == "ground_truth") {

        list_object_mean_centroid_displacement.push_back(
                outer_multiframe_flowvector_simple_avg_mean);
        list_object_shapepoints_displacement.push_back(outer_multiframe_shapepoints_displacement_simple_avg_mean);


    } else {

        list_object_mean_centroid_displacement.push_back(
                outer_multiframe_flowvector_simple_avg_mean);
        list_object_mean_centroid_displacement.push_back(
                outer_multiframe_flowvector_voted_mean);
        list_object_mean_centroid_displacement.push_back(
                outer_multiframe_flowvector_ranked_mean);

        list_object_shapepoints_displacement.push_back(outer_multiframe_shapepoints_displacement_simple_avg_mean);
        list_object_shapepoints_displacement.push_back(outer_multiframe_shapepoints_displacement_voted_mean);
        list_object_shapepoints_displacement.push_back(outer_multiframe_shapepoints_displacement_ranked_mean);


        generate_updated_mean_from_multiple_sensors(post_processing_algorithm,
                outer_multiframe_flowvector_voted_mean,
                outer_multiframe_flowvector_sensor_fusion_mean,
                outer_multiframe_shapepoints_displacement_voted_mean,
                outer_multiframe_shapepoints_displacement_sensor_fusion_mean);

        list_object_mean_centroid_displacement.push_back(
                outer_multiframe_flowvector_sensor_fusion_mean);
        list_object_shapepoints_displacement.push_back(outer_multiframe_shapepoints_displacement_sensor_fusion_mean);

    }

    m_list_object_mean_centroid_displacement = list_object_mean_centroid_displacement;
    m_list_object_shapepoints_displacement = list_object_shapepoints_displacement;

    generate_object_mean_lineparameters(post_processing_algorithm);

}



void Objects::generate_updated_mean_from_multiple_sensors( std::string post_processing_algorithm,
        const std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > &multi_sensor_input_flow_vector,
        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > &outer_multiframe_flowvector_sensor_fusion_mean,
        const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &multi_sensor_input_shape,
        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &outer_multiframe_shapepoints_displacement_sensor_fusion_mean
        ) {

    std::vector<std::pair<cv::Point2f, cv::Point2f> >
            multiframe_flowvector_sensor_fusion_mean;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            multiframe_shapepoints_displacement_sensor_fusion_mean;

    std::cout << "generate_object_mean_centroid_displacement for sensor fusion "
            << " for object name " << m_objectName << " " << std::endl;

    unsigned long FRAME_COUNT = m_object_stencil_point_displacement.at(0)
            .size();


    for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
        // gt_displacement

        std::vector<std::pair<cv::Point2f, cv::Point2f> >
                frame_shapepoints_displacement_sensor_fusion_mean;

        std::cout << "frame_count " << frame_count << std::endl;

        float mean_pts_sensor_fusion_mean_x = 0.0f;
        float mean_pts_sensor_fusion_mean_y = 0.0f;
        float mean_displacement_vector_sensor_fusion_mean_x = 0.0f;
        float mean_displacement_vector_sensor_fusion_mean_y = 0.0f;

        bool visibility_1 = m_object_visibility.at(0).at(frame_count);
        bool visibility_2 = m_object_visibility.at(1).at(frame_count);

        if (visibility_1 || visibility_2 ) {

            unsigned cluster_size_sensor_fusion_mean_x = 0,
                    cluster_size_sensor_fusion_mean_y = 0;
            
            cv::Point2f pts = multi_sensor_input_flow_vector.at(0).at(frame_count).first;
            cv::Point2f pts_2 = multi_sensor_input_flow_vector.at(1).at(frame_count).first;

            cv::Mat_<float> covar_new, mean_new, corr;
            cv::Scalar mean;
            cv::Scalar stddev;

            cv::Mat_<float> visibility_mat(2,1);
            cv::Mat_<float> mean_mat_x(1,2), mean_mat_y(1,2);

            visibility_mat << visibility_1, visibility_2;

            mean_mat_x << multi_sensor_input_flow_vector.at(0).at(frame_count).second.x, multi_sensor_input_flow_vector.at(1).at(frame_count).second.x;
            mean_mat_y << multi_sensor_input_flow_vector.at(0).at(frame_count).second.y, multi_sensor_input_flow_vector.at(1).at(frame_count).second.y;

            mean_displacement_vector_sensor_fusion_mean_x = ((cv::Mat)( mean_mat_x * visibility_mat)).at<float>(0) / (visibility_1+visibility_2);
            mean_displacement_vector_sensor_fusion_mean_y = ((cv::Mat)( mean_mat_y * visibility_mat)).at<float>(0) / (visibility_1+visibility_2);


            //cv::calcCovarMatrix(samples, covar, mean, cv::COVAR_NORMAL | cv::COVAR_COLS | cv::COVAR_SCALE, CV_32FC1 );

            std::cout << "\nMean\n" << mean << "\nCovar\n" << covar_new <<
                    "\nstddev_x\n" << stddev <<
                    "\ncorr\n" << corr << std::endl;

            /*std::cout << "mean_displacement_gt_value" << cv::Point2f(mean_displacement_vector_moving_avg_mean_x, mean_displacement_vector_moving_avg_mean_y) << std::endl; */
             std::cout << "mean_displacement_vector_sensor_fusion_mean " << cv::Point2f(mean_displacement_vector_sensor_fusion_mean_x, mean_displacement_vector_sensor_fusion_mean_y) << std::endl;


            if (frame_count > 0) {

                assert(mean_displacement_vector_sensor_fusion_mean_x!=0);
            }

            const unsigned CLUSTER_SIZE_1 = (unsigned) multi_sensor_input_shape.at
                    (0).at(frame_count).size();


            for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE_1; cluster_index++) {

                if ( visibility_1 == 1 ) {
                    mean_pts_sensor_fusion_mean_x += pts.x;
                    cluster_size_sensor_fusion_mean_x++;
                    mean_pts_sensor_fusion_mean_y += pts.y;
                    cluster_size_sensor_fusion_mean_y++;

                    frame_shapepoints_displacement_sensor_fusion_mean.push_back(
                            std::make_pair(cv::Point2f(pts.x, pts.y), cv::Point2f
                                    (mean_displacement_vector_sensor_fusion_mean_x, mean_displacement_vector_sensor_fusion_mean_y)));
                }

                else if ( visibility_2 == 1 ) {
                    mean_pts_sensor_fusion_mean_x += pts_2.x;
                    cluster_size_sensor_fusion_mean_x++;
                    mean_pts_sensor_fusion_mean_y += pts_2.y;
                    cluster_size_sensor_fusion_mean_y++;

                    frame_shapepoints_displacement_sensor_fusion_mean.push_back(
                            std::make_pair(cv::Point2f(pts_2.x, pts_2.y), cv::Point2f
                                    (mean_displacement_vector_sensor_fusion_mean_x, mean_displacement_vector_sensor_fusion_mean_y)));
                }

            }

            mean_pts_sensor_fusion_mean_x /= cluster_size_sensor_fusion_mean_x;
            mean_pts_sensor_fusion_mean_y /= cluster_size_sensor_fusion_mean_y;


            multiframe_flowvector_sensor_fusion_mean.push_back(std::make_pair(cv::Point2f(mean_pts_sensor_fusion_mean_x, mean_pts_sensor_fusion_mean_y)
                    , cv::Point2f(mean_displacement_vector_sensor_fusion_mean_x, mean_displacement_vector_sensor_fusion_mean_y)));

            multiframe_shapepoints_displacement_sensor_fusion_mean.push_back(frame_shapepoints_displacement_sensor_fusion_mean);

        }

        else {

            std::cout << "not visible by both sensors" << std::endl;

            multiframe_flowvector_sensor_fusion_mean.push_back(std::make_pair(cv::Point2f(0, 0)
                        , cv::Point2f(mean_displacement_vector_sensor_fusion_mean_x, mean_displacement_vector_sensor_fusion_mean_y)));

            //mean_pts_simple_avg_mean_x, mean_pts_simple_avg_mean_y
            multiframe_shapepoints_displacement_sensor_fusion_mean.push_back({std::make_pair(cv::Point2f(0, 0), cv::Point2f
                    (mean_displacement_vector_sensor_fusion_mean_x,mean_displacement_vector_sensor_fusion_mean_y))});


        }
    }

    outer_multiframe_flowvector_sensor_fusion_mean.push_back(multiframe_flowvector_sensor_fusion_mean);
    outer_multiframe_flowvector_sensor_fusion_mean.push_back(multiframe_flowvector_sensor_fusion_mean);
    outer_multiframe_shapepoints_displacement_sensor_fusion_mean.push_back(multiframe_shapepoints_displacement_sensor_fusion_mean);
    outer_multiframe_shapepoints_displacement_sensor_fusion_mean.push_back(multiframe_shapepoints_displacement_sensor_fusion_mean);

}



void Objects::generate_object_mean_lineparameters( std::string post_processing_algorithm) {


    /// BEWARE !! I am in Cartesian co-ordinate system here.


    unsigned COUNT;
    if ( post_processing_algorithm == "ground_truth") {
        COUNT = 1;
    }
    else {
        COUNT = DATAFILTER_COUNT;
    }

    std::vector<std::vector<std::vector<cv::Point2f > > > list_object_line_parameters;


    for ( unsigned datafilter_index = 0; datafilter_index < COUNT; datafilter_index++ ) {

        std::vector<std::vector<cv::Point2f > > outer_line_parameters;

        for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

            std::cout << "generate_object_mean_lineparameters for sensor_index " << sensor_index << " for datafilter_index " << datafilter_index << " for object name " << m_objectName << " " << std::endl;

            std::vector<cv::Point2f > frame_line_parameters;

            const unsigned long FRAME_COUNT =
                    m_list_object_mean_centroid_displacement.at(datafilter_index).at
                            (sensor_index).size();

            for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
// gt_displacement

                if (m_object_mean_visibility.at(sensor_index).at(frame_count) == true) {

                    if ( frame_count > 0 ) {
                        cv::Point2f next_pts = m_list_object_mean_centroid_displacement.at(datafilter_index).at
                                (sensor_index).at(frame_count).first;
                        cv::Point2f mean_displacement_vector =
                                m_list_object_mean_centroid_displacement.at(datafilter_index).at
                                        (sensor_index).at(frame_count).second;

                        float m, c;
                        //Invert Y co-ordinates to match Cartesian co-ordinate system. This is just for the line angle and
                        //collision points. This is because, the camera origin are upper left and Cartesian is lower left
                        mean_displacement_vector.y = -mean_displacement_vector.y;
                        next_pts.y = -next_pts.y;
                        m = mean_displacement_vector.y / mean_displacement_vector.x;
                        c = next_pts.y - m * next_pts.x;  // c = y - mx
                        
                        assert(c!=0);

                        if ((int) c == 0) {
                            c += 0.001;
                        }

                        //float d = (float) sqrt((double) mean_displacement_vector.x * mean_displacement_vector.x +
                        //                       (double) mean_displacement_vector.y * mean_displacement_vector.y);
                        //mean_displacement_vector.x /= d; // normalized vector in x
                        //mean_displacement_vector.y /= d; // normalized vector in y

                        cv::Point2f pt2;

                        assert(std::isinf(m) == 0);

                        if (std::isinf(m)) {
                            if (mean_displacement_vector.y > 0.0f) {  // going up
                                pt2.x = next_pts.x;
                                pt2.y = Dataset::getFrameSize().height;
                            } else {  // going down
                                pt2.x = next_pts.x;
                                pt2.y = 0;
                            }
                        } else if (m == 0) {
                            //std::cout << frame_count << " " << next_pts<<  " " << m << " " << mean_displacement_vector << std::endl;
                            if (std::signbit(m)) { //  going left
                                pt2.x = 0;
                                pt2.y = next_pts.y;
                            } else {  // going right
                                pt2.x = Dataset::getFrameSize().width;
                                pt2.y = next_pts.y;
                            }
                        }

                        if (mean_displacement_vector.y > 0.0f) {
                            pt2.x = (Dataset::getFrameSize().height - c) / m; //
                            pt2.y = Dataset::getFrameSize().height;
                        } else if (mean_displacement_vector.y < 0.0f) {
                            pt2.x = (-c / m); //
                            pt2.y = 0;
                        }

                        frame_line_parameters.push_back(cv::Point2f(m, c));

                    } else {
                        frame_line_parameters.push_back(cv::Point2f(0.0f, 0.0f));
                    }

                }
                else {
                    frame_line_parameters.push_back(cv::Point2f(0.0f, 0.0f));
                }

            }

            //std::cout << frame_line_parameters << std::endl;

            outer_line_parameters.push_back(frame_line_parameters);
        }
        list_object_line_parameters.push_back(outer_line_parameters);
    }
    m_list_object_line_parameters = list_object_line_parameters;
    assert(m_list_object_line_parameters.at(0).at(0).size() == m_object_stencil_point_displacement.at(0).size());
    std::cout << "line done" << std::endl;
}

