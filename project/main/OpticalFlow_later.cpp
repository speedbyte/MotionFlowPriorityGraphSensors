//
// Created by veikas on 19.05.18.
//

#include <map>
#include <chrono>
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>
#include "OpticalFlow.h"
#include "FlowImageExtended.h"


void OpticalFlow::generate_shape_points_sensor_fusion(const ushort &datafilter_index, std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > >  &sensor_shape_points) {

    std::vector<Objects*> ptr_list_of_current_objects;
    std::vector<Objects *> ptr_list_of_copied_gt_objects;
    std::vector<Objects *> ptr_list_of_copied_simulated_objects;
    for ( auto i = 0; i < m_ptr_list_gt_objects.size(); i++) {
        ptr_list_of_copied_gt_objects.push_back(static_cast<Objects*>(m_ptr_list_gt_objects.at(i)));
    }

    if ( m_opticalFlowName == "ground_truth") {
        ptr_list_of_current_objects = ptr_list_of_copied_gt_objects;
    }
    else {
        for ( auto i = 0; i < m_ptr_list_simulated_objects.size(); i++) {
            ptr_list_of_copied_simulated_objects.push_back(static_cast<Objects*>(m_ptr_list_simulated_objects.at(i)));
        }
        ptr_list_of_current_objects = ptr_list_of_copied_simulated_objects;
    }

    std::vector<std::map<std::pair<float, float>, int> > sensor_scenario_displacement_occurence;
    std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > sensor_frame_shape_points;
    std::map<std::pair<float, float>, int> scenario_displacement_occurence;

    unsigned FRAME_COUNT = (unsigned) ptr_list_of_current_objects.at(0)
            ->get_list_object_dataprocessing_stencil_points_displacement().at(datafilter_index).at(0).size();

    std::cout << "generating shape points in OpticalFlow.cpp for sensor fusion" << m_resultordner << " for datafilter " << datafilter_index << std::endl;

    for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

        assert(FRAME_COUNT > 0);

        std::vector<std::pair<cv::Point2i, cv::Point2f>> frame_shape_points;
        std::cout << "current_frame_index " << current_frame_index << " for datafilter_index " << datafilter_index<< std::endl;

        for (unsigned sensor_index = 0; sensor_index <= 0 ; sensor_index++) {

            for (ushort obj_index = 0; obj_index < ptr_list_of_current_objects.size(); obj_index++) {

                auto CLUSTER_COUNT_GT = m_ptr_list_gt_objects.at(
                        obj_index)->get_list_object_dataprocessing_stencil_points_displacement().at(0).at(0).at(current_frame_index).size();

                auto CLUSTER_COUNT_GT_2 = m_ptr_list_gt_objects.at(
                        obj_index)->get_list_object_dataprocessing_stencil_points_displacement().at(0).at(1).at(current_frame_index).size();

//CLUSTER_COUNT_GT =  ( CLUSTER_COUNT_GT + CLUSTER_COUNT_GT_2 ) /2;
                if (ptr_list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(0).at(current_frame_index) ||
                    ptr_list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(1).at(current_frame_index)) {

// Instances of CLUSTER_COUNT_ALGO in CLUSTER_COUNT_GT

                    float vollTreffer = 0;
                    float baseTreffer;

                    cv::Point2f gt_displacement = m_ptr_list_gt_objects.at(obj_index)->get_object_extrapolated_point_displacement().at
                            (0).at(current_frame_index).second;

                    cv::Point2f gt_displacement_2 = m_ptr_list_gt_objects.at(obj_index)->get_object_extrapolated_point_displacement().at
                            (1).at(current_frame_index).second;

//gt_displacement = (gt_displacement + gt_displacement_2)/2;

                    auto dist_gt = cv::norm(gt_displacement);
                    auto angle_gt = std::tanh(gt_displacement.y / gt_displacement.x);

                    auto dist_gt_2 = cv::norm(gt_displacement_2);
                    auto angle_gt_2 = std::tanh(gt_displacement_2.y / gt_displacement_2.x);


                    if (m_opticalFlowName == "ground_truth") {

                        vollTreffer = CLUSTER_COUNT_GT;
// this is the full resolution ! Because there is no stepSize in GroundTruth
                        baseTreffer = CLUSTER_COUNT_GT;

                    }
                    else {

                        if (ptr_list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(0).at(current_frame_index)) {

                            auto CLUSTER_COUNT_ALGO = ptr_list_of_current_objects.at(
                                    obj_index)->get_list_object_dataprocessing_stencil_points_displacement().at(datafilter_index).at(0).at(current_frame_index).size();

                            for (auto cluster_index = 0; cluster_index < CLUSTER_COUNT_ALGO; cluster_index++) {

                                cv::Point2f algo_displacement = ptr_list_of_current_objects.at(obj_index)->
                                        get_list_object_dataprocessing_stencil_points_displacement().at(datafilter_index
                                ).at(0).at(current_frame_index).at(cluster_index).second;

                                auto dist_algo = cv::norm(algo_displacement);
                                auto dist_err = std::abs(dist_gt - dist_algo);

                                auto angle_algo = std::tanh(algo_displacement.y / algo_displacement.x);

                                auto angle_err = std::abs(angle_algo - angle_gt);
                                auto angle_err_dot = std::cosh(
                                        algo_displacement.dot(gt_displacement) / (dist_gt * dist_algo));

//assert(angle_err_dot==angle_err);

                                if (
                                        //(dist_err) < DISTANCE_ERROR_TOLERANCE &&
                                        (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE

                                        ) {
                                    vollTreffer++;
                                }

                            }

                            baseTreffer = ((float) CLUSTER_COUNT_GT) / mStepSize;
                        }

                        else if (ptr_list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(1).at(current_frame_index)) {

                            auto CLUSTER_COUNT_ALGO_2 = ptr_list_of_current_objects.at(
                                    obj_index)->get_list_object_dataprocessing_stencil_points_displacement().at(datafilter_index).at(1).at(current_frame_index).size();

                            for (auto cluster_index = 0; cluster_index < CLUSTER_COUNT_ALGO_2; cluster_index++) {

                                cv::Point2f algo_displacement = ptr_list_of_current_objects.at(obj_index)->
                                        get_list_object_dataprocessing_stencil_points_displacement().at(datafilter_index
                                ).at(1).at(current_frame_index).at(cluster_index).second;

                                auto dist_algo = cv::norm(algo_displacement);
                                auto dist_err = std::abs(dist_gt - dist_algo);

                                auto angle_algo = std::tanh(algo_displacement.y / algo_displacement.x);

                                auto angle_err = std::abs(angle_algo - angle_gt_2);
                                auto angle_err_dot = std::cosh(
                                        algo_displacement.dot(gt_displacement_2) / (dist_gt_2 * dist_algo));

//assert(angle_err_dot==angle_err);

                                if (
                                        //(dist_err) < DISTANCE_ERROR_TOLERANCE &&
                                        (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE

                                        ) {
                                    vollTreffer++;
                                }

                            }

                            baseTreffer = ((float) CLUSTER_COUNT_GT_2) / mStepSize;
                        }

                    }
                    frame_shape_points.push_back(std::make_pair(cv::Point2i(current_frame_index, 0), cv::Point2f(vollTreffer, baseTreffer)));

                    std::cout << "vollTreffer for object " << ptr_list_of_current_objects.at(obj_index)->getObjectId() << " = "
                              << vollTreffer << std::endl;
                    std::cout << "baseTreffer for object " << ptr_list_of_current_objects.at(obj_index)->getObjectId() << " = "
                              << baseTreffer << std::endl;

                    assert(vollTreffer <= std::ceil(baseTreffer) + 20 );

                } else {
                    std::cout << "visibility of object " << ptr_list_of_current_objects.at(obj_index)->getObjectId() << " = " <<
                              ptr_list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(sensor_index)
                                      .at(current_frame_index)
                              << " and hence not generating any shape points for this object " << std::endl;

                    frame_shape_points.push_back(std::make_pair(cv::Point2i(current_frame_index,0), cv::Point2f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));

                }
            }
        }
        sensor_frame_shape_points.push_back(frame_shape_points);

    }
    sensor_shape_points.push_back(sensor_frame_shape_points);
    sensor_scenario_displacement_occurence.push_back(scenario_displacement_occurence);


// plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
}


/*
void Objects::generate_updated_mean_from_multiple_sensors( std::string post_processing_algorithm,
        const std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > &multi_sensor_input_flow_vector,
        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > &sensor_multiframe_centroid_displacement_sensor_fusion_mean,
        const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &multi_sensor_input_shape,
        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &sensor_multiframe_dataprocessing_stencil_points_displacement_sensor_fusion_mean
        ) {

    std::vector<std::pair<cv::Point2f, cv::Point2f> >
            multiframe_centroid_displacement_sensor_fusion_mean;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            multiframe_dataprocessing_stencil_points_displacement_sensor_fusion_mean;

    std::cout << "generate_object_mean_centroid_displacement for sensor fusion "
            << " for object name " << m_objectName << " " << std::endl;

    unsigned long FRAME_COUNT = m_object_stencil_point_displacement.at(0)
            .size();


    for (unsigned current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {
        // gt_displacement

        std::vector<std::pair<cv::Point2f, cv::Point2f> >
                frame_dataprocessing_stencil_points_displacement_sensor_fusion_mean;

        std::cout << "current_frame_index " << current_frame_index << std::endl;

        float mean_pts_sensor_fusion_mean_x = 0.0f;
        float mean_pts_sensor_fusion_mean_y = 0.0f;
        float mean_displacement_vector_sensor_fusion_mean_x = 0.0f;
        float mean_displacement_vector_sensor_fusion_mean_y = 0.0f;

        bool visibility_1 = m_object_extrapolated_visibility.at(0).at(current_frame_index);
        bool visibility_2 = m_object_extrapolated_visibility.at(1).at(current_frame_index);

        if (visibility_1 || visibility_2 ) {

            unsigned cluster_size_sensor_fusion_mean_x = 0,
                    cluster_size_sensor_fusion_mean_y = 0;

            cv::Point2f pts = multi_sensor_input_flow_vector.at(0).at(current_frame_index).first;
            cv::Point2f pts_2 = multi_sensor_input_flow_vector.at(1).at(current_frame_index).first;

            cv::Mat_<float> covar_new, mean_new, corr;
            cv::Scalar mean;
            cv::Scalar stddev;

            cv::Mat_<float> visibility_mat(2,1);
            cv::Mat_<float> mean_mat_x(1,2), mean_mat_y(1,2);

            visibility_mat << visibility_1, visibility_2;

            mean_mat_x << multi_sensor_input_flow_vector.at(0).at(current_frame_index).second.x, multi_sensor_input_flow_vector.at(1).at(current_frame_index).second.x;
            mean_mat_y << multi_sensor_input_flow_vector.at(0).at(current_frame_index).second.y, multi_sensor_input_flow_vector.at(1).at(current_frame_index).second.y;

            mean_displacement_vector_sensor_fusion_mean_x = ((cv::Mat)( mean_mat_x * visibility_mat)).at<float>(0) / (visibility_1+visibility_2);
            mean_displacement_vector_sensor_fusion_mean_y = ((cv::Mat)( mean_mat_y * visibility_mat)).at<float>(0) / (visibility_1+visibility_2);


            //cv::calcCovarMatrix(samples, covar, mean, cv::COVAR_NORMAL | cv::COVAR_COLS | cv::COVAR_SCALE, CV_32FC1 );

            std::cout << "\nMean\n" << mean << "\nCovar\n" << covar_new <<
                    "\nstddev_x\n" << stddev <<
                    "\ncorr\n" << corr << std::endl;

            //std::cout << "mean_displacement_gt_value" << cv::Point2f(mean_displacement_vector_moving_avg_mean_x, mean_displacement_vector_moving_avg_mean_y) << std::endl;
             std::cout << "mean_displacement_vector_sensor_fusion_mean " << cv::Point2f(mean_displacement_vector_sensor_fusion_mean_x, mean_displacement_vector_sensor_fusion_mean_y) << std::endl;


            if (current_frame_index > 0) {

                assert(mean_displacement_vector_sensor_fusion_mean_x!=0);
            }

            const unsigned CLUSTER_SIZE_1 = (unsigned) multi_sensor_input_shape.at
                    (0).at(current_frame_index).size();


            for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE_1; cluster_index++) {

                if ( visibility_1 == 1 ) {
                    mean_pts_sensor_fusion_mean_x += pts.x;
                    cluster_size_sensor_fusion_mean_x++;
                    mean_pts_sensor_fusion_mean_y += pts.y;
                    cluster_size_sensor_fusion_mean_y++;

                    frame_dataprocessing_stencil_points_displacement_sensor_fusion_mean.push_back(
                            std::make_pair(cv::Point2f(pts.x, pts.y), cv::Point2f
                                    (mean_displacement_vector_sensor_fusion_mean_x, mean_displacement_vector_sensor_fusion_mean_y)));
                }

                else if ( visibility_2 == 1 ) {
                    mean_pts_sensor_fusion_mean_x += pts_2.x;
                    cluster_size_sensor_fusion_mean_x++;
                    mean_pts_sensor_fusion_mean_y += pts_2.y;
                    cluster_size_sensor_fusion_mean_y++;

                    frame_dataprocessing_stencil_points_displacement_sensor_fusion_mean.push_back(
                            std::make_pair(cv::Point2f(pts_2.x, pts_2.y), cv::Point2f
                                    (mean_displacement_vector_sensor_fusion_mean_x, mean_displacement_vector_sensor_fusion_mean_y)));
                }

            }

            mean_pts_sensor_fusion_mean_x /= cluster_size_sensor_fusion_mean_x;
            mean_pts_sensor_fusion_mean_y /= cluster_size_sensor_fusion_mean_y;


            multiframe_centroid_displacement_sensor_fusion_mean.push_back(std::make_pair(cv::Point2f(mean_pts_sensor_fusion_mean_x, mean_pts_sensor_fusion_mean_y)
                    , cv::Point2f(mean_displacement_vector_sensor_fusion_mean_x, mean_displacement_vector_sensor_fusion_mean_y)));

            multiframe_dataprocessing_stencil_points_displacement_sensor_fusion_mean.push_back(frame_dataprocessing_stencil_points_displacement_sensor_fusion_mean);

        }

        else {

            std::cout << "not visible by both sensors" << std::endl;

            multiframe_centroid_displacement_sensor_fusion_mean.push_back(std::make_pair(cv::Point2f(0, 0)
                        , cv::Point2f(mean_displacement_vector_sensor_fusion_mean_x, mean_displacement_vector_sensor_fusion_mean_y)));

            //mean_pts_simple_avg_mean_x, mean_pts_simple_avg_mean_y
            multiframe_dataprocessing_stencil_points_displacement_sensor_fusion_mean.push_back({std::make_pair(cv::Point2f(0, 0), cv::Point2f
                    (mean_displacement_vector_sensor_fusion_mean_x,mean_displacement_vector_sensor_fusion_mean_y))});


        }
    }

    sensor_multiframe_centroid_displacement_sensor_fusion_mean.push_back(multiframe_centroid_displacement_sensor_fusion_mean);
    sensor_multiframe_centroid_displacement_sensor_fusion_mean.push_back(multiframe_centroid_displacement_sensor_fusion_mean);
    sensor_multiframe_dataprocessing_stencil_points_displacement_sensor_fusion_mean.push_back(multiframe_dataprocessing_stencil_points_displacement_sensor_fusion_mean);
    sensor_multiframe_dataprocessing_stencil_points_displacement_sensor_fusion_mean.push_back(multiframe_dataprocessing_stencil_points_displacement_sensor_fusion_mean);

    //multiframe_dataprocessing_stencil_points_displacement_sensor_fusion_mean.push_back(frame_dataprocessing_stencil_points_displacement_sensor_fusion_mean);

    sensor_multiframe_dataprocessing_stencil_points_displacement_sensor_fusion_mean.push_back(multiframe_dataprocessing_stencil_points_displacement_sensor_fusion_mean);


}

     generate_updated_mean_from_multiple_sensors(post_processing_algorithm,
                                                sensor_multiframe_centroid_displacement_voted_mean,
                                                sensor_multiframe_centroid_displacement_sensor_fusion_mean,
                                                m_object_stencil_point_displacement,
                                                sensor_multiframe_dataprocessing_stencil_points_displacement_sensor_fusion_mean);

    list_object_dataprocessing_stencil_points_displacement.push_back(sensor_multiframe_dataprocessing_stencil_points_displacement_sensor_fusion_mean);
    m_list_object_dataprocessing_stencil_points_displacement = list_object_dataprocessing_stencil_points_displacement;

 */