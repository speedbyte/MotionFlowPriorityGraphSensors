#include "OpticalFlow.h"
#include <gnuplot-iostream/gnuplot-iostream.h>
#include "Utils.h"
#include "SortandIntersect.h"

void OpticalFlow::generate_metrics_optical_flow_algorithm(ushort SENSOR_COUNT) {

    std::vector<Objects *> list_of_current_objects;

    const float DISTANCE_ERROR_TOLERANCE = 1;

    std::vector<Objects *> ptr_list_of_derived_objects;
    for ( auto i = 0; i < m_ptr_list_gt_objects.size(); i++) {
        ptr_list_of_derived_objects.push_back(static_cast<Objects*>(m_ptr_list_gt_objects.at(i)));
    }

    unsigned COUNT;
    if (m_opticalFlowName == "ground_truth") {
        COUNT = 1;
        list_of_current_objects = ptr_list_of_derived_objects;
    } else {
        list_of_current_objects = m_ptr_list_simulated_objects;
        COUNT = (unsigned)list_of_current_objects.at(0)->
                get_list_object_dataprocessing_mean_centroid_displacement().size();
    }

    for (unsigned datafilter_index = 0; datafilter_index < COUNT; datafilter_index++) {

        std::vector<std::map<std::pair<float, float>, int> > sensor_scenario_displacement_occurence;
        std::vector<std::vector<std::vector<OPTICAL_FLOW_EVALUATION_METRICS> > > sensor_multiframe_evaluation_data;

        for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

            std::cout << "generating evaluation metrics in OpticalFlow.cpp for sensor index " << sensor_index
                      << " for opticalflow  " << m_opticalFlowName << std::endl;

            unsigned FRAME_COUNT = (unsigned) list_of_current_objects.at(0)
                    ->get_object_stencil_point_displacement().at(sensor_index).size();

            assert(FRAME_COUNT > 0);

            std::vector<std::vector<OPTICAL_FLOW_EVALUATION_METRICS> > multiframe_evaluation_data;

            for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

                std::vector<OPTICAL_FLOW_EVALUATION_METRICS> evaluationData(list_of_current_objects.size());

                char file_name_image_output[50], file_name_image_output_stiched[50], sensor_index_folder_suffix[10], stiched_sensor_index_folder_suffix[10];
                sprintf(sensor_index_folder_suffix, "%02d", sensor_index);
                sprintf(stiched_sensor_index_folder_suffix, "%02d", (SENSOR_COUNT-1));

                std::string gnuplot_image_file_with_path;
                std::string gnuplot_image_file_with_path_stiched;

                ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                        (0).at(current_frame_index).frame_no;

                sprintf(file_name_image_output, "000%03d_10.png", image_frame_count);
                sprintf(file_name_image_output_stiched, "stiched_000%03d_10.png", image_frame_count);

                gnuplot_image_file_with_path = m_gnuplots_path.string() + sensor_index_folder_suffix + "/" + file_name_image_output;

                gnuplot_image_file_with_path_stiched = m_gnuplots_path.string() + stiched_sensor_index_folder_suffix + "/" + file_name_image_output_stiched;

                std::cout << "current_frame_index " << current_frame_index << " for opticalflow_index " << m_opticalFlowName
                          << std::endl;

                Gnuplot gp2d;
                cv::Mat stich_plots;

                for (ushort obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++) {

                    std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > > entire_roi_object = list_of_current_objects.at(obj_index)->get_list_object_dataprocessing_stencil_points_displacement();

                    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > entire_roi_object_interpolated = list_of_current_objects.at(obj_index)->get_object_interpolated_stencil_point_displacement();

                    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >  special_roi_object = m_ptr_list_gt_objects.at(obj_index)->get_object_special_region_of_interest();

                    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > gt_roi_object = m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement();

                    // displacements found by the ground truth for this object
                    auto CLUSTER_COUNT_GT = m_ptr_list_gt_objects.at(
                            obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).size();

                    unsigned CLUSTER_COUNT_GT_SPECIAL_ROI = (unsigned) special_roi_object.at(sensor_index).at(
                            current_frame_index).size();

                    /*
                    unsigned CLUSTER_COUNT_DISJOINT_SPECIAL_ROI = (unsigned) m_ptr_list_gt_objects.at(
                            obj_index)->get_object_disjoint_special_region_of_interest().at(sensor_index).at(
                            current_frame_index).size();
                            */

                    evaluationData.at(obj_index).current_frame_index = image_frame_count;
                    evaluationData.at(obj_index).obj_index = obj_index;
                    evaluationData.at(obj_index).visiblity = list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(sensor_index).at(current_frame_index);

                    if (evaluationData.at(obj_index).visiblity) {

                        // Instances of CLUSTER_COUNT_ALGO in CLUSTER_COUNT_GT

                        cv::Point2f gt_displacement = m_ptr_list_gt_objects.at(
                                obj_index)->get_object_extrapolated_point_displacement().at
                                (sensor_index).at(current_frame_index).second;

                        auto euclidean_dist_gt = cv::norm(gt_displacement);
                        auto angle_gt = std::tanh(gt_displacement.y / gt_displacement.x);

                        evaluationData.at(obj_index).mean_pts = list_of_current_objects.at(obj_index)->
                                get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index
                        ).at(sensor_index).at(current_frame_index).mean_pts;

                        evaluationData.at(obj_index).mean_displacement = list_of_current_objects.at(obj_index)->
                                get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index
                        ).at(sensor_index).at(current_frame_index).mean_displacement;

                        evaluationData.at(obj_index).gt_mean_displacement = m_ptr_list_gt_objects.at(obj_index)->
                                get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index
                        ).at(sensor_index).at(current_frame_index).mean_displacement;

                        evaluationData.at(obj_index).stddev_pts = list_of_current_objects.at(
                                obj_index)->
                                get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index
                        ).at(sensor_index).at(current_frame_index).stddev_pts;

                        evaluationData.at(obj_index).stddev_displacement = list_of_current_objects.at(
                                obj_index)->
                                get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index
                        ).at(sensor_index).at(current_frame_index).stddev_displacement;

                        evaluationData.at(obj_index).covar_pts = list_of_current_objects.at(
                                obj_index)->
                                get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index
                        ).at(sensor_index).at(current_frame_index).covar_pts;

                        evaluationData.at(obj_index).covar_displacement = list_of_current_objects.at(
                                obj_index)->
                                get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index
                        ).at(sensor_index).at(current_frame_index).covar_displacement;


                        evaluationData.at(
                                obj_index).ground_truth_pixels_count = (ushort)CLUSTER_COUNT_GT; //(dimension.x * dimension.y); // how many pixels are visible ( it could be that some pixels are occluded )

                        evaluationData.at(
                                obj_index).ground_truth_sroi_pixels_count = (ushort)CLUSTER_COUNT_GT_SPECIAL_ROI; //(dimension.x * dimension.y); // how many pixels are visible ( it could be that some pixels are occluded )

                        evaluationData.at(
                                obj_index).algorithm_pixels_count = (ushort)0; //(dimension.x * dimension.y); // how many pixels are visible ( it could be that some pixels are occluded )
                        evaluationData.at(
                                obj_index).l1_cumulative_distance_good_pixels = CLUSTER_COUNT_GT; // how many pixels in the found pixel are actually valid
                        evaluationData.at(
                                obj_index).l2_cumulative_distance_good_pixels = CLUSTER_COUNT_GT; // how many pixels in the found pixel are actually valid
                        evaluationData.at(
                                obj_index).ma_cumulative_distance_good_pixels = CLUSTER_COUNT_GT; // how many pixels in the found pixel are actually valid

                        double l1_cumulative_distance_all_pixels = 0;
                        double l2_cumulative_distance_all_pixels = 0;
                        double ma_cumulative_distance_all_pixels = 0;

                        double l1_cumulative_error_tolerated = 0;
                        double l2_cumulative_error_tolerated = 0;
                        double ma_cumulative_error_tolerated = 0;

                        double l1_cumulative_distance_all_pixels_interpolated = 0;
                        double l2_cumulative_distance_all_pixels_interpolated = 0;
                        double ma_cumulative_distance_all_pixels_interpolated = 0;

                        double l1_cumulative_error_tolerated_interpolated = 0;
                        double l2_cumulative_error_tolerated_interpolated = 0;
                        double ma_cumulative_error_tolerated_interpolated = 0;

                        double l1_cumulative_distance_sroi_pixels = 0;
                        double l2_cumulative_distance_sroi_pixels = 0;
                        double ma_cumulative_distance_sroi_pixels = 0;

                        double l1_cumulative_sroi_error_tolerated = 0;
                        double l2_cumulative_sroi_error_tolerated = 0;
                        double ma_cumulative_sroi_error_tolerated = 0;


                        cv::Mat icovar;
                        // this should be sent to Objects.cpp
                        if ( evaluationData.at(obj_index).covar_displacement.data != NULL ) {

                            icovar = evaluationData.at(obj_index).covar_displacement.inv(cv::DECOMP_SVD);
                        }

                        evaluationData.at(
                                obj_index).l1_total_count_good_pixels = (ushort)CLUSTER_COUNT_GT;
                        evaluationData.at(
                                obj_index).l2_total_count_good_pixels = (ushort)CLUSTER_COUNT_GT;
                        evaluationData.at(
                                obj_index).ma_total_count_good_pixels = (ushort)CLUSTER_COUNT_GT;

                        if (m_opticalFlowName != "ground_truth") {

                            // displacements found by the algorithm for this object
                            unsigned CLUSTER_COUNT_ALGORITHM = (unsigned) entire_roi_object.at(datafilter_index).at(sensor_index).at(
                                    current_frame_index).size();

                            unsigned CLUSTER_COUNT_INTERPOLATED_ALGORITHM = (unsigned) entire_roi_object_interpolated.at(sensor_index).at(
                                    current_frame_index).size();

                            evaluationData.at(
                                    obj_index).l1_total_count_good_pixels = (ushort) 0;
                            evaluationData.at(
                                    obj_index).l2_total_count_good_pixels = (ushort) 0;
                            evaluationData.at(
                                    obj_index).ma_total_count_good_pixels = (ushort) 0;

                            // how many pixelsi are visible ( it could be that some pixels are occluded ). This wll be found out using k-means
                            evaluationData.at(
                                    obj_index).l1_cumulative_distance_good_pixels = 0; // how many pixels in the found pixel are actually valid
                            evaluationData.at(
                                    obj_index).l2_cumulative_distance_good_pixels = 0; // how many pixels in the found pixel are actually valid
                            evaluationData.at(
                                    obj_index).ma_cumulative_distance_good_pixels = 0; // how many pixels in the found pixel are actually valid

                            std::vector<std::pair<float, float>> xy_pts;

                            // all pixels
                            for (auto cluster_index = 0; cluster_index < CLUSTER_COUNT_ALGORITHM; cluster_index++) {

                                cv::Point2f algo_displacement = entire_roi_object.at(datafilter_index
                                ).at(sensor_index).at(current_frame_index).at(cluster_index).second;

                                // l1_cumulative_distance_all_pixels
                                auto l1_dist_err = ( std::abs(algo_displacement.x - gt_displacement.x ) + std::abs(algo_displacement.y - gt_displacement.y));
                                l1_cumulative_distance_all_pixels += l1_dist_err;

                                // l2_cumulative_distance_all_pixels
                                auto euclidean_dist_algo_square = (std::pow((algo_displacement.x - gt_displacement.x),2 ) + std::pow((algo_displacement.y - gt_displacement.y),2 ));
                                auto euclidean_dist_err = std::sqrt(euclidean_dist_algo_square);
                                l2_cumulative_distance_all_pixels += euclidean_dist_err;

                                // ma_cumulative_distance_all_pixels
                                auto ma_dist_algo = Utils::getMahalanobisDistance(icovar, algo_displacement, evaluationData.at(obj_index).mean_displacement);
                                ma_cumulative_distance_all_pixels += ma_dist_algo;

                                //auto angle_algo = std::tanh(algo_displacement.y / algo_displacement.x);
                                //auto angle_err = std::abs(angle_algo - angle_gt);
                                //auto angle_err_dot = std::cosh(algo_displacement.dot(gt_displacement) / (euclidean_dist_gt * std::sqrt(euclidean_dist_algo_square)));
                                //assert(angle_err_dot==angle_err);

                                if (
                                        (l1_dist_err) < DISTANCE_ERROR_TOLERANCE
                                    //&& (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE
                                        ) {
                                    l1_cumulative_error_tolerated += l1_dist_err;
                                    evaluationData.at(
                                            obj_index).l1_total_count_good_pixels++; // how many pixels in the found pixel are actually valid
                                }
                                if (
                                        (euclidean_dist_err) < DISTANCE_ERROR_TOLERANCE
                                    //&& (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE
                                        ) {
                                    l2_cumulative_error_tolerated += euclidean_dist_err;
                                    evaluationData.at(
                                            obj_index).l2_total_count_good_pixels++; // how many pixels in the found pixel are actually valid
                                }
                                if (
                                        (ma_dist_algo) < DISTANCE_ERROR_TOLERANCE
                                    // && (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE
                                        ) {
                                    ma_cumulative_error_tolerated += ma_dist_algo;
                                    evaluationData.at(
                                            obj_index).ma_total_count_good_pixels++; // how many pixels in the found pixel are actually valid
                                }
                                
                                xy_pts.push_back(std::make_pair(algo_displacement.x, algo_displacement.y));
                            }


                            for (auto cluster_index = 0; cluster_index < CLUSTER_COUNT_INTERPOLATED_ALGORITHM; cluster_index++) {

                                cv::Point2f algo_displacement = entire_roi_object_interpolated.at(sensor_index).at(current_frame_index).at(cluster_index).second;

                                // l1_cumulative_distance_all_pixels
                                auto l1_dist_err = ( std::abs(algo_displacement.x - gt_displacement.x ) + std::abs(algo_displacement.y - gt_displacement.y));
                                l1_cumulative_distance_all_pixels_interpolated += l1_dist_err;

                                // l2_cumulative_distance_all_pixels
                                auto euclidean_dist_algo_square = (std::pow((algo_displacement.x - gt_displacement.x),2 ) + std::pow((algo_displacement.y - gt_displacement.y),2 ));
                                auto euclidean_dist_err = std::sqrt(euclidean_dist_algo_square);
                                l2_cumulative_distance_all_pixels_interpolated += euclidean_dist_err;

                                // ma_cumulative_distance_all_pixels
                                auto ma_dist_algo = Utils::getMahalanobisDistance(icovar, algo_displacement, evaluationData.at(obj_index).mean_displacement);
                                ma_cumulative_distance_all_pixels_interpolated += ma_dist_algo;

                                //auto angle_algo = std::tanh(algo_displacement.y / algo_displacement.x);
                                //auto angle_err = std::abs(angle_algo - angle_gt);
                                //auto angle_err_dot = std::cosh(algo_displacement.dot(gt_displacement) / (euclidean_dist_gt * std::sqrt(euclidean_dist_algo_square)));
                                //assert(angle_err_dot==angle_err);

                                if (
                                        (l1_dist_err) < DISTANCE_ERROR_TOLERANCE
                                    //&& (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE
                                        ) {
                                    l1_cumulative_error_tolerated_interpolated += l1_dist_err;
                                    evaluationData.at(
                                            obj_index).l1_total_count_good_pixels_interpolated++; // how many pixels in the found pixel are actually valid
                                }
                                if (
                                        (euclidean_dist_err) < DISTANCE_ERROR_TOLERANCE
                                    //&& (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE
                                        ) {
                                    l2_cumulative_error_tolerated_interpolated += euclidean_dist_err;
                                    evaluationData.at(
                                            obj_index).l2_total_count_good_pixels_interpolated++; // how many pixels in the found pixel are actually valid
                                }
                                if (
                                        (ma_dist_algo) < DISTANCE_ERROR_TOLERANCE
                                    // && (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE
                                        ) {
                                    ma_cumulative_error_tolerated_interpolated += ma_dist_algo;
                                    evaluationData.at(
                                            obj_index).ma_total_count_good_pixels_interpolated++; // how many pixels in the found pixel are actually valid
                                }

                                xy_pts.push_back(std::make_pair(algo_displacement.x, algo_displacement.y));
                            }

                            {

                                // sroi pixels
                                // does eroi contains sroi coordinates? It should have because we are expanding eroi with new interpolated values. So, what is the final value?
                                std::vector<std::pair<cv::Point2f, cv::Point2f> > intersection_of_algorithm_and_sroi;
                                std::vector<std::pair<cv::Point2f, cv::Point2f> > dummy(gt_roi_object.at(sensor_index).at(current_frame_index).size());

                                MyIntersection intersection_eroi_sroi_objects;
                                std::vector<std::pair<cv::Point2f, cv::Point2f> >::iterator result_it;

                                result_it = intersection_eroi_sroi_objects.find_intersection_pair(entire_roi_object.at(datafilter_index).at(sensor_index).at(current_frame_index).begin(), entire_roi_object.at(datafilter_index).at(sensor_index).at(current_frame_index).end(), special_roi_object.at(sensor_index).at(current_frame_index).begin(), special_roi_object.at(sensor_index).at(current_frame_index).end(),
                                                                                                  dummy.begin());
                                intersection_of_algorithm_and_sroi = intersection_eroi_sroi_objects.getResultIntersectingPair();
                                bool isSorted = std::is_sorted(entire_roi_object.at(datafilter_index).at(sensor_index).at(current_frame_index).begin(), entire_roi_object.at(datafilter_index).at(sensor_index).at(current_frame_index).end(), PairPointsSort<float>());
                                assert(isSorted);
                                bool isSorted_sroi = std::is_sorted(special_roi_object.at(sensor_index).at(current_frame_index).begin(), special_roi_object.at(sensor_index).at(current_frame_index).end(), PairPointsSort<float>());
                                assert(isSorted_sroi);

                                //assert(intersection_of_algorithm_and_sroi.size() > 0);
                                // Validate
                                cv::Mat tempImage(Dataset::m_frame_size, CV_8UC3);
                                tempImage = cv::Scalar::all(255);
                                for ( auto it = intersection_of_algorithm_and_sroi.begin(); it != intersection_of_algorithm_and_sroi.end(); it++) {
                                    cv::circle(tempImage, (*it).first, 1, cv::Scalar(255,0,0));
                                }
                                cv::imshow("algorithm_sroi", tempImage);
                                cv::waitKey(0);
                                cv::destroyAllWindows();

                            }

                            {
                                // sroi pixels
                                // does eroi_interpolated contains sroi coordinates? It should have because we are expanding eroi with new interpolated values. So, what is the final value?
                                std::vector<std::pair<cv::Point2f, cv::Point2f> > intersection_of_interpolated_algorithm_and_sroi;

                                MyIntersection intersection_interpolated_eroi_sroi_objects;
                                std::vector<std::pair<cv::Point2f, cv::Point2f> >::iterator result_it_interpolated;

                                result_it_interpolated = intersection_interpolated_eroi_sroi_objects.find_intersection_pair(gt_roi_object.at(sensor_index).at(current_frame_index).begin(), gt_roi_object.at(sensor_index).at(current_frame_index).end(), special_roi_object.at(sensor_index).at(current_frame_index).begin(), special_roi_object.at(sensor_index).at(current_frame_index).end(),
                                                                                                               intersection_of_interpolated_algorithm_and_sroi.begin());
                                intersection_of_interpolated_algorithm_and_sroi = intersection_interpolated_eroi_sroi_objects.getResultIntersectingPair();
                                bool isSorted = std::is_sorted(entire_roi_object_interpolated.at(sensor_index).at(current_frame_index).begin(), entire_roi_object_interpolated.at(sensor_index).at(current_frame_index).end(), PairPointsSort<float>());
                                assert(isSorted);
                                bool isSorted_sroi = std::is_sorted(special_roi_object.at(sensor_index).at(current_frame_index).begin(), special_roi_object.at(sensor_index).at(current_frame_index).end(), PairPointsSort<float>());
                                assert(isSorted_sroi);

                                //assert(intersection_of_algorithm_and_sroi.size() > 0);
                                // Validate
                                cv::Mat tempImageInterpolated(Dataset::m_frame_size, CV_8UC3);
                                tempImageInterpolated = cv::Scalar::all(255);
                                for ( auto it = intersection_of_interpolated_algorithm_and_sroi.begin(); it != intersection_of_interpolated_algorithm_and_sroi.end(); it++) {
                                    cv::circle(tempImageInterpolated, (*it).first, 1, cv::Scalar(255,0,0));
                                }
                                cv::imshow("interpolated_algorithm_sroi", tempImageInterpolated);
                                cv::waitKey(0);
                                cv::destroyAllWindows();

                                for (auto cluster_index = 0; cluster_index < intersection_of_interpolated_algorithm_and_sroi.size(); cluster_index++) {

                                    cv::Point2f algo_displacement = intersection_of_interpolated_algorithm_and_sroi.at(cluster_index).second;

                                    // l1_cumulative_distance_sroi_pixels
                                    auto l1_dist_err = ( std::abs(algo_displacement.x - gt_displacement.x ) + std::abs(algo_displacement.y - gt_displacement.y));
                                    l1_cumulative_distance_sroi_pixels += l1_dist_err;

                                    // l2_cumulative_distance_sroi_pixels
                                    auto euclidean_dist_algo_square = (std::pow((algo_displacement.x - gt_displacement.x),2 ) + std::pow((algo_displacement.y - gt_displacement.y),2 ));
                                    auto euclidean_dist_err = std::sqrt(euclidean_dist_algo_square);
                                    l2_cumulative_distance_sroi_pixels += euclidean_dist_err;

                                    // ma_cumulative_distance_sroi_pixels
                                    auto ma_dist_algo = Utils::getMahalanobisDistance(icovar, algo_displacement, evaluationData.at(obj_index).mean_displacement);
                                    ma_cumulative_distance_sroi_pixels += ma_dist_algo;

                                    //auto angle_algo = std::tanh(algo_displacement.y / algo_displacement.x);
                                    //auto angle_err = std::abs(angle_algo - angle_gt);
                                    //auto angle_err_dot = std::cosh(algo_displacement.dot(gt_displacement) / (euclidean_dist_gt * std::sqrt(euclidean_dist_algo_square)));
                                    //assert(angle_err_dot==angle_err);

                                    if (
                                            (l1_dist_err) < DISTANCE_ERROR_TOLERANCE
                                        //&& (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE
                                            ) {
                                        l1_cumulative_sroi_error_tolerated += l1_dist_err;
                                        evaluationData.at(
                                                obj_index).l1_total_count_sroi_good_pixels++; // how many pixels in the found pixel are actually valid
                                    }
                                    if (
                                            (euclidean_dist_err) < DISTANCE_ERROR_TOLERANCE
                                        //&& (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE
                                            ) {
                                        l2_cumulative_sroi_error_tolerated += euclidean_dist_err;
                                        evaluationData.at(
                                                obj_index).l2_total_count_sroi_good_pixels++; // how many pixels in the found pixel are actually valid
                                    }
                                    if (
                                            (ma_dist_algo) < DISTANCE_ERROR_TOLERANCE
                                        // && (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE
                                            ) {
                                        ma_cumulative_sroi_error_tolerated += ma_dist_algo;
                                        evaluationData.at(
                                                obj_index).ma_total_count_sroi_good_pixels++; // how many pixels in the found pixel are actually valid
                                    }

                                    xy_pts.push_back(std::make_pair(algo_displacement.x, algo_displacement.y));
                                }

                            }


                            // start gnuplotting
                            std::vector<std::pair<float, float>> gt_mean_pts, algo_mean_pts;
                            gt_mean_pts.push_back(std::make_pair(gt_displacement.x, gt_displacement.y));
                            algo_mean_pts.push_back(std::make_pair(evaluationData.at(obj_index).mean_displacement.x, evaluationData.at(obj_index).mean_displacement.y));

                            //Get the eigenvalues and eigenvectors
                            cv::Mat_<float> ellipse(3,1);

                            if ( CLUSTER_COUNT_ALGORITHM > 1 ) {

                                double chisquare_val = 2.4477;
                                cv::Mat_<float> eigenvectors(2,2);
                                cv::Mat_<float> eigenvalues(1,2);

                                cv::eigen(evaluationData.at(obj_index).covar_displacement, eigenvalues, eigenvectors);

                                //std::cout << "eigen " << eigenvectors << "\n" << eigenvalues << std::endl ;

                                if ( eigenvectors.data != NULL ) {
                                    //Calculate the angle between the largest eigenvector and the x-axis
                                    double angle = atan2(eigenvectors(0,1), eigenvectors(0,0));

                                    //Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
                                    if(angle < 0)
                                        angle += 6.28318530718;

                                    //Conver to degrees instead of radians
                                    angle = 180*angle/3.14159265359;

                                    //Calculate the size of the minor and major axes
                                    double halfmajoraxissize=chisquare_val*sqrt(eigenvalues(0));
                                    double halfminoraxissize=chisquare_val*sqrt(eigenvalues(1));

                                    //Return the oriented ellipse
                                    //The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
                                    ellipse << halfmajoraxissize, halfminoraxissize, -angle;

                                    //std::cout << "ellips" << ellipse;

                                    //cv::Mat visualizeimage(240, 320, CV_8UC1, cv::Scalar::all(0));
                                    //cv::ellipse(visualizeimage, ellipse, cv::Scalar::all(255), 2);
                                    //cv::imshow("EllipseDemo", visualizeimage);
                                    //cv::waitKey(1000);

                                }

                            }

                            float m, c;

                            std::string coord1;
                            std::string coord2;
                            std::string gp_line;
                            cv::Vec4f line = list_of_current_objects.at(
                                    obj_index)->
                                    get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index
                            ).at(sensor_index).at(current_frame_index).regression_line;;

                            m = line[1] / line[0];
                            c = line[3] - line[2] * m;
                            coord1 = "-4," + std::to_string(m*(-4) + c);
                            coord2 = "4," + std::to_string(m*(4) + c);
                            gp_line = "set arrow from " + coord1 + " to " + coord2 + " nohead lc rgb \'red\'\n";

                            if ( obj_index == 0 ) {

                                std::cout << "ellipse" << ellipse ;

                                std::string ellipse_plot = "set object 1 ellipse center " + std::to_string(evaluationData.at(obj_index).mean_displacement.x) + "," + std::to_string(evaluationData.at(obj_index).mean_displacement.y) + " size " + std::to_string(ellipse(0)) + "," +  std::to_string(ellipse(1)) + "  angle " + std::to_string(ellipse(2)) + " lw 5 front fs empty bo 3\n";

                                gp2d << "set term png size 400,400\n";
                                gp2d << "set output \"" + gnuplot_image_file_with_path + "\"\n";
                                gp2d << "set xrange [-5:5]\n";
                                gp2d << "set yrange [-5:5]\n";
                                //gp2d << gp_line;
                                gp2d << ellipse_plot;
                                gp2d << "plot '-' with points title '" + m_ptr_list_gt_objects.at(obj_index)->getObjectName() + "'"
                                        ", '-' with points pt 22 notitle 'GT'"
                                        ", '-' with points pt 15 notitle 'Algo'"
                                        // , '-' with circles linecolor rgb \"#FF0000\" fill solid notitle 'GT'"
                                        "\n";
                                gp2d.send1d(xy_pts);
                                gp2d.send1d(gt_mean_pts);
                                gp2d.send1d(algo_mean_pts);

                                usleep(100000);  // give some time to gnuplot to write the plot on the filesystem

                                if ( sensor_index != (SENSOR_COUNT-1) || ( sensor_index == 0 && SENSOR_COUNT == 1)) {

                                    if ( sensor_index == 0 ) {
                                        stich_plots.create(800, 2400, CV_8UC3); // make space for 6 objects
                                        stich_plots = cv::Scalar::all(0);
                                        //cv::imwrite(gnuplot_image_file_with_path_stiched, stich_plots);
                                    }
                                    cv::Mat gnuplot_index = cv::imread(gnuplot_image_file_with_path, CV_LOAD_IMAGE_ANYCOLOR);
                                    if ( gnuplot_index.empty()) {
                                        throw "no image is found error";
                                    }
                                    //cv::imshow("plot", gnuplot_index);
                                    //cv::waitKey(0);

                                    //stich_plots = cv::imread(gnuplot_image_file_with_path_stiched, CV_LOAD_IMAGE_ANYCOLOR);

                                    cv::Mat object_index_stich_location = stich_plots.rowRange(0+(sensor_index*400)*2, (sensor_index*400)*2 + 400).colRange(0,400);
                                    gnuplot_index.copyTo(object_index_stich_location);

                                }

                            }
                            else if ( obj_index == 5 ) {

                                //gp2d << "replot\n";

                                gp2d << "replot '-' with points title'" + m_ptr_list_gt_objects.at(obj_index)->getObjectName() + "'"
                                        ", '-' with points pt 22 notitle 'GT'"
                                        ", '-' with points pt 15 notitle 'Algo'"
                                        // , '-' with circles linecolor rgb \"#FF0000\" fill solid notitle 'GT'"
                                        "\n";

                                gp2d.send1d(xy_pts);
                                gp2d.send1d(gt_mean_pts);
                                gp2d.send1d(algo_mean_pts);
                            }


                            // shift stich multiple sensor images in a grid.

                        }

                        evaluationData.at(obj_index).l1_cumulative_distance_all_pixels = l1_cumulative_distance_all_pixels;
                        evaluationData.at(obj_index).l2_cumulative_distance_all_pixels = l2_cumulative_distance_all_pixels;
                        evaluationData.at(obj_index).ma_cumulative_distance_all_pixels = ma_cumulative_distance_all_pixels;

                        evaluationData.at(obj_index).l1_cumulative_distance_good_pixels = l1_cumulative_error_tolerated;
                        evaluationData.at(obj_index).l2_cumulative_distance_good_pixels = l2_cumulative_error_tolerated;
                        evaluationData.at(obj_index).ma_cumulative_distance_good_pixels = ma_cumulative_error_tolerated;

                        std::cout << "l1_cumulative_distance_good_pixels for object "
                                  << list_of_current_objects.at(obj_index)->getObjectName() << " = "
                                  << evaluationData.at(obj_index).l1_cumulative_distance_good_pixels << std::endl;
                        std::cout << "l2_cumulative_distance_good_pixels for object "
                                  << list_of_current_objects.at(obj_index)->getObjectName() << " = "
                                  << evaluationData.at(obj_index).l2_cumulative_distance_good_pixels << std::endl;
                        std::cout << "ma_cumulative_distance_good_pixels for object "
                                  << list_of_current_objects.at(obj_index)->getObjectName() << " = "
                                  << evaluationData.at(obj_index).ma_cumulative_distance_good_pixels << std::endl;
                        std::cout << "algorithm_pixels_count for object "
                                  << list_of_current_objects.at(obj_index)->getObjectName() << " = "
                                  << evaluationData.at(obj_index).algorithm_pixels_count << std::endl;

                        //assert(evaluationData.l2_cumulative_distance_good_pixels <= std::ceil(evaluationData.algorithm_pixels_count) + 20 );

                    } else {
                        std::cout << "visibility of object "
                                  << list_of_current_objects.at(obj_index)->getObjectName() << " = " <<
                                  list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(
                                                  sensor_index)
                                          .at(current_frame_index)
                                  << " and hence not generating any shape points for this object " << std::endl;

                        evaluationData.at(obj_index).l1_cumulative_distance_good_pixels = 0;
                        evaluationData.at(obj_index).l2_cumulative_distance_good_pixels = 0;
                        evaluationData.at(obj_index).ma_cumulative_distance_good_pixels = 0;
                        evaluationData.at(obj_index).algorithm_pixels_count = 0;

                    }
                }

                if (m_opticalFlowName != "ground_truth") {
                    cv::imwrite(gnuplot_image_file_with_path_stiched, stich_plots);
                }
                multiframe_evaluation_data.push_back(evaluationData);
            }

            sensor_multiframe_evaluation_data.push_back(multiframe_evaluation_data);
            //sensor_scenario_displacement_occurence.push_back(scenario_displacement_occurence);

            /* generate for every algorithm, an extra sensor */
            //if ( sensor_index == (SENSOR_COUNT-1) ) {
            //    generate_shape_points_sensor_fusion(datafilter_index, sensor_shape_points );
            //}
        }

        m_sensor_scenario_displacement_occurence = sensor_scenario_displacement_occurence;
        m_sensor_multiframe_evaluation_data.push_back(sensor_multiframe_evaluation_data);
        std::cout << "end of generate_metrics_optical_flow_algorithm. this is the last step" << std::endl;

    }

}
