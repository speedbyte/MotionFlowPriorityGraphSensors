
#include "OpticalFlow.h"
#include <gnuplot-iostream/gnuplot-iostream.h>
#include "Utils.h"

void OpticalFlow::generate_metrics_optical_flow_algorithm() {

    std::vector<Objects *> ptr_list_of_current_objects;
    std::vector<Objects *> ptr_list_of_copied_gt_objects;
    std::vector<Objects *> ptr_list_of_copied_simulated_objects;

    for ( auto i = 0; i < m_ptr_list_gt_objects.size(); i++) {
        ptr_list_of_copied_gt_objects.push_back(static_cast<Objects*>(m_ptr_list_gt_objects.at(i)));
    }

    unsigned COUNT;
    if (m_opticalFlowName == "ground_truth") {
        COUNT = 1;
        ptr_list_of_current_objects = ptr_list_of_copied_gt_objects;
    } else {
        for ( auto i = 0; i < get_simulated_objects_ptr_list().size(); i++) {
            ptr_list_of_copied_simulated_objects.push_back(static_cast<Objects*>(get_simulated_objects_ptr_list().at(i)));
        }
        ptr_list_of_current_objects = ptr_list_of_copied_simulated_objects;
        COUNT = (unsigned)ptr_list_of_current_objects.at(0)->
                get_list_object_dataprocessing_mean_centroid_displacement().size();
    }

    for (unsigned datafilter_index = 0; datafilter_index < COUNT; datafilter_index++) {

        std::vector<std::map<std::pair<float, float>, int> > sensor_scenario_displacement_occurence;
        std::vector<std::vector<std::vector<OPTICAL_FLOW_EVALUATION_METRICS> > > sensor_multiframe_evaluation_data;

        for (unsigned sensor_index = 0; sensor_index < Dataset::SENSOR_COUNT; sensor_index++) {

            std::cout << "generating evaluation metrics in OpticalFlow.cpp for sensor index " << sensor_index
                      << " for opticalflow  " << m_opticalFlowName << std::endl;

            unsigned FRAME_COUNT = (unsigned) ptr_list_of_current_objects.at(0)
                    ->get_object_stencil_point_displacement().at(sensor_index).size();

            assert(FRAME_COUNT > 0);

            std::vector<std::vector<OPTICAL_FLOW_EVALUATION_METRICS> > multiframe_evaluation_data;

            for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

                (ptr_list_of_current_objects.size());
                std::vector<OPTICAL_FLOW_EVALUATION_METRICS> evaluationData(ptr_list_of_current_objects.size());


                ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                        (0).at(current_frame_index).frame_no;

                std::cout << "current_frame_index " << current_frame_index << " for opticalflow_index " << m_opticalFlowName
                          << std::endl;

                for (ushort obj_index = 0; obj_index < ptr_list_of_current_objects.size(); obj_index++) {


                    // displacements found by the ground truth for this object
                    auto CLUSTER_COUNT_GT = m_ptr_list_gt_objects.at(
                            obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).size();

                    evaluationData.at(obj_index).frame_number = image_frame_count;
                    evaluationData.at(obj_index).obj_index = obj_index;
                    evaluationData.at(obj_index).visiblity = ptr_list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(sensor_index).at(current_frame_index);

                    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > entire_roi_object = ptr_list_of_current_objects.at(obj_index)->get_object_stencil_point_displacement();

                    std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > > contour_roi_object = ptr_list_of_current_objects.at(obj_index)->get_object_contour_region_of_interest();

                    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > entire_roi_object_interpolated = ptr_list_of_current_objects.at(obj_index)->get_object_interpolated_stencil_point_displacement();

                    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > intersection_of_algorithm_and_sroi = ptr_list_of_current_objects.at(obj_index)->get_object_special_region_of_interest();

                    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > intersection_of_interpolated_algorithm_and_sroi = ptr_list_of_current_objects.at(obj_index)->get_object_interpolated_intersection_sroi();

                    if (evaluationData.at(obj_index).visiblity) {

                        evaluationData.at(obj_index).regression_line = ptr_list_of_current_objects.at(obj_index)->
                                get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index
                        ).at(sensor_index).at(current_frame_index).regression_line;

                        evaluationData.at(obj_index).mean_pts = ptr_list_of_current_objects.at(obj_index)->
                                get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index
                        ).at(sensor_index).at(current_frame_index).mean_pts;

                        evaluationData.at(obj_index).mean_displacement = ptr_list_of_current_objects.at(obj_index)->
                                get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index
                        ).at(sensor_index).at(current_frame_index).mean_displacement;

                        evaluationData.at(obj_index).stddev_pts = ptr_list_of_current_objects.at(
                                obj_index)->
                                get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index
                        ).at(sensor_index).at(current_frame_index).stddev_pts;

                        evaluationData.at(obj_index).stddev_displacement = ptr_list_of_current_objects.at(
                                obj_index)->
                                get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index
                        ).at(sensor_index).at(current_frame_index).stddev_displacement;

                        evaluationData.at(obj_index).covar_pts = ptr_list_of_current_objects.at(
                                obj_index)->
                                get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index
                        ).at(sensor_index).at(current_frame_index).covar_pts;

                        evaluationData.at(obj_index).covar_displacement = ptr_list_of_current_objects.at(
                                obj_index)->
                                get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index
                        ).at(sensor_index).at(current_frame_index).covar_displacement;

                        // Instances of CLUSTER_COUNT_ALGO in CLUSTER_COUNT_GT
                        cv::Mat icovar;
                        // this should be written in Objects.cpp
                        if ( evaluationData.at(obj_index).covar_displacement.data != NULL ) {

                            icovar = evaluationData.at(obj_index).covar_displacement.inv(cv::DECOMP_SVD);
                        }

                        std::vector<std::pair<float, float>> gnuplot_xy_pts;

//--------------------------------------------------------------------------------------------
                        COUNT_METRICS &entire_roi_object_count_metrics = evaluationData.at(obj_index).entire_metrics;
                        gnuplot_xy_pts = generate_count_metrics_data("entire", entire_roi_object, sensor_index, current_frame_index, obj_index, evaluationData, entire_roi_object_count_metrics, icovar);
                        show_gnuplot("entire", gnuplot_xy_pts, sensor_index, current_frame_index, obj_index, evaluationData, entire_roi_object_count_metrics, icovar);

//--------------------------------------------------------------------------------------------

                        COUNT_METRICS &entire_roi_object_interpolated_count_metrics = evaluationData.at(obj_index).entire_interpolated_metrics;
                        gnuplot_xy_pts = generate_count_metrics_data("entire_interpolated", entire_roi_object_interpolated, sensor_index, current_frame_index, obj_index, evaluationData, entire_roi_object_interpolated_count_metrics, icovar);
                        show_gnuplot("entire_interpolated", gnuplot_xy_pts, sensor_index, current_frame_index, obj_index, evaluationData, entire_roi_object_count_metrics, icovar);

//--------------------------------------------------------------------------------------------

                        COUNT_METRICS &special_roi_object_count_metrics = evaluationData.at(obj_index).sroi_metrics;
                        gnuplot_xy_pts = generate_count_metrics_data("special", intersection_of_algorithm_and_sroi, sensor_index, current_frame_index, obj_index, evaluationData, special_roi_object_count_metrics, icovar);
                        show_gnuplot("special", gnuplot_xy_pts, sensor_index, current_frame_index, obj_index, evaluationData, entire_roi_object_count_metrics, icovar);

//--------------------------------------------------------------------------------------------

                        COUNT_METRICS &special_roi_object_interpolated_count_metrics = evaluationData.at(obj_index).sroi_interpolated_metrics;
                        gnuplot_xy_pts = generate_count_metrics_data("special_interpolated", intersection_of_interpolated_algorithm_and_sroi, sensor_index, current_frame_index, obj_index, evaluationData, special_roi_object_interpolated_count_metrics, icovar);
                        show_gnuplot("special_interpolated", gnuplot_xy_pts, sensor_index, current_frame_index, obj_index, evaluationData, entire_roi_object_count_metrics, icovar);

//--------------------------------------------------------------------------------------------
                        evaluationData.at(obj_index).all_contour_size = contour_roi_object.at(sensor_index).at(current_frame_index).size();
                        for ( ushort contour_index = 0 ; contour_index < evaluationData.at(obj_index).all_contour_size; contour_index++) {
                            evaluationData.at(obj_index).all_contour_pixels[contour_index] = contour_roi_object.at(sensor_index).at(current_frame_index).at(contour_index).size();
                        }


                    } else {

                        std::cout << "visibility of object "
                                  << ptr_list_of_current_objects.at(obj_index)->getObjectName() << " = " <<
                                  ptr_list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(
                                                  sensor_index)
                                          .at(current_frame_index)
                                  << " and hence not generating any shape points for this object " << std::endl;
                        // all struct values are implicitly set to 0.


                        std::vector<std::pair<float, float>> gnuplot_xy_pts = {{0,0}};
                        cv::Mat icovar;

//--------------------------------------------------------------------------------------------
                        COUNT_METRICS &entire_roi_object_count_metrics = evaluationData.at(
                                obj_index).entire_metrics;
                        entire_roi_object_count_metrics.sync_point[0] = {'$'};
                        show_gnuplot("entire", gnuplot_xy_pts, sensor_index, current_frame_index, obj_index,
                                     evaluationData, entire_roi_object_count_metrics, icovar);

//--------------------------------------------------------------------------------------------

                        COUNT_METRICS &entire_roi_object_interpolated_count_metrics = evaluationData.at(
                                obj_index).entire_interpolated_metrics;
                        entire_roi_object_interpolated_count_metrics.sync_point[0] = {'$'};
                        show_gnuplot("entire_interpolated", gnuplot_xy_pts, sensor_index, current_frame_index,
                                     obj_index, evaluationData, entire_roi_object_count_metrics, icovar);

//--------------------------------------------------------------------------------------------

                        COUNT_METRICS &special_roi_object_count_metrics = evaluationData.at(
                                obj_index).sroi_metrics;
                        special_roi_object_count_metrics.sync_point[0] = {'$'};
                        show_gnuplot("special", gnuplot_xy_pts, sensor_index, current_frame_index, obj_index,
                                     evaluationData, entire_roi_object_count_metrics, icovar);

//--------------------------------------------------------------------------------------------

                        COUNT_METRICS &special_roi_object_interpolated_count_metrics = evaluationData.at(
                                obj_index).sroi_interpolated_metrics;
                        special_roi_object_interpolated_count_metrics.sync_point[0] = {'$'};
                        show_gnuplot("special_interpolated", gnuplot_xy_pts, sensor_index, current_frame_index,
                                     obj_index, evaluationData, entire_roi_object_count_metrics, icovar);

//--------------------------------------------------------------------------------------------
                    }
                }

                multiframe_evaluation_data.push_back(evaluationData);
            }

            sensor_multiframe_evaluation_data.push_back(multiframe_evaluation_data);
            //sensor_scenario_displacement_occurence.push_back(scenario_displacement_occurence);

            /* generate for every algorithm, an extra sensor */
            //if ( sensor_index == (Dataset::SENSOR_COUNT-1) ) {
            //    generate_shape_points_sensor_fusion(datafilter_index, sensor_shape_points );
            //}
        }

        m_sensor_scenario_displacement_occurence = sensor_scenario_displacement_occurence;
        m_sensor_multiframe_evaluation_data.push_back(sensor_multiframe_evaluation_data);
        std::cout << "end of generate_metrics_optical_flow_algorithm. this is the last step" << std::endl;

    }
}


std::vector<std::pair<float, float>> OpticalFlow::generate_count_metrics_data(std::string gnuplotname_prefix, const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &cluster_to_evaluate, const ushort sensor_index, const ushort current_frame_index,  const ushort obj_index, const std::vector<OPTICAL_FLOW_EVALUATION_METRICS> &evaluationData, COUNT_METRICS &count_metrics, cv::Mat &icovar) {

    const float DISTANCE_ERROR_TOLERANCE = 1;

// displacements found by the algorithm for this object
    unsigned CLUSTER_COUNT = (unsigned) cluster_to_evaluate.at(sensor_index).at(
            current_frame_index).size();

    count_metrics.sync_point[0] = {'$'};

    count_metrics.all_pixels = (ushort)CLUSTER_COUNT;

    std::vector<std::pair<float, float>> gnuplot_xy_pts;
    cv::Point2f gt_displacement;
    if ( m_opticalFlowName == "ground_truth") {
        gt_displacement = m_ptr_list_gt_objects.at(obj_index)->get_object_extrapolated_point_displacement().at(sensor_index).at(current_frame_index).second;
    } else {
        gt_displacement = m_ptr_gt_flow->get_sensor_multiframe_evaluation_data().at(0).at(sensor_index).at(current_frame_index).at(obj_index).mean_displacement;
    }
    auto euclidean_dist_gt = cv::norm(gt_displacement);
    auto angle_gt = std::tanh(gt_displacement.y / gt_displacement.x);


    double l1_cumulative_error_all_pixels = 0;
    double l2_cumulative_error_all_pixels = 0;
    double ma_cumulative_error_all_pixels = 0;

    double l1_cumulative_error_tolerated = 0;
    double l2_cumulative_error_tolerated = 0;
    double ma_cumulative_error_tolerated = 0;

    for (auto cluster_index = 0; cluster_index < CLUSTER_COUNT; cluster_index++) {

        cv::Point2f algo_displacement = cluster_to_evaluate.at(sensor_index).at(current_frame_index).at(cluster_index).second;

        // l1_cumulative_error_all_pixels
        auto l1_dist_err = ( std::abs(algo_displacement.x - gt_displacement.x ) + std::abs(algo_displacement.y - gt_displacement.y));
        l1_cumulative_error_all_pixels += l1_dist_err;

        // l2_cumulative_error_all_pixels
        auto euclidean_dist_algo_square = (std::pow((algo_displacement.x - gt_displacement.x),2 ) + std::pow((algo_displacement.y - gt_displacement.y),2 ));
        auto euclidean_dist_err = std::sqrt(euclidean_dist_algo_square);
        l2_cumulative_error_all_pixels += euclidean_dist_err;

        // ma_cumulative_error_all_pixels
        auto ma_dist_algo = Utils::getMahalanobisDistance(icovar, algo_displacement, evaluationData.at(obj_index).mean_displacement);
        ma_cumulative_error_all_pixels += ma_dist_algo;

        //auto angle_algo = std::tanh(algo_displacement.y / algo_displacement.x);
        //auto angle_err = std::abs(angle_algo - angle_gt);
        //auto angle_err_dot = std::cosh(algo_displacement.dot(gt_displacement) / (euclidean_dist_gt * std::sqrt(euclidean_dist_algo_square)));
        //assert(angle_err_dot==angle_err);

        if (
                (l1_dist_err) < DISTANCE_ERROR_TOLERANCE
            //&& (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE
                ) {
            l1_cumulative_error_tolerated += l1_dist_err;
            count_metrics.l1_good_pixels++; // how many valid pixels in the found pixel are actually
        }
        if (
                (euclidean_dist_err) < DISTANCE_ERROR_TOLERANCE
            //&& (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE
                ) {
            l2_cumulative_error_tolerated += euclidean_dist_err;
            count_metrics.l2_good_pixels++; // how many pixels in the found pixel are actually valid
        }
        if (
                (ma_dist_algo) < DISTANCE_ERROR_TOLERANCE
            // && (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE
                ) {
            ma_cumulative_error_tolerated += ma_dist_algo;
            count_metrics.ma_good_pixels++; // how many pixels in the found pixel are actually valid
        }

        gnuplot_xy_pts.push_back(std::make_pair(algo_displacement.x, algo_displacement.y));
    }

    count_metrics.l1_cumulative_error_all_pixels = l1_cumulative_error_all_pixels;
    count_metrics.l2_cumulative_error_all_pixels = l2_cumulative_error_all_pixels;
    count_metrics.ma_cumulative_error_all_pixels = ma_cumulative_error_all_pixels;
    count_metrics.l1_cumulative_error_good_pixels = l1_cumulative_error_tolerated;
    count_metrics.l2_cumulative_error_good_pixels = l2_cumulative_error_tolerated;
    count_metrics.ma_cumulative_error_good_pixels = ma_cumulative_error_tolerated;


    // Overload operator
    //std::cout << "count metrics for object index "          << obj_index << " = " << count_metrics << std::endl;

    return gnuplot_xy_pts;

}


void OpticalFlow::show_gnuplot(std::string gnuplotname_prefix, const std::vector<std::pair<float, float> > &gnuplot_xy_pts, const ushort sensor_index, const ushort current_frame_index, const ushort obj_index, const std::vector<OPTICAL_FLOW_EVALUATION_METRICS> &evaluationData, COUNT_METRICS &count_metrics, cv::Mat &icovar) {


    if ( m_opticalFlowName != "ground_truth") {

        std::vector<std::pair<float, float>> gt_mean_pts, algo_mean_pts;
        gt_mean_pts.push_back(std::make_pair(m_ptr_gt_flow->get_sensor_multiframe_evaluation_data().at(0).at(sensor_index).at(current_frame_index).at(obj_index).mean_displacement.x, m_ptr_gt_flow->m_sensor_multiframe_evaluation_data.at(0).at(sensor_index).at(current_frame_index).at(obj_index).mean_displacement.y));
        algo_mean_pts.push_back(std::make_pair(evaluationData.at(obj_index).mean_displacement.x, evaluationData.at(obj_index).mean_displacement.y));

        //Get the eigenvalues and eigenvectors
        cv::Mat_<float> ellipse(3,1);
        ellipse.setTo(255);
        if ( 1 > 1 ) {

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

        Gnuplot gp2d;
        float m, c;

        std::string coord1;
        std::string coord2;
        std::string gp_line;
        cv::Vec4f line = evaluationData.at(obj_index).regression_line;

        m = line[1] / line[0];
        c = line[3] - line[2] * m;
        coord1 = "-4," + std::to_string(m*(-4) + c);
        coord2 = "4," + std::to_string(m*(4) + c);
        gp_line = "set arrow from " + coord1 + " to " + coord2 + " nohead lc rgb \'red\'\n";

        char file_name_image_output[50], sensor_index_folder_suffix[10];
        std::string gnuplot_image_file_with_path;
        sprintf(sensor_index_folder_suffix, "%02d", sensor_index);
        sprintf(file_name_image_output, "%s_000%03d_10.png", gnuplotname_prefix.c_str(), evaluationData.at(obj_index).frame_number);
        gnuplot_image_file_with_path = m_gnuplots_path.string() + sensor_index_folder_suffix + "/" + file_name_image_output;

        if ( obj_index == 0 ) {

            std::cout << "ellipse for " << gnuplotname_prefix << " is \n" << ellipse << std::endl;

            std::string ellipse_plot = "set object 1 ellipse center " + std::to_string(evaluationData.at(obj_index).mean_displacement.x) + "," + std::to_string(evaluationData.at(obj_index).mean_displacement.y) + " size " + std::to_string(ellipse(0)) + "," +  std::to_string(ellipse(1)) + "  angle " + std::to_string(ellipse(2)) + " lw 5 front fs empty bo 3\n";

            gp2d << "set term png size 400,400\n";
            //gp2d << "set output \"" + std::string("../gnuplot.png") + "\"\n";
            gp2d << "set output \"" + gnuplot_image_file_with_path + "\"\n";
            gp2d << "set xrange [-5:5]\n";
            gp2d << "set yrange [-5:5]\n";
            if ( !std::isnan(m)) {
                gp2d << gp_line;
            }
            gp2d << ellipse_plot;
            gp2d << "plot '-' with points title '" + m_ptr_list_gt_objects.at(obj_index)->getObjectName() + "'"
                    ", '-' with points pt 22 title 'Mean GT'"
                    ", '-' with points pt 15 title 'Mean Algo'"
                    // , '-' with circles linecolor rgb \"#FF0000\" fill solid notitle 'GT'"
                    "\n";
            gp2d.send1d(gnuplot_xy_pts);
            gp2d.send1d(gt_mean_pts);
            gp2d.send1d(algo_mean_pts);

            usleep(100000);  // give some time to gnuplot to write the plot on the filesystem

        }
    }
    // shift stich multiple sensor images in a grid.
}

