#include "OpticalFlow.h"
#include <gnuplot-iostream/gnuplot-iostream.h>
#include "Utils.h"

void OpticalFlow::generate_metrics_optical_flow_algorithm(ushort SENSOR_COUNT) {

    std::vector<Objects *> list_of_current_objects;

    const float DISTANCE_ERROR_TOLERANCE = 2;

    unsigned COUNT;
    if (m_opticalFlowName == "ground_truth") {
        COUNT = 1;
        list_of_current_objects = m_ptr_list_gt_objects;
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
                char file_name_image_output[50];
                std::string output_image_file_with_path, output_image_file_with_path_stiched;
                ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                        (0).at(current_frame_index).frame_no;
                sprintf(file_name_image_output, "000%03d_10.png", image_frame_count);
                output_image_file_with_path = m_gnuplots_path.string() + "0" + std::to_string(sensor_index) + "/" + file_name_image_output;

                output_image_file_with_path_stiched = m_gnuplots_path.string() + "0" + std::to_string(SENSOR_COUNT-1) + "/" + file_name_image_output;

                std::cout << "current_frame_index " << current_frame_index << " for opticalflow_index " << m_opticalFlowName
                          << std::endl;

                Gnuplot gp2d;

                for (ushort obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++) {


                    // displacements found by the ground truth for this object
                    auto CLUSTER_COUNT_GT = m_ptr_list_gt_objects.at(
                            obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).size();

                    cv::Point2i dimension = {
                            cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    sensor_index).at(current_frame_index).m_region_of_interest_px.width_px),
                            cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    sensor_index).at(current_frame_index).m_region_of_interest_px.height_px)
                    };

                    // displacements found by the algorithm for this object
                    unsigned CLUSTER_COUNT = (unsigned) list_of_current_objects.at(
                            obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(
                            current_frame_index).size();

                    evaluationData.at(obj_index).current_frame_index = image_frame_count;
                    evaluationData.at(obj_index).obj_index = obj_index;

                    if (list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(
                            sensor_index).at(current_frame_index)) {

                        // Instances of CLUSTER_COUNT_ALGO in CLUSTER_COUNT_GT

                        cv::Point2f gt_displacement = m_ptr_list_gt_objects.at(
                                obj_index)->get_object_extrapolated_point_displacement().at
                                (sensor_index).at(current_frame_index).second;

                        auto euclidean_dist_gt = cv::norm(gt_displacement);
                        auto angle_gt = std::tanh(gt_displacement.y / gt_displacement.x);

                        evaluationData.at(obj_index).object_dimension = dimension;

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
                                obj_index).visiblePixels = CLUSTER_COUNT; //(dimension.x * dimension.y); // how many pixels are visible ( it could be that some pixels are occluded )
                        evaluationData.at(
                                obj_index).goodPixels_l2 = CLUSTER_COUNT; // how many pixels in the found pixel are actually valid
                        evaluationData.at(
                                obj_index).goodPixels_maha = CLUSTER_COUNT; // how many pixels in the found pixel are actually valid

                        double l1_cumulative = 0;
                        double l2_cumulative = 0;
                        double maha_cumulative = 0;

                        double l2_good = 0;
                        double maha_good = 0;

                        cv::Mat icovar;
                        // this should be sent to Objects.cpp
                        if ( evaluationData.at(obj_index).covar_displacement.data != NULL ) {

                            icovar = evaluationData.at(obj_index).covar_displacement.inv(cv::DECOMP_SVD);

                        }

                        if (m_opticalFlowName != "ground_truth") {

                            // how many pixelsi are visible ( it could be that some pixels are occluded ). This wll be found out using k-means
                            evaluationData.at(
                                    obj_index).goodPixels_l2 = 0; // how many pixels in the found pixel are actually valid
                            evaluationData.at(
                                    obj_index).goodPixels_maha = 0; // how many pixels in the found pixel are actually valid

                            std::vector<std::pair<float, float>> xy_pts;

                            for (auto cluster_index = 0; cluster_index < CLUSTER_COUNT; cluster_index++) {

                                cv::Point2f algo_displacement = list_of_current_objects.at(obj_index)->
                                        get_list_object_dataprocessing_stencil_points_displacement().at(datafilter_index
                                ).at(sensor_index).at(current_frame_index).at(cluster_index).second;


                                l1_cumulative += ( std::abs(algo_displacement.x ) + std::abs(algo_displacement.y)) ;

                                auto euclidean_dist_algo_square = (std::pow((algo_displacement.x - gt_displacement.x),2 ) + std::pow((algo_displacement.y - gt_displacement.y),2 ));
                                l2_cumulative += euclidean_dist_algo_square;

                                auto maha_dist_algo = Utils::getMahalanobisDistance(icovar, algo_displacement, evaluationData.at(obj_index).mean_displacement);
                                maha_cumulative += maha_dist_algo;

                                auto euclidean_dist_err = std::sqrt(euclidean_dist_algo_square);
                                //auto maha_dist_err = std::abs(euclidean_dist_gt - maha_dist_algo);
                                auto angle_algo = std::tanh(algo_displacement.y / algo_displacement.x);

                                auto angle_err = std::abs(angle_algo - angle_gt);
                                auto angle_err_dot = std::cosh(
                                        algo_displacement.dot(gt_displacement) / (euclidean_dist_gt * std::sqrt(euclidean_dist_algo_square)));

                                //assert(angle_err_dot==angle_err);
                                if (
                                        (euclidean_dist_err) < DISTANCE_ERROR_TOLERANCE
                                    //&& (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE

                                        ) {
                                    l2_good += euclidean_dist_algo_square;
                                    evaluationData.at(
                                            obj_index).goodPixels_l2++; // how many pixels in the found pixel are actually valid
                                }
                                if (
                                        (maha_dist_algo) < DISTANCE_ERROR_TOLERANCE
                                    // && (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE

                                        ) {
                                    maha_good += euclidean_dist_algo_square;
                                    evaluationData.at(
                                            obj_index).goodPixels_maha++; // how many pixels in the found pixel are actually valid
                                }

                                xy_pts.push_back(std::make_pair(algo_displacement.x, algo_displacement.y));
                            }

                            std::vector<std::pair<float, float>> gt_mean_pts, algo_mean_pts;
                            gt_mean_pts.push_back(std::make_pair(gt_displacement.x, gt_displacement.y));
                            algo_mean_pts.push_back(std::make_pair(evaluationData.at(obj_index).mean_displacement.x, evaluationData.at(obj_index).mean_displacement.y));

                            //Get the eigenvalues and eigenvectors
                            cv::Mat_<float> ellipse(3,1);

                            if ( CLUSTER_COUNT > 1 ) {
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
                                gp2d << "set output \"" + output_image_file_with_path + "\"\n";
                                gp2d << "set xrange [-5:5]\n";
                                gp2d << "set yrange [-5:5]\n";
                                gp2d << gp_line;
                                gp2d << ellipse_plot;
                                gp2d << "plot '-' with points title 'Car'"
                                        ", '-' with points pt 22 notitle 'GT'"
                                        ", '-' with points pt 15 notitle 'Algo'"
                                        //", '-' with points title 'Boy'
                                        // , '-' with circles linecolor rgb \"#FF0000\" fill solid notitle 'GT'"
                                        "\n";
                                gp2d.send1d(xy_pts);
                                gp2d.send1d(gt_mean_pts);
                                gp2d.send1d(algo_mean_pts);



                            }
                            else if ( obj_index == 5 ) {

                                //gp2d << "replot\n";
                                //gp2d << "replot '-' with points title 'Car', '-' with circles linecolor rgb \"#FF0000\" fill solid title 'GT'\n";
                                gp2d.send1d(xy_pts);
                                gp2d.send1d(gt_mean_pts);
                                gp2d.send1d(algo_mean_pts);
                            }

                            // shift stich plots somewhere else
                            if ( obj_index == 0 && current_frame_index > 0  && sensor_index != (SENSOR_COUNT-1) ) {

                                cv::Mat stich_plots;

                                std::string ground_truth_image_path = m_GroundTruthImageLocation.string() + "_" + std::to_string(sensor_index) + "/" + file_name_image_output;
                                cv::Mat ground_truth_image = cv::imread(ground_truth_image_path, CV_LOAD_IMAGE_ANYCOLOR);
                                if (boost::filesystem::exists(output_image_file_with_path_stiched)) {
                                    stich_plots = cv::imread(output_image_file_with_path_stiched, CV_LOAD_IMAGE_ANYCOLOR);
                                }
                                else {
                                    stich_plots.create(1200, 1200, CV_8UC3);
                                    stich_plots = cv::Scalar::all(0);
                                }
                                cv::Mat roi = stich_plots.rowRange(400, 800).colRange(0+sensor_index*400, sensor_index*400+400);
                                cv::Mat roi_vires_image = stich_plots.rowRange(0+(sensor_index*400)*2, (sensor_index*400)*2 + 400);
                                ground_truth_image.copyTo(roi_vires_image);

                                cv::Mat get_image;
                                while (get_image.data == NULL) {
                                    get_image = cv::imread(output_image_file_with_path, CV_LOAD_IMAGE_ANYCOLOR);
                                    usleep(100);
                                }
                                get_image.copyTo(roi);
                                cv::imwrite(output_image_file_with_path_stiched, stich_plots);
                            }

                        }
                        l1_cumulative   = l1_cumulative / CLUSTER_COUNT;
                        l2_cumulative   = (l2_cumulative) / CLUSTER_COUNT;
                        maha_cumulative = (maha_cumulative) / CLUSTER_COUNT;

                        l2_good   = (l2_good) / CLUSTER_COUNT;
                        maha_good = (maha_good) / CLUSTER_COUNT;

                        evaluationData.at(obj_index).l1 = l1_cumulative;
                        evaluationData.at(obj_index).l2 = l2_cumulative;
                        evaluationData.at(obj_index).l2 = l2_good;
                        evaluationData.at(obj_index).mahalanobisDistance = maha_cumulative;
                        evaluationData.at(obj_index).mahalanobisDistance = maha_good;

                        std::cout << "goodPixels_l2 for object "
                                  << list_of_current_objects.at(obj_index)->getObjectName() << " = "
                                  << evaluationData.at(obj_index).goodPixels_l2 << std::endl;
                        std::cout << "goodPixels_maha for object "
                                  << list_of_current_objects.at(obj_index)->getObjectName() << " = "
                                  << evaluationData.at(obj_index).goodPixels_maha << std::endl;
                        std::cout << "visiblePixels for object "
                                  << list_of_current_objects.at(obj_index)->getObjectName() << " = "
                                  << evaluationData.at(obj_index).visiblePixels << std::endl;

                        //assert(evaluationData.goodPixels_l2 <= std::ceil(evaluationData.algorithmPixels) + 20 );

                    } else {
                        std::cout << "visibility of object "
                                  << list_of_current_objects.at(obj_index)->getObjectName() << " = " <<
                                  list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(
                                                  sensor_index)
                                          .at(current_frame_index)
                                  << " and hence not generating any shape points for this object " << std::endl;

                        evaluationData.at(obj_index).goodPixels_l2 = 0;
                        evaluationData.at(obj_index).goodPixels_maha = 0;
                        evaluationData.at(obj_index).visiblePixels = 0;

                    }
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
        std::cout << "adding list to object" << std::endl;

    }

}
