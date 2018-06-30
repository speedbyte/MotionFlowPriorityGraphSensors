//
// Created by veikas on 06.02.18.
//

#include <map>
#include <chrono>
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>
#include "OpticalFlow.h"
#include "FlowImageExtended.h"
#include "Objects.h"
#include "Utils.h"
#include <gnuplot-iostream/gnuplot-iostream.h>

using namespace std::chrono;


void OpticalFlow::prepare_directories_common(ushort SENSOR_COUNT) {

    char char_dir_append[20];

    std::cout << "Creating Flow directories " << m_resultordner << std::endl;

    if (boost::filesystem::exists(m_generatepath)) {
        system(("rm -rf " + m_generatepath.string()).c_str());
    }
    boost::filesystem::create_directories(m_generatepath);

    boost::filesystem::path path;

    for (int i = 0; i < SENSOR_COUNT; ++i) {

        sprintf(char_dir_append, "%02d", i);

        m_collision_object_path = m_generatepath.string() + "/collision_object_";
        path =  m_collision_object_path.string() + char_dir_append;
        boost::filesystem::create_directories(path);

        m_flow_occ_path = m_generatepath.string() + "/flow_occ_";
        path =  m_flow_occ_path.string() + char_dir_append;
        boost::filesystem::create_directories(path);

        m_edge_path = m_generatepath.string() + "/edge_";
        path =  m_edge_path.string() + char_dir_append;
        boost::filesystem::create_directories(path);

        m_position_occ_path = m_generatepath.string() + "/position_occ_";
        path =  m_position_occ_path.string() + char_dir_append;
        boost::filesystem::create_directories(path);

        m_plots_path = m_generatepath.string() + "/plots_";
        path =  m_plots_path.string() + char_dir_append;
        boost::filesystem::create_directories(path);

        m_gnuplots_path = m_generatepath.string() + "/gnuplots_";
        path =  m_gnuplots_path.string() + char_dir_append;
        boost::filesystem::create_directories(path);


    }

    std::cout << "Ending Flow directories " << m_resultordner << std::endl;

}


void OpticalFlow::common_flow_frame(ushort sensor_index, ushort current_frame_index, std::vector<cv::Point2f> &frame_next_pts_array, std::vector<cv::Point2f>  &displacement_array,std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &multiframe_stencil_displacement, std::vector<std::vector<std::vector<bool> >  > &multiframe_stencil_visibility) {

    char sensor_index_folder_suffix[50];
    sprintf(sensor_index_folder_suffix, "%02d", sensor_index);

    char file_name_image_output[50];
    ushort evaluation_frame_index = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
            (0).at(current_frame_index).frame_no;
    sprintf(file_name_image_output, "000%03d_10.png", evaluation_frame_index);


    for (ushort obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++) {
        std::vector<std::pair<cv::Point2f, cv::Point2f> > frame_stencil_displacement;

        std::vector<bool> frame_stencil_visibility;

        if (m_resultordner == "/ground_truth") {
            frame_next_pts_array.clear();
            displacement_array.clear();
        }

        float columnBegin = m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                (sensor_index).at(current_frame_index).m_region_of_interest_px.x;
        float rowBegin = m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                (sensor_index).at(current_frame_index).m_region_of_interest_px.y;
        int width = cvRound(
                m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(
                        current_frame_index).m_region_of_interest_px.width_px);
        int height = cvRound(
                m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(
                        current_frame_index).m_region_of_interest_px.height_px);
        bool visibility = m_ptr_list_gt_objects.at(obj_index)->get_object_extrapolated_visibility().at(
                sensor_index).at(current_frame_index);

        cv::Mat flowFrame;
        flowFrame.create(Dataset::getFrameSize(), CV_32FC3);
        flowFrame = cv::Scalar_<unsigned>(0, 0, 0);
        assert(flowFrame.channels() == 3);

        if (visibility) {

            // 1st method
            cv::Mat roi = flowFrame.
                    rowRange(cvRound(rowBegin - (DO_STENCIL_GRID_EXTENSION * STENCIL_GRID_EXTENDER)),
                             (cvRound(rowBegin + height + (DO_STENCIL_GRID_EXTENSION * STENCIL_GRID_EXTENDER)))).
                    colRange(cvRound(columnBegin - (DO_STENCIL_GRID_EXTENSION * STENCIL_GRID_EXTENDER)),
                             (cvRound(columnBegin + width + (DO_STENCIL_GRID_EXTENSION * STENCIL_GRID_EXTENDER))));

            cv::Size roi_size;
            cv::Point roi_offset;
            roi.locateROI(roi_size, roi_offset);

            cv::Point2f gt_displacement = m_ptr_list_gt_objects.at(
                    obj_index)->get_object_extrapolated_point_displacement().at(sensor_index).at(
                    current_frame_index).second;

            if (m_resultordner == "/ground_truth") {
                // gt_displacement
                roi = cv::Scalar(gt_displacement.x, gt_displacement.y, static_cast<float>(1.0f));

                for (unsigned j = 0; j < width; j += 1) {
                    for (unsigned k = 0; k < height; k += 1) {

                        frame_next_pts_array.push_back(cv::Point2f(columnBegin + j, rowBegin + k));
                        displacement_array.push_back(gt_displacement);

                        frame_stencil_displacement.push_back(
                                std::make_pair(cv::Point2f(columnBegin + j, rowBegin + k), gt_displacement));
                        frame_stencil_visibility.push_back(visibility);

                    }
                }
            } else {

                if (m_weather == "blue_sky"  || m_weather == "heavy_snow") {

                    assert(m_ptr_list_simulated_objects.size() == m_ptr_list_gt_objects.size());

                    std::cout << "making a stencil on the basis of groundtruth object "
                              << m_ptr_list_gt_objects.at(obj_index)->getObjectId() << std::endl;

                    //std::cout << next_pts_array << std::endl;

                    std::string output_image_file_with_path = m_gnuplots_path.string() + sensor_index_folder_suffix + "/" + file_name_image_output;

                    for (unsigned row_index = 0; row_index < roi.rows; row_index++) {
                        for (unsigned col_index = 0; col_index < roi.cols; col_index++) {

                            for (ushort next_pts_index = 0;
                                 next_pts_index < frame_next_pts_array.size(); next_pts_index++) {
                                if (((roi_offset.x + col_index) ==
                                     std::round(frame_next_pts_array.at(next_pts_index).x)) &&
                                    ((roi_offset.y + row_index) ==
                                     std::round(frame_next_pts_array.at(next_pts_index).y))) {

                                    cv::Point2f algo_displacement = displacement_array.at(next_pts_index);

                                    frame_stencil_displacement.push_back(std::make_pair(
                                            cv::Point2f(roi_offset.x + col_index, roi_offset.y + row_index),
                                            algo_displacement));
                                    frame_stencil_visibility.push_back(visibility);

                                }
                            }
                        }
                    }


                } else {

                    std::cout << "making a stencil on the basis of base algorithm object "
                              << m_ptr_list_simulated_objects_base.at(obj_index)->getObjectId() << std::endl;
                    assert(m_ptr_list_simulated_objects.size() == m_ptr_list_simulated_objects_base.size());

                    auto COUNT = m_ptr_list_simulated_objects_base.at(
                            obj_index)->get_object_stencil_point_displacement().at
                            (sensor_index).at(current_frame_index).size();
                    for (auto count = 0; count < COUNT; count++) {

                        float x = m_ptr_list_simulated_objects_base.at(
                                obj_index)->get_object_stencil_point_displacement().at
                                (sensor_index).at(current_frame_index).at(count).first.x;
                        float y = m_ptr_list_simulated_objects_base.at(
                                obj_index)->get_object_stencil_point_displacement().at
                                (sensor_index).at(current_frame_index).at(count).first.y;

                        for (auto next_pts_index = 0;
                             next_pts_index < frame_next_pts_array.size(); next_pts_index++) {
                            if (((x) == frame_next_pts_array.at(next_pts_index).x) &&
                                ((y) == frame_next_pts_array.at(next_pts_index).y)) {
                                cv::Point2f algo_displacement = flowFrame.at<cv::Vec2f>(y, x);
                                frame_stencil_displacement.push_back(
                                        std::make_pair(cv::Point2f(x, y), algo_displacement));
                            }
                        }
                    }
                }
            }

            std::cout << "stencil size = " << frame_stencil_displacement.size() << " " << frame_next_pts_array.size()
                      << std::endl;

            //assert(frame_stencil_displacement.size() != 0);

            // TODO scratch : if frame_stencil_displacement does not work

        } else {

            frame_stencil_displacement.push_back(std::make_pair(cv::Point2f(0, 0), cv::Point2f(0, 0)));
            frame_stencil_visibility.push_back(false);


        }

        multiframe_stencil_displacement.at(obj_index).push_back(frame_stencil_displacement);
        multiframe_stencil_visibility.at(obj_index).push_back(frame_stencil_visibility);

    }

}


void OpticalFlow::generate_displacement_vector(ushort SENSOR_COUNT) {

    std::cout << "ground truth flow will be stored in " << m_generatepath << std::endl;

    char sensor_index_folder_suffix[50];

    for (ushort sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

        std::cout << "generate_object_stencil_point_displacement_pixel_visibility for sensor_index " << sensor_index
                  << std::endl;


        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > multiframe_stencil_displacement(
                m_ptr_list_gt_objects.size());
        std::vector<std::vector<std::vector<bool> > > multiframe_visibility(m_ptr_list_gt_objects.size());

        unsigned FRAME_COUNT = (unsigned) m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(
                sensor_index).size();
        assert(FRAME_COUNT > 0);


        sprintf(sensor_index_folder_suffix, "%02d", sensor_index);
        std::cout << "saving algorithm flow files in flow/ for sensor_index  " << sensor_index << std::endl;

        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

            std::cout << "current_frame_index " << current_frame_index << std::endl;


            std::vector<cv::Point2f> frame_next_pts_array, displacement_array;

            common_flow_frame(sensor_index, current_frame_index, frame_next_pts_array, displacement_array,
                              multiframe_stencil_displacement, multiframe_visibility);

        }

        for (ushort obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++) {

            m_ptr_list_gt_objects.at(obj_index)->push_back_object_stencil_point_displacement_pixel_visibility(
                    multiframe_stencil_displacement.at(obj_index), multiframe_visibility.at(obj_index));
        }

        cv::destroyAllWindows();
    }

    std::cout << "end of saving ground truth flow files " << std::endl;

}

void OpticalFlow::generate_flow_frames(ushort SENSOR_COUNT) {

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the position in a png file

    std::vector<Objects *> list_of_current_objects;

    unsigned COUNT;
    if (m_opticalFlowName == "ground_truth") {
        COUNT = 1;
        list_of_current_objects = m_ptr_list_gt_objects;
    } else {
        COUNT = DATAFILTER_COUNT;
        list_of_current_objects = m_ptr_list_simulated_objects;
    }

    char sensor_index_folder_suffix[50];
    for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {


        unsigned FRAME_COUNT = (unsigned) m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(
                sensor_index).size();
        assert(FRAME_COUNT > 0);
        cv::Mat image_02_frame = cv::Mat::zeros(Dataset::getFrameSize(), CV_32FC3);
        sprintf(sensor_index_folder_suffix, "%02d", sensor_index);
        std::cout << "saving flow files in flow/ for sensor_index  " << sensor_index << std::endl;

        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

            char file_name_input_image[50];
            ushort evaluation_frame_index = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                    (0).at(current_frame_index).frame_no;

            sprintf(file_name_input_image, "000%03d_10.png", evaluation_frame_index);
            std::string flow_path = m_flow_occ_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
            std::string kitti_path = m_plots_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;

            FlowImageExtended F_png_write(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

            std::cout << "current_frame_index " << current_frame_index << std::endl;

            float max_magnitude = 0.0;

            for (auto obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++) {

                unsigned CLUSTER_COUNT = (unsigned) list_of_current_objects.at(
                        obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).size();

                for (auto cluster_index = 0; cluster_index < CLUSTER_COUNT; cluster_index++) {

                    cv::Point2f pts = list_of_current_objects.at(obj_index)->
                            get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).at(
                            cluster_index).first;

                    cv::Point2f displacement = list_of_current_objects.at(obj_index)->
                            get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).at(
                            cluster_index).second;

                    max_magnitude = std::max((float) cv::norm(displacement), max_magnitude);

                    F_png_write.setFlowU(pts.x, pts.y, displacement.x);
                    F_png_write.setFlowV(pts.x, pts.y, displacement.y);
                    F_png_write.setValid(pts.x, pts.y, true);
                }

                F_png_write.writeExtended(flow_path);
                F_png_write.writeColor(kitti_path, max_magnitude);

            }
        }
    }

    std::cout << "end of saving ground truth flow files " << std::endl;

}


void OpticalFlow::getCombination(const std::vector<Objects *> &m_list_objects,
                                 std::vector<std::pair<Objects *, Objects *> > &list_of_objects_combination) {
    std::vector<Objects *>::const_iterator objectIterator = m_list_objects.begin();
    std::vector<Objects *>::const_iterator objectIteratorNext;

    for (; objectIterator < m_list_objects.end(); objectIterator++) {
        for (objectIteratorNext = objectIterator + 1; objectIteratorNext < m_list_objects.end();
             objectIteratorNext++) {

            list_of_objects_combination.push_back(std::make_pair(((*objectIterator)),
                                                                 ((*objectIteratorNext))));
        }
    }
}


void OpticalFlow::generate_metrics_optical_flow_algorithm(ushort SENSOR_COUNT) {

    std::vector<Objects *> list_of_current_objects;

    const float DISTANCE_ERROR_TOLERANCE = 2;

    unsigned COUNT;
    if (m_opticalFlowName == "ground_truth") {
        COUNT = 1;
        list_of_current_objects = m_ptr_list_gt_objects;
    } else {
        COUNT = DATAFILTER_COUNT;
        list_of_current_objects = m_ptr_list_simulated_objects;
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
                ushort evaluation_frame_index = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                        (0).at(current_frame_index).frame_no;
                sprintf(file_name_image_output, "000%03d_10.png", evaluation_frame_index);
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

                    evaluationData.at(obj_index).current_frame_index = evaluation_frame_index;
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



void OpticalFlow::plot_stencil(ushort SENSOR_COUNT) {

    std::vector<Objects*> list_of_current_objects;

    unsigned COUNT;
    if ( m_opticalFlowName == "ground_truth") {
        COUNT = 1;
        list_of_current_objects = m_ptr_list_gt_objects;
    }
    else {
        COUNT = DATAFILTER_COUNT;
        list_of_current_objects = m_ptr_list_simulated_objects;
    }

    char sensor_index_folder_suffix[50];

    std::cout << "visualise stencil algorithm at " << m_generatepath.string() + "stencil/" << std::endl;


    cv::Mat image_data_and_shape;

    cv::Mat tempGroundTruthImage(Dataset::getFrameSize(), CV_8UC3);
    FlowImageExtended F_png_write;

    ushort datafilter_index = 0;

    for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

        tempGroundTruthImage = cv::Scalar::all(255);
        F_png_write = FlowImageExtended(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

        std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > sensor_frame_shape_points;
        std::map<std::pair<float, float>, int> scenario_displacement_occurence;

        sprintf(sensor_index_folder_suffix, "%02d", sensor_index);

        unsigned FRAME_COUNT = (unsigned) list_of_current_objects.at(0)
                ->get_object_stencil_point_displacement().at(sensor_index).size();

        assert(FRAME_COUNT > 0);

        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

            char file_name_image_output[50];
            std::string output_image_file_with_path;
            ushort evaluation_frame_index = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                    (0).at(current_frame_index).frame_no;
            sprintf(file_name_image_output, "000%03d_10.png", evaluation_frame_index);
            output_image_file_with_path = m_plots_path.string() + sensor_index_folder_suffix + "/" + file_name_image_output;

            //---------------------------------------------------------------------------------
            tempGroundTruthImage = cv::imread(output_image_file_with_path, cv::IMREAD_COLOR);
            if ( tempGroundTruthImage.data == NULL ) {
                std::cout << "no image found, exiting" << std::endl;
                throw;
            }

            std::cout << "current_frame_index " << current_frame_index << " for datafilter_index " << datafilter_index<< std::endl;

            for (ushort obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++) {

                if (list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(sensor_index).at(current_frame_index) ) {

                    cv::Point2f pts_gt = m_ptr_list_gt_objects.at(
                            obj_index)->get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index).at(sensor_index).at(current_frame_index).mean_pts;

                    cv::Point2f mean_displacement_gt = m_ptr_list_gt_objects.at(
                            obj_index)->get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index).at(sensor_index).at(current_frame_index).mean_displacement;

                    cv::Point2f next_pts = cv::Point2f(pts_gt.x + mean_displacement_gt.x*10, pts_gt.y + mean_displacement_gt.y*10);

                    cv::arrowedLine(tempGroundTruthImage, pts_gt, next_pts, cv::Scalar(0,0,255), 2);

                    if (m_opticalFlowName == "ground_truth") {


                    }
                    else {

                        cv::Point2f algo_pts = m_ptr_list_simulated_objects.at(
                                obj_index)->get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index).at(sensor_index).at(current_frame_index).mean_pts;

                        cv::Point2f algo_displacement = m_ptr_list_simulated_objects.at(
                                obj_index)->get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index).at(sensor_index).at(current_frame_index).mean_displacement;

                        cv::Point2f next_pts = cv::Point2f(algo_pts.x + algo_displacement.x*10, algo_pts.y + algo_displacement.y*10);

                        cv::arrowedLine(tempGroundTruthImage, algo_pts, next_pts, cv::Scalar(0, 255, 0), 2);

                    }
                }
            }
            cv::imshow("ww", tempGroundTruthImage);
            cv::waitKey(100);
            cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
        }
    }
}

