//
// Created by veikas on 06.02.18.
//

#include <map>
#include <chrono>
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>
#include "OpticalFlow.h"
#include "FlowImageExtended.h"

using namespace std::chrono;


void OpticalFlow::prepare_directories_common() {

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

        // post processing step
        boost::filesystem::path bbox_dir = m_generatepath.string() + "/stencil/";
        if (boost::filesystem::exists(m_generatepath)) {
            system(("rm -rf " + bbox_dir.string()).c_str());
        }
        boost::filesystem::create_directories(bbox_dir);
    }

    std::cout << "Ending Flow directories " << m_resultordner << std::endl;

}


void OpticalFlow::common_flow_frame(std::string sensor_index_folder_suffix, std::string file_name_input_image, ushort sensor_index, ushort frame_count, cv::Mat &flowFrame, std::vector<cv::Point2f> &next_pts_array, std::vector<cv::Point2f>  &displacement_array,FlowImageExtended &F_png_write, std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &multiframe_stencil_displacement, std::vector<std::vector<std::vector<bool> >  > &multiframe_visibility) {

    std::string flow_path = m_flow_occ_path.string() + sensor_index_folder_suffix + "/" +
                            file_name_input_image;
    std::string kitti_path = m_plots_path.string() + sensor_index_folder_suffix + "/" +
                             file_name_input_image;


    for (ushort obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++) {

        std::vector<std::pair<cv::Point2f, cv::Point2f> >  frame_stencil_displacement;
        std::vector<bool>  frame_visibility;

        if ( m_resultordner == "/ground_truth") {
            next_pts_array.clear();
            displacement_array.clear();
        }

        float columnBegin = m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                (sensor_index).at(frame_count).m_region_of_interest_px.x;
        float rowBegin = m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                (sensor_index).at(frame_count).m_region_of_interest_px.y;
        int width = cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(frame_count).m_region_of_interest_px.width_px);
        int height = cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(frame_count).m_region_of_interest_px.height_px);
        bool visibility = m_ptr_list_simulated_objects.at(obj_index)->get_object_extrapolated_visibility().at(sensor_index).at(frame_count);

        if ( visibility ) {

            // gt_displacement
            cv::Point2f gt_displacement = m_ptr_list_gt_objects.at(obj_index)->get_object_extrapolated_point_displacement().at(sensor_index).at(frame_count).second;

            float max_magnitude = std::max((float)cv::norm(gt_displacement), max_magnitude);

            // 1st method
            cv::Mat roi = flowFrame.
                    rowRange(cvRound(rowBegin-(DO_STENCIL_GRID_EXTENSION*STENCIL_GRID_EXTENDER)),
                             (cvRound(rowBegin+height+(DO_STENCIL_GRID_EXTENSION*STENCIL_GRID_EXTENDER)))).
                    colRange(cvRound(columnBegin-(DO_STENCIL_GRID_EXTENSION*STENCIL_GRID_EXTENDER)),
                             (cvRound(columnBegin+width+(DO_STENCIL_GRID_EXTENSION*STENCIL_GRID_EXTENDER))));

            roi = cv::Scalar(gt_displacement.x, gt_displacement.y, static_cast<float>(1.0f));

            cv::Size roi_size;
            cv::Point roi_offset;
            roi.locateROI(roi_size, roi_offset);

            if ( m_resultordner == "/ground_truth") {
                for (unsigned j = 0; j < width; j += 1) {
                    for (unsigned k = 0; k < height; k += 1) {

                        next_pts_array.push_back(cv::Point2f(columnBegin + j, rowBegin + k));
                        displacement_array.push_back(gt_displacement);

                        frame_stencil_displacement.push_back(
                                std::make_pair(cv::Point2f(columnBegin + j, rowBegin + k), gt_displacement));
                        frame_visibility.push_back(visibility);

                    }
                }
            }

            else {
                // 2nd method
                // TODO scratch : This is for the base model for algorithms
                for (unsigned row_index = 0; row_index < roi.rows; row_index++) {
                    for (unsigned col_index = 0; col_index < roi.cols; col_index++) {

                        cv::Point2f algo_displacement = roi.at<cv::Vec2f>(row_index, col_index);
                        for ( auto next_pts_index = 0; next_pts_index < next_pts_array.size(); next_pts_index++ ) {
                            if ( (( roi_offset.x + col_index ) == next_pts_array.at(next_pts_index).x) &&
                                 (( roi_offset.y + row_index ) == next_pts_array.at(next_pts_index).y)) {
                                frame_stencil_displacement.push_back(
                                        std::make_pair(cv::Point2f(roi_offset.x + col_index, roi_offset.y + row_index), algo_displacement));
                                frame_visibility.push_back(visibility);

                            }
                        }
                    }
                }
            }

            if ( m_resultordner == "/ground_truth") {

                std::vector<cv::Point2f>::iterator it, it2 ;

                for ( it = next_pts_array.begin(), it2 = displacement_array.begin(); it !=next_pts_array.end(); it++, it2++ ) {

                    F_png_write.setFlowU((*it).x,(*it).y,(*it2).x);
                    F_png_write.setFlowV((*it).x,(*it).y,(*it2).y);
                    F_png_write.setValid((*it).x,(*it).y,true);
                }
            }

            std::cout << "stencil size = " << frame_stencil_displacement.size() << " " << next_pts_array.size() << std::endl;
            assert(frame_stencil_displacement.size() != 0);

            // TODO scratch : if frame_stencil_displacement does not work


        }

        else {

            frame_stencil_displacement.push_back(std::make_pair(cv::Point2f(0, 0),cv::Point2f(0, 0)));
            frame_visibility.push_back(false);


        }

        multiframe_stencil_displacement.at(obj_index).push_back(frame_stencil_displacement);
        multiframe_visibility.at(obj_index).push_back(frame_visibility);

    }
    std::vector<cv::Point2f>::iterator it, it2 ;

    for ( it = next_pts_array.begin(), it2 = displacement_array.begin(); it !=next_pts_array.end(); it++, it2++ ) {

        F_png_write.setFlowU((*it).x,(*it).y,(*it2).x);
        F_png_write.setFlowV((*it).x,(*it).y,(*it2).y);
        F_png_write.setValid((*it).x,(*it).y,true);
    }

    F_png_write.writeExtended(flow_path);
    F_png_write.writeColor(kitti_path, 5);

}



void OpticalFlow::save_flow_frame_from_displacement() {

    std::cout << "ground truth flow will be stored in " << m_generatepath << std::endl;

    //ptr_list_of_gt_objects.at(obj_index)->generate_object_stencil_point_displacement_pixel_visibility("ground_truth");


    char sensor_index_folder_suffix[50];

    for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

        std::cout << "generate_object_stencil_point_displacement_pixel_visibility for sensor_index " << sensor_index << std::endl;


        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > multiframe_stencil_displacement(m_ptr_list_gt_objects.size());
        std::vector<std::vector<std::vector<bool> >  > multiframe_visibility(m_ptr_list_gt_objects.size());

        unsigned FRAME_COUNT = (unsigned)m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(sensor_index).size();
        assert(FRAME_COUNT>0);


        sprintf(sensor_index_folder_suffix, "%02d", sensor_index);
        std::cout << "saving algorithm flow files in flow/ for sensor_index  " << sensor_index << std::endl;

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

            char file_name_input_image[50];
            std::cout << "frame_count " << frame_count << std::endl;
            sprintf(file_name_input_image, "000%03d_10.png", frame_count);
            std::string input_image_path = m_GroundTruthImageLocation.string() + "_" + std::to_string(sensor_index) + "/" + file_name_input_image;

            std::string flow_path = m_flow_occ_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
            std::string kitti_path = m_plots_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;

            FlowImageExtended F_png_write( Dataset::getFrameSize().width, Dataset::getFrameSize().height);

            float max_magnitude = 0;
            cv::Mat flowFrame;
            flowFrame.create(Dataset::getFrameSize(), CV_32FC3);
            flowFrame = cv::Scalar_<unsigned>(0,0,0);
            assert(flowFrame.channels() == 3);

            std::vector<cv::Point2f> next_pts_array, displacement_array;


            common_flow_frame(sensor_index_folder_suffix, file_name_input_image, sensor_index, frame_count, flowFrame, next_pts_array, displacement_array, F_png_write, multiframe_stencil_displacement, multiframe_visibility);

        }

        for ( ushort obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++) {

            m_ptr_list_simulated_objects.at(obj_index)->set_object_stencil_point_displacement_pixel_visibility("ground_truth", multiframe_stencil_displacement.at(obj_index), multiframe_visibility.at(obj_index));
        }

        cv::destroyAllWindows();
    }
    std::cout << "end of saving ground truth flow files " << std::endl;

}

void OpticalFlow::getCombination( const std::vector<Objects *> &m_list_objects, std::vector<std::pair<Objects*, Objects* > > &list_of_objects_combination) {
    std::vector<Objects*>::const_iterator objectIterator = m_list_objects.begin();
    std::vector<Objects*>::const_iterator  objectIteratorNext;

    for ( ; objectIterator < m_list_objects.end() ; objectIterator++ ) {
        for ( objectIteratorNext = objectIterator+1; objectIteratorNext < m_list_objects.end();
                objectIteratorNext++) {

            list_of_objects_combination.push_back(std::make_pair(((*objectIterator)),
                    ((*objectIteratorNext))));
        }
    }
}



void OpticalFlow::generate_metrics_optical_flow_algorithm() {

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

    std::vector<std::vector< OPTICAL_FLOW_EVALUATION_METRICS > > sensor_multiframe_evaluation_data;

    for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

        sprintf(sensor_index_folder_suffix, "%02d", sensor_index);

        std::cout << "generating shape points in OpticalFlow.cpp for sensor index "  << sensor_index
                  << " for opticalflow  " << m_opticalFlowName << std::endl;

        unsigned FRAME_COUNT = (unsigned) list_of_current_objects.at(0)
                ->get_object_stencil_point_displacement().at(sensor_index).size();

        assert(FRAME_COUNT > 0);

        std::vector<OPTICAL_FLOW_EVALUATION_METRICS> multiframe_evaluation_data;

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

            OPTICAL_FLOW_EVALUATION_METRICS evaluationData;

            std::cout << "frame_count " << frame_count << " for opticalflow_index " << m_opticalFlowName << std::endl;


            for (ushort obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++) {

                if ( list_of_current_objects.at(obj_index)->getObjectName() == "New Character" ||
                        list_of_current_objects.at(obj_index)->getObjectName() == "simulated_New Character") {
                    cv::Point2i dimension = {
                            cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(frame_count).m_region_of_interest_px.width_px),
                            cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(frame_count).m_region_of_interest_px.height_px)
                    };

                    // displacements found by the algorithm for this object
                    unsigned CLUSTER_COUNT = (unsigned)list_of_current_objects.at(
                            obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(frame_count).size();

                    evaluationData.frame_count = frame_count;

                    if (list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(sensor_index).at(frame_count) ) {

                        // Instances of CLUSTER_COUNT_ALGO in CLUSTER_COUNT_GT

                        cv::Point2f gt_displacement = m_ptr_list_gt_objects.at(obj_index)->get_object_extrapolated_point_displacement().at
                                (sensor_index).at(frame_count).second;

                        auto dist_gt = cv::norm(gt_displacement);
                        auto angle_gt = std::tanh(gt_displacement.y / gt_displacement.x);

                        if (m_opticalFlowName == "ground_truth") {

                            evaluationData.visiblePixels = dimension.x * dimension.y; // how many pixels are visible ( it could be that some pixels are occluded )
                            evaluationData.goodPixels = CLUSTER_COUNT; // how many pixels in the found pixel are actually valid

                        }
                        else {

                            evaluationData.visiblePixels = CLUSTER_COUNT / mStepSize;
                            // how many pixelsi are visible ( it could be that some pixels are occluded ). This wll be found out using k-means
                            evaluationData.goodPixels = 0; // how many pixels in the found pixel are actually valid

                            for (auto cluster_index = 0; cluster_index < CLUSTER_COUNT; cluster_index++) {

                                cv::Point2f algo_displacement = list_of_current_objects.at(obj_index)->
                                        get_object_stencil_point_displacement().at(sensor_index).at(frame_count).at(cluster_index).second;

                                auto dist_algo = cv::norm(algo_displacement);
                                auto dist_err = std::abs(dist_gt - dist_algo);

                                auto angle_algo = std::tanh(algo_displacement.y/algo_displacement.x);

                                auto angle_err = std::abs(angle_algo - angle_gt);
                                auto angle_err_dot = std::cosh(
                                        algo_displacement.dot(gt_displacement) / (dist_gt * dist_algo));

                                //assert(angle_err_dot==angle_err);
                                if (
                                        (dist_err) < DISTANCE_ERROR_TOLERANCE &&
                                        (angle_err*180/CV_PI) < ANGLE_ERROR_TOLERANCE

                                        ) {
                                    evaluationData.goodPixels++; // how many pixels in the found pixel are actually valid
                                }
                            }
                        }

                        multiframe_evaluation_data.push_back(evaluationData);

                        std::cout << "vollTreffer for object " << list_of_current_objects.at(obj_index)->getObjectName() << " = "
                                  << evaluationData.goodPixels<< std::endl;
                        std::cout << "baseTreffer for object " << list_of_current_objects.at(obj_index)->getObjectName() << " = "
                                  << evaluationData.visiblePixels << std::endl;

                        //assert(evaluationData.goodPixels <= std::ceil(evaluationData.algorithmPixels) + 20 );

                    } else {
                        std::cout << "visibility of object " << list_of_current_objects.at(obj_index)->getObjectName() << " = " <<
                                  list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(sensor_index)
                                          .at(frame_count)
                                  << " and hence not generating any shape points for this object " << std::endl;

                        multiframe_evaluation_data.push_back(evaluationData);

                    }
                }
            }
        }
        sensor_multiframe_evaluation_data.push_back(multiframe_evaluation_data);
    }
    m_sensor_multiframe_evaluation_data = sensor_multiframe_evaluation_data;
}


/*

void OpticalFlow::visualiseStencilAlgorithms() {

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

    char file_name_image[50], file_name_image_output[50];

    cv::Mat image_data_and_shape;

    cv::Mat tempGroundTruthImage(Dataset::getFrameSize(), CV_8UC3);
    FlowImageExtended F_png_write;

    ushort datafilter_index = 0;


    std::vector<std::map<std::pair<float, float>, int> > sensor_scenario_displacement_occurence;
    std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > > sensor_shape_points;

    for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

        tempGroundTruthImage = cv::Scalar::all(255);
        F_png_write = FlowImageExtended(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

        std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > sensor_frame_shape_points;
        std::map<std::pair<float, float>, int> scenario_displacement_occurence;

        sprintf(sensor_index_folder_suffix, "%02d", sensor_index);

        std::cout << "generating kitti flow in OpticalFlow.cpp for " << m_resultordner << " " << sensor_index
                  << " for datafilter " << datafilter_index << std::endl;

        unsigned FRAME_COUNT = (unsigned) list_of_current_objects.at(0)
                ->get_object_stencil_point_displacement().at(sensor_index).size();

        assert(FRAME_COUNT > 0);

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

            std::string output_image_file_with_path;

            sprintf(file_name_image_output, "000%03d_10.png", frame_count);

            output_image_file_with_path = m_plots_path.string() + sensor_index_folder_suffix + "/" + file_name_image_output;


            //---------------------------------------------------------------------------------
            tempGroundTruthImage = cv::Scalar::all(0);


            std::cout << "frame_count " << frame_count << " for datafilter_index " << datafilter_index<< std::endl;

            std::vector<std::pair<cv::Point2i, cv::Point2f>> frame_shape_points;

            for (ushort obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++) {

                auto CLUSTER_COUNT = list_of_current_objects.at(
                        obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(frame_count).size();

                if (list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(sensor_index).at(frame_count) ) {

                    // Instances of CLUSTER_COUNT_ALGO in CLUSTER_COUNT_GT


                    cv::Point2f pts_mean = m_ptr_list_gt_objects.at(obj_index)->get_list_object_mean_centroid_displacement().at(0).
                            at(sensor_index).at(frame_count).first;
                    cv::Point2f pts_basic = m_ptr_list_gt_objects.at(obj_index)->get_object_extrapolated_point_displacement().at(
                            sensor_index).at(frame_count).first;

                    cv::Point2f displacement = m_ptr_list_gt_objects.at(obj_index)->get_list_object_mean_centroid_displacement().at(sensor_index).at(
                            0).at(frame_count).second;

                    cv::Point2f next_pts = cv::Point2f(pts_basic.x + displacement.x*10, pts_basic.y + displacement.y*10);

                    cv::arrowedLine(tempGroundTruthImage, pts_basic, next_pts, cv::Scalar(0,0,255), 2);



                    if (m_opticalFlowName == "ground_truth") {

                        vollTreffer = CLUSTER_COUNT_GT;
                        // this is the full resolution ! Because there is no stepSize in GroundTruth
                        baseTreffer = CLUSTER_COUNT_GT;

                    }
                    else {
                        for (auto cluster_index = 0; cluster_index < CLUSTER_COUNT; cluster_index++) {


                            cv::Point2f pts_base = m_ptr_list_simulated_objects.at(
                                    obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(
                                    sensor_index).at(frame_count).at(cluster_index).first;

                            cv::Point2f displacement_base = m_ptr_list_simulated_objects.at(
                                    obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(
                                    sensor_index).at(frame_count).at(cluster_index).second;

                            cv::Point2f next_pts_base = cv::Point2f(pts_base.x + displacement_base.x, pts_base.y + displacement_base.y);

                            cv::Point2f algo_pts = m_ptr_list_simulated_objects.at(
                                            obj_index)->get_object_stencil_point_displacement().at(
                                            sensor_index)
                                    .at(frame_count).at(cluster_index).first;

                            cv::Point2f algo_displacement = m_ptr_list_simulated_objects.at(
                                            obj_index)->get_object_stencil_point_displacement().at(
                                            sensor_index)
                                    .at(frame_count).at(cluster_index).second;


                            cv::Point2f next_pts = cv::Point2f(algo_pts.x + algo_displacement.x, algo_pts.y + algo_displacement.y);

                            cv::arrowedLine(tempGroundTruthImage, algo_pts, next_pts, cv::Scalar(0, 255, 0), 1, 8, 0, 0.25);

                            auto dist_algo = cv::norm(algo_displacement);
                            auto dist_err = std::abs(dist_gt - dist_algo);

                            auto angle_algo = std::tanh(algo_displacement.y/algo_displacement.x);

                            auto angle_err = std::abs(angle_algo - angle_gt);
                            auto angle_err_dot = std::cosh(
                                    algo_displacement.dot(gt_displacement) / (dist_gt * dist_algo));

                            cv::Point2f pts = m_ptr_list_simulated_objects_base.at(
                                            obj_index)->get_object_stencil_point_displacement().at(
                                            sensor_index)
                                    .at(frame_count).at(cluster_index).first;

                            cv::circle(tempGroundTruthImage, pts, 1.5, cv::Scalar(255, 255, 255), 1, 8);

                            //assert(angle_err_dot==angle_err);

                            if (
                                    (dist_err) < DISTANCE_ERROR_TOLERANCE &&
                                    (angle_err*180/CV_PI) < ANGLE_ERROR_TOLERANCE

                                    ) {
                                vollTreffer++;
                                F_png_write.setFlowU(algo_pts.x, algo_pts.y, algo_displacement.x);
                                F_png_write.setFlowV(algo_pts.x, algo_pts.y, algo_displacement.y);
                                F_png_write.setValid(algo_pts.x, algo_pts.y, true);
                            }
                            else {

                                F_png_write.setFlowU(algo_pts.x, algo_pts.y, algo_displacement.x);
                                F_png_write.setFlowV(algo_pts.x, algo_pts.y, algo_displacement.y);
                                F_png_write.setValid(algo_pts.x, algo_pts.y, false);

                            }

                        }

                        baseTreffer = ((float) CLUSTER_COUNT_GT) / mStepSize;
                    }
                    frame_shape_points.push_back(std::make_pair(cv::Point2i(frame_count, 0), cv::Point2f(vollTreffer, baseTreffer)));

                    std::cout << "vollTreffer for object " << list_of_current_objects.at(obj_index)->getObjectId() << " = "
                              << vollTreffer << std::endl;
                    std::cout << "baseTreffer for object " << list_of_current_objects.at(obj_index)->getObjectId() << " = "
                              << baseTreffer << std::endl;

                    assert(vollTreffer <= std::ceil(baseTreffer) + 20 );

                } else {
                    std::cout << "visibility of object " << list_of_current_objects.at(obj_index)->getObjectId() << " = " <<
                              list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(sensor_index)
                                      .at(frame_count)
                              << " and hence not generating any shape points for this object " << std::endl;

                    frame_shape_points.push_back(std::make_pair(cv::Point2i(frame_count,0), cv::Point2f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));

                }
            }

            sensor_frame_shape_points.push_back(frame_shape_points);
        }
        sensor_shape_points.push_back(sensor_frame_shape_points);
        sensor_scenario_displacement_occurence.push_back(scenario_displacement_occurence);

        // generate for every algorithm, an extra sensor
        if ( sensor_index == (SENSOR_COUNT-1) ) {
            generate_shape_points_sensor_fusion(datafilter_index, sensor_shape_points );
        }

    }

    m_sensor_shape_points.push_back(sensor_shape_points);
    m_sensor_scenario_displacement_occurence = sensor_scenario_displacement_occurence;

    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
}

*/