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


void OpticalFlow::prepare_directories() {

    char char_dir_append[20];

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
                            cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(frame_count).m_object_dimensions_px.dim_width_m),
                            cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(frame_count).m_object_dimensions_px.dim_height_m)
                    };

                    // displacements found by the algorithm for this object
                    unsigned CLUSTER_COUNT = (unsigned)list_of_current_objects.at(
                            obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(frame_count).size();

                    evaluationData.frame_count = frame_count;
                    evaluationData.realClusterSize = dimension.x * dimension.y;

                    if (list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(sensor_index).at(frame_count) ) {

                        // Instances of CLUSTER_COUNT_ALGO in CLUSTER_COUNT_GT

                        cv::Point2f gt_displacement = m_ptr_list_gt_objects.at(obj_index)->get_object_pixel_position_pixel_displacement().at
                                (sensor_index).at(frame_count).second;

                        auto dist_gt = cv::norm(gt_displacement);
                        auto angle_gt = std::tanh(gt_displacement.y / gt_displacement.x);

                        if (m_opticalFlowName == "ground_truth") {

                            evaluationData.visibleClusterSize = CLUSTER_COUNT; // how many pixels are visible ( it could be that some pixels are occluded )
                            evaluationData.algorithmClusterSize = CLUSTER_COUNT; // what does the algorithm find?
                            evaluationData.goodPixels = CLUSTER_COUNT; // how many pixels in the found pixel are actually valid

                        }
                        else {

                            evaluationData.visibleClusterSize = CLUSTER_COUNT; // how many pixels are visible ( it could be that some pixels are occluded )
                            evaluationData.algorithmClusterSize = CLUSTER_COUNT / mStepSize;
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

                        std::cout << "vollTreffer for object " << list_of_current_objects.at(obj_index)->getObjectId() << " = "
                                  << evaluationData.goodPixels<< std::endl;
                        std::cout << "baseTreffer for object " << list_of_current_objects.at(obj_index)->getObjectId() << " = "
                                  << evaluationData.algorithmClusterSize << std::endl;

                        //assert(evaluationData.goodPixels <= std::ceil(evaluationData.algorithmClusterSize) + 20 );

                    } else {
                        std::cout << "visibility of object " << list_of_current_objects.at(obj_index)->getObjectId() << " = " <<
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


