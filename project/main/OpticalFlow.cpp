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
#include "SortandIntersect.h"
#include "InterpolateData.h"
#include "GroundTruthScene.h"
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

        sprintf(char_dir_append, "%02d", m_evaluation_list.at(i));

        m_collision_object_path = m_generatepath.string() + "/collision_object_";
        path =  m_collision_object_path.string() + char_dir_append;
        boost::filesystem::create_directories(path);

        m_flow_occ_path = m_generatepath.string() + "/flow_occ_";
        path =  m_flow_occ_path.string() + char_dir_append;
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

void OpticalFlow::common_flow_frame(ushort sensor_index, ushort current_frame_index, const std::vector<cv::Point2f> &frame_next_pts_array, const std::vector<cv::Point2f>  &displacement_array,std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &multiframe_stencil_displacement, std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &multiframe_stencil_disjoint_displacement, std::vector<std::vector<std::vector<bool> >  > &multiframe_stencil_visibility,
                                    std::vector<cv::Point2f> all_moving_objects_in_frame) {

    char sensor_index_folder_suffix[50];
    sprintf(sensor_index_folder_suffix, "%02d", sensor_index);

    ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
            (0).at(current_frame_index).frame_no;

    // DEPTH READ
    cv::Mat finalDepth(Dataset::m_frame_size,CV_32FC1);
    char file_name_input_image_depth[50];
    sprintf(file_name_input_image_depth, "depth_000%03d_10.png", image_frame_count);
    std::string input_image_path_depth = m_GroundTruthImageLocation.string() + "_" + sensor_index_folder_suffix + "/" + file_name_input_image_depth;
    cv::Mat depth_02_frame = cv::imread(input_image_path_depth, CV_LOAD_IMAGE_UNCHANGED);

    auto START_BENCHMARK
    for (ushort obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++) {

        std::vector<std::pair<cv::Point2f, cv::Point2f> > object_stencil_displacement;
        std::vector<std::pair<cv::Point2f, cv::Point2f> > frame_stencil_disjoint_displacement;
        std::vector<bool> frame_stencil_visibility;

        frame_stencil_displacement_region_of_interest_method(sensor_index, current_frame_index, frame_next_pts_array, displacement_array, obj_index, object_stencil_displacement, frame_stencil_disjoint_displacement,frame_stencil_visibility,  all_moving_objects_in_frame, depth_02_frame);

        multiframe_stencil_displacement.at(obj_index).push_back(object_stencil_displacement);
        multiframe_stencil_disjoint_displacement.at(obj_index).push_back(frame_stencil_disjoint_displacement);
        multiframe_stencil_visibility.at(obj_index).push_back(frame_stencil_visibility);

    }
    PRINT_BENCHMARK(frame_stencil_displacement_time_required)
}

void OpticalFlow::frame_stencil_displacement_region_of_interest_method(ushort sensor_index, ushort current_frame_index, const std::vector<cv::Point2f> &frame_next_pts_array, const std::vector<cv::Point2f>  &displacement_array, ushort obj_index, std::vector<std::pair<cv::Point2f, cv::Point2f> > &object_stencil_displacement, std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_stencil_disjoint_displacement, std::vector<bool> &frame_stencil_visibility, const std::vector<cv::Point2f>& all_moving_objects_in_frame, const cv::Mat& depth_02_frame) {

    std::vector<cv::Point2f> gt_frame_stencil_displacement_from_roi;
    std::vector<cv::Point2f> gt_frame_stencil_displacement_from_depth;

    float columnBegin = (unsigned)m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
            (sensor_index).at(current_frame_index).m_region_of_interest_px.x;
    float rowBegin = (unsigned)m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
            (sensor_index).at(current_frame_index).m_region_of_interest_px.y;
    int width = cvRound(
            m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(
                    current_frame_index).m_region_of_interest_px.width_px);
    int height = cvRound(
            m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(
                    current_frame_index).m_region_of_interest_px.height_px);
    bool visibility = m_ptr_list_gt_objects.at(obj_index)->get_object_extrapolated_visibility().at(
            sensor_index).at(current_frame_index);

    if (visibility) {

        if ( m_resultordner == "/ground_truth" ) {
            // Squared region of Interest
            std::vector<cv::Point2f> squared_region_of_interest;
            for (unsigned j = 0; j < width; j += 1) {
                for (unsigned k = 0; k < height; k += 1) {
                    squared_region_of_interest.push_back(
                            cv::Point2f(columnBegin + j, rowBegin + k));
                }
            }

            // 1st step - Intersection of Squared ROI and Frame Differencing
            // ---------------------------------------------------------------------------------------------------------
            assert(squared_region_of_interest.size() > 0);
            gt_frame_stencil_displacement_from_roi.resize(squared_region_of_interest.size());
            gt_frame_stencil_displacement_from_roi.clear();

            MyIntersection myIntersection_gt_roi_objects;
            std::vector<cv::Point2f>::iterator result_it;
            result_it = myIntersection_gt_roi_objects.find_intersection(squared_region_of_interest.begin(), squared_region_of_interest.end(),
                                                         all_moving_objects_in_frame.begin(), all_moving_objects_in_frame.end(),
                                                                        gt_frame_stencil_displacement_from_roi.begin());
            gt_frame_stencil_displacement_from_roi = myIntersection_gt_roi_objects.getResult();
            assert(gt_frame_stencil_displacement_from_roi.size() > 0);

            auto size_stencil_from_roi = gt_frame_stencil_displacement_from_roi.size();

            cv::Mat tempImage(Dataset::m_frame_size, CV_8UC3, cv::Scalar(255,255,255));
            float depth_value_object = m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(current_frame_index).m_object_distances.sensor_to_obj_usk;

            if ( m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(current_frame_index).object_name == "Pedesterian" ) {
                //depth_value_object++;
            }
            for ( auto it = gt_frame_stencil_displacement_from_roi.begin(); it != gt_frame_stencil_displacement_from_roi.end(); it++) {
                cv::circle(tempImage, (*it), 1, cv::Scalar(255,0,0));
            }
            //cv::imshow("roi", tempImage);
            //cv::waitKey(0);
            cv::destroyAllWindows();
            tempImage = cv::Scalar::all(255);
//---------------------------------------------------------------------------------------------------------
            // 2nd step - Refine intersection in case multiple objects are inside the ROI.
            for (unsigned j = 0; j < gt_frame_stencil_displacement_from_roi.size(); j += 1) {
                ushort val = depth_02_frame.at<unsigned char>(gt_frame_stencil_displacement_from_roi.at(j));
                //std::cout << (ushort)val << "*" << std::round(depth_value_object-3) << " ";
                bool found_correct_depth = (val == std::round(depth_value_object));
                if  ( Dataset::m_dataset_basepath.string() == VIRES_DATASET_PATH) {
                    found_correct_depth = (val == std::round(depth_value_object-3));
                }
                if ( found_correct_depth )  {
                    gt_frame_stencil_displacement_from_depth.push_back(gt_frame_stencil_displacement_from_roi.at(j));
                }
            }

            assert(gt_frame_stencil_displacement_from_depth.size()>0);

            auto size_stencil_from_depth = gt_frame_stencil_displacement_from_depth.size();

            for ( auto it = gt_frame_stencil_displacement_from_depth.begin(); it != gt_frame_stencil_displacement_from_depth.end(); it++) {
                cv::circle(tempImage, (*it), 1, cv::Scalar(255,0,0));
            }
            //cv::imshow("temp", tempImage);
            //cv::waitKey(0);
            cv::destroyAllWindows();
            cv::Point2f gt_displacement = m_ptr_list_gt_objects.at(
                                obj_index)->get_object_extrapolated_point_displacement().at(sensor_index).at(
                                current_frame_index).second;


            for ( auto it = gt_frame_stencil_displacement_from_depth.begin(); it!=gt_frame_stencil_displacement_from_depth.end(); it++) {
                object_stencil_displacement.push_back(std::make_pair((*it), gt_displacement));
            }
            //object_stencil_displacement = gt_frame_stencil_displacement_from_depth;
            assert(object_stencil_displacement.size()>0);

            if ( cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(current_frame_index).m_object_location_camera_px.cog_px.x)  ) {
            }

            // std::sort(object_stencil_displacement.begin(), object_stencil_displacement.end(), PairPointsSort<float>());
            bool isSorted = std::is_sorted(object_stencil_displacement.begin(), object_stencil_displacement.end(), PairPointsSort<float>());
            frame_stencil_visibility.resize(object_stencil_displacement.size());
            std::fill(frame_stencil_visibility.begin(), frame_stencil_visibility.end(), (bool)1);

        } else {

            if (m_weather == "blue_sky"  || m_weather == "heavy_snow") {

                // Intersection between ground truth stencil and the algorithm stencil.

                cv::Mat tempImage(Dataset::m_frame_size, CV_8UC3, cv::Scalar(255,255,255));

                object_stencil_displacement.resize(frame_next_pts_array.size());
                frame_stencil_visibility.resize(frame_next_pts_array.size());

                object_stencil_displacement.clear();
                frame_stencil_visibility.clear();

                assert(m_ptr_list_simulated_objects.size() == m_ptr_list_gt_objects.size());
                std::cout << "making a stencil on the basis of groundtruth object "
                          << m_ptr_list_gt_objects.at(obj_index)->getObjectId() << std::endl;

                //std::cout << next_pts_array << std::endl;
                // benchmark this and then use intersection

                std::vector<std::pair<cv::Point2f,cv::Point2f > > entire_frame_algorithm_result_pts_displacement;
                for (ushort next_pts_index = 0; next_pts_index < frame_next_pts_array.size(); next_pts_index++) {
                    entire_frame_algorithm_result_pts_displacement.push_back(std::make_pair(cv::Point2f(std::trunc(frame_next_pts_array.at(next_pts_index).x), std::trunc(frame_next_pts_array.at(next_pts_index).y)), displacement_array.at(next_pts_index)));
                }

                std::sort(entire_frame_algorithm_result_pts_displacement.begin(), entire_frame_algorithm_result_pts_displacement.end(), PairPointsSort<float>());
                bool isSorted = std::is_sorted(entire_frame_algorithm_result_pts_displacement.begin(), entire_frame_algorithm_result_pts_displacement.end(), PairPointsSort<float>());

                //327, 250 until 327, 278 in current_frame_index = 1, starts from index 69
                //327, 250 until 327, 286 in current_frame_index = 1, starts from index 52 . ground truth

                MyIntersection myIntersection;
                std::vector<std::pair<cv::Point2f, cv::Point2f> >::iterator result_it;

                // Look for only those pixels that lie within the ground truth stencil of this particular object
                result_it = myIntersection.find_intersection_pair(entire_frame_algorithm_result_pts_displacement.begin(), entire_frame_algorithm_result_pts_displacement.end(),
                        m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).begin(),
                        m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).end(),
                        object_stencil_displacement.begin());

                object_stencil_displacement = myIntersection.getResultIntersectingPair();
                frame_stencil_visibility.resize(object_stencil_displacement.size());
                std::fill(frame_stencil_visibility.begin(), frame_stencil_visibility.end(), (bool)1);

                for ( auto it = object_stencil_displacement.begin(); it != object_stencil_displacement.end(); it++) {
                    cv::circle(tempImage, (*it).first, 1, cv::Scalar(0,0,255));
                }


                // Look for only those pixels that does not lie within the ground truth stencil of this particular object
                MyIntersection myDisjoint;
                std::vector<std::pair<cv::Point2f, cv::Point2f> >::iterator result_disjoint_it;

                result_disjoint_it = myDisjoint.find_disjoint_pair(object_stencil_displacement.begin(), object_stencil_displacement.end(),
                                                                  m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).begin(),
                                                                  m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).end(),
                                                                  object_stencil_displacement.begin());

                frame_stencil_disjoint_displacement = myDisjoint.getResultDisjointPair();
                //frame_stencil_disjoint_visibility.resize(frame_stencil_disjoint_displacement.size());
                //std::fill(frame_stencil_disjoint_visibility.begin(), frame_stencil_disjoint_visibility.end(), (bool)1);

                for ( auto it = frame_stencil_disjoint_displacement.begin(); it != frame_stencil_disjoint_displacement.end(); it++) {
                    cv::circle(tempImage, (*it).first, 1, cv::Scalar(255,0,0));
                }
                //cv::imshow("disjoint", tempImage);
                //cv::waitKey(0);
                cv::destroyAllWindows();

                std::cout << "disjoint " << frame_stencil_disjoint_displacement.size() << " found " << object_stencil_displacement.size() << " total " << m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).size() << std::endl;

                //InterpolateData interpolateData;
                //interpolateData.interpolateBackground(object_stencil_displacement, frame_stencil_disjoint_displacement);


            } else {

                // this should contain the intersection between entire_frame_algorithm_result_pts_displacement and algorithm_result_pts_displacement
                // so we need to store the data.
                
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

                            cv::Point2f algo_displacement = displacement_array.at(next_pts_index);
                            object_stencil_displacement.push_back(
                                    std::make_pair(cv::Point2f(x, y), algo_displacement));
                        }
                    }
                }
            }
        }

        std::cout << "stencil size = " << object_stencil_displacement.size() << " " << frame_next_pts_array.size()
                  << std::endl;

        assert(object_stencil_displacement.size() != 0);
        // TODO scratch : if object_stencil_displacement does not work

    }

    else {

        object_stencil_displacement.push_back(std::make_pair(cv::Point2f(0, 0), cv::Point2f(0, 0)));
        frame_stencil_visibility.push_back(visibility);

    }
}

void OpticalFlow::save_flow_vector(ushort SENSOR_COUNT) {

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the position in a png file

    std::vector<Objects *> list_of_current_objects;

    if (m_opticalFlowName == "ground_truth") {
        for ( auto i = 0; i < m_ptr_list_gt_objects.size(); i++) {
            list_of_current_objects.push_back(static_cast<GroundTruthObjects*>(m_ptr_list_gt_objects.at(i)));
        }
    } else {
        list_of_current_objects = m_ptr_list_simulated_objects;
    }

    char sensor_index_folder_suffix[50];
    for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

        unsigned FRAME_COUNT = (unsigned) m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(
                sensor_index).size();
        assert(FRAME_COUNT > 0);
        cv::Mat image_02_frame = cv::Mat::zeros(Dataset::m_frame_size, CV_32FC3);
        sprintf(sensor_index_folder_suffix, "%02d", m_evaluation_list.at(sensor_index));
        std::cout << "saving flow files in flow/ for sensor_index  " << sensor_index_folder_suffix << std::endl;

        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

            char file_name_input_image[50];
            ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                    (0).at(current_frame_index).frame_no;

            sprintf(file_name_input_image, "000%03d_10.png", image_frame_count);

            std::string flow_path = m_flow_occ_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
            std::string plot_path = m_plots_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;

            if ( m_resultordner == "/ground_truth" ) {
                flow_path = GroundTruthScene::m_ground_truth_flow_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
                plot_path = GroundTruthScene::m_ground_truth_plot_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
            }

            FlowImageExtended F_png_write(Dataset::m_frame_size.width, Dataset::m_frame_size.height);

            std::cout << "current_frame_index " << current_frame_index << std::endl;

            float max_magnitude = 0.0;

            for (auto obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++) {

                unsigned CLUSTER_COUNT = (unsigned) list_of_current_objects.at(
                        obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).size();

                unsigned CLUSTER_COUNT_NO_DATA;

                if ( m_resultordner == "/ground_truth" )
                    CLUSTER_COUNT_NO_DATA = 0;
                else
                CLUSTER_COUNT_NO_DATA = (unsigned) list_of_current_objects.at(obj_index)->get_object_stencil_point_disjoint_displacement().at(sensor_index).at(current_frame_index).size();


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
                    F_png_write.setObjectId(pts.x, pts.y, (obj_index+1));
                }


                for (auto cluster_index = 0; cluster_index < CLUSTER_COUNT_NO_DATA; cluster_index++) {

                    cv::Point2f pts = list_of_current_objects.at(obj_index)->
                            get_object_stencil_point_disjoint_displacement().at(sensor_index).at(current_frame_index).at(
                            cluster_index).first;

                    cv::Point2f displacement = list_of_current_objects.at(obj_index)->
                            get_object_stencil_point_disjoint_displacement().at(sensor_index).at(current_frame_index).at(
                            cluster_index).second;

                    max_magnitude = std::max((float) cv::norm(displacement), max_magnitude);

                    displacement.x = 1;
                    displacement.y = 1;

                    F_png_write.setFlowU(pts.x, pts.y, displacement.x);
                    F_png_write.setFlowV(pts.x, pts.y, displacement.y);
                    F_png_write.setObjectId(pts.x, pts.y, -1);
                }


                F_png_write.interpolateBackground();
                F_png_write.write(flow_path);
                F_png_write.writeColor(plot_path, max_magnitude);

            }
        }
    }

    std::cout << "end of saving " + m_resultordner + " flow files in an image" << std::endl;

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

