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

void OpticalFlow::prepare_directories_common() {

    char char_dir_append[20];

    std::cout << "Creating Flow directories " << m_resultordner << std::endl;

    if (boost::filesystem::exists(m_generatepath)) {
        system(("rm -rf " + m_generatepath.string()).c_str());
    }
    boost::filesystem::create_directories(m_generatepath);

    boost::filesystem::path path;

    for (int i = 0; i < Dataset::SENSOR_COUNT; ++i) {

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


    bool visibility = m_ptr_list_gt_objects.at(obj_index)->get_object_extrapolated_visibility().at(
            sensor_index).at(current_frame_index);

    if (visibility) {

        if ( m_opticalFlowName == "ground_truth" ) {


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


            cv::Mat tempImage(Dataset::m_frame_size, CV_8UC3, cv::Scalar(255,255,255));
            // ---------------------------------------------------------------------------------------------------------
            // 1st step - Squared region of Interest
            std::vector<cv::Point2f> squared_region_of_interest;
            for (unsigned j = 0; j < width; j += 1) {
                for (unsigned k = 0; k < height; k += 1) {
                    squared_region_of_interest.push_back(
                            cv::Point2f(columnBegin + j, rowBegin + k));
                }
            }
            assert(squared_region_of_interest.size() > 0);
            // Validate
            tempImage = cv::Scalar::all(255);
            for ( auto it = squared_region_of_interest.begin(); it != squared_region_of_interest.end(); it++) {
                cv::circle(tempImage, (*it), 1, cv::Scalar(255,0,0));
            }
            //cv::imshow("coarse", tempImage);
            //cv::waitKey(0);
            cv::destroyAllWindows();

            // ---------------------------------------------------------------------------------------------------------
            // 2nd step - Refine intersection between squared ROI and frame difference.
            gt_frame_stencil_displacement_from_roi.resize(squared_region_of_interest.size());
            gt_frame_stencil_displacement_from_roi.clear();

            MyIntersection myIntersection_gt_roi_objects;
            std::vector<cv::Point2f>::iterator result_it;
            result_it = myIntersection_gt_roi_objects.find_intersection(squared_region_of_interest.begin(), squared_region_of_interest.end(),
                                                         all_moving_objects_in_frame.begin(), all_moving_objects_in_frame.end(),
                                                                        gt_frame_stencil_displacement_from_roi.begin());
            gt_frame_stencil_displacement_from_roi = myIntersection_gt_roi_objects.getResult();
            assert(gt_frame_stencil_displacement_from_roi.size() > 0);
            // Validate
            tempImage = cv::Scalar::all(255);
            for ( auto it = gt_frame_stencil_displacement_from_roi.begin(); it != gt_frame_stencil_displacement_from_roi.end(); it++) {
                cv::circle(tempImage, (*it), 1, cv::Scalar(0,255,0));
            }
            for ( auto it = all_moving_objects_in_frame.begin(); it != all_moving_objects_in_frame.end(); it++) {
                cv::circle(tempImage, (*it), 1, cv::Scalar(0,0,255));
            }
            //cv::imshow("refined", tempImage);
            //cv::waitKey(0);
            cv::destroyAllWindows();

            //---------------------------------------------------------------------------------------------------------
            // 3rd step - Refine intersection in case multiple objects are inside the ROI.
            float depth_value_object = m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(current_frame_index).m_object_distances.sensor_to_obj_usk;

            if ( m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(current_frame_index).object_name == "Pedesterian" ) {
                //depth_value_object++;
            }
            // The problem is that VIRES delivers depth map with respect to the rear axle and not the mounted sensor.
            // Still to do -> bounding box upper right and lower left corner is the total depth of the object. Right now, I am just considering a straight depth of 1 unit.
            float offset_x = m_list_of_gt_sensors.at(sensor_index).getExtrapolatedGroundTruthDetails().at(0).at(current_frame_index).m_sensor_offset_m.offset_x;
            for (unsigned j = 0; j < gt_frame_stencil_displacement_from_roi.size(); j += 1) {
                float val = depth_02_frame.at<unsigned char>(gt_frame_stencil_displacement_from_roi.at(j));
                //std::cout << (ushort)val << "*" << std::round(depth_value_object-3) << " ";
                bool found_correct_depth = (val == std::round(depth_value_object));
                if  ( Dataset::m_dataset_basepath.string() == VIRES_DATASET_PATH) {
                    found_correct_depth = (val >= ((depth_value_object-offset_x-1.5)) && val <= ((depth_value_object-offset_x)));
                }
                if ( found_correct_depth )  {
                    gt_frame_stencil_displacement_from_depth.push_back(gt_frame_stencil_displacement_from_roi.at(j));
                }
            }
            assert(gt_frame_stencil_displacement_from_depth.size()>0);
            // Validate
            tempImage = cv::Scalar::all(255);
            for ( auto it = gt_frame_stencil_displacement_from_depth.begin(); it != gt_frame_stencil_displacement_from_depth.end(); it++) {
                cv::circle(tempImage, (*it), 1, cv::Scalar(255,255,0));
            }
            //cv::imshow("depth", tempImage);
            //cv::waitKey(0);
            cv::destroyAllWindows();

            //---------------------------------------------------------------------------------------------------------
            // 4th step - Populate the ground truth object stencil with gt_displacement
            cv::Point2f gt_displacement = m_ptr_list_gt_objects.at(
                                obj_index)->get_object_extrapolated_point_displacement().at(sensor_index).at(
                                current_frame_index).second;
            for ( auto it = gt_frame_stencil_displacement_from_depth.begin(); it!=gt_frame_stencil_displacement_from_depth.end(); it++) {
                object_stencil_displacement.push_back(std::make_pair((*it), gt_displacement));
            }

            if ( cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(current_frame_index).m_object_location_camera_px.cog_px.x)  ) {
            }

            // std::sort(object_stencil_displacement.begin(), object_stencil_displacement.end(), PairPointsSort<float>());
            bool isSorted = std::is_sorted(object_stencil_displacement.begin(), object_stencil_displacement.end(), PairPointsSort<float>());
            assert(isSorted);

            //---------------------------------------------------------------------------------------------------------
            // 5th step - Populate the ground truth object stencil visibility with true
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

                //---------------------------------------------------------------------------------------------------------
                // Look for only those pixels that lie within the ground truth stencil of this particular object
                MyIntersection myIntersection;
                myIntersection.find_intersection_pair(entire_frame_algorithm_result_pts_displacement.begin(), entire_frame_algorithm_result_pts_displacement.end(),
                        m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).begin(),
                        m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).end());
                object_stencil_displacement = myIntersection.getResultIntersectingPair();
                frame_stencil_visibility.resize(object_stencil_displacement.size());
                std::fill(frame_stencil_visibility.begin(), frame_stencil_visibility.end(), (bool)1);

                for ( auto it = object_stencil_displacement.begin(); it != object_stencil_displacement.end(); it++) {
                    cv::circle(tempImage, (*it).first, 1, cv::Scalar(0,0,255));
                }

                //---------------------------------------------------------------------------------------------------------
                // Look for only those pixels that does not lie within the ground truth stencil of this particular object
                MyIntersection myDisjoint;
                std::vector<std::pair<cv::Point2f, cv::Point2f> >::iterator result_disjoint_it;
                std::vector<std::pair<cv::Point2f, cv::Point2f> > dummy(m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).size());


                result_disjoint_it = myDisjoint.find_disjoint_pair(object_stencil_displacement.begin(), object_stencil_displacement.end(),
                                                                  m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).begin(),
                                                                  m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).end(),
                                                                  dummy.begin());

                frame_stencil_disjoint_displacement = myDisjoint.getResultDisjointPair();
                //frame_stencil_disjoint_visibility.resize(frame_stencil_disjoint_displacement.size());
                //std::fill(frame_stencil_disjoint_visibility.begin(), frame_stencil_disjoint_visibility.end(), (bool)1);

                for ( auto it = frame_stencil_disjoint_displacement.begin(); it != frame_stencil_disjoint_displacement.end(); it++) {
                    cv::circle(tempImage, (*it).first, 1, cv::Scalar(255,0,0));
                }
                //cv::imshow("disjoint", tempImage);
                //cv::waitKey(0);
                cv::destroyAllWindows();

                std::cout << "found " << object_stencil_displacement.size() << " disjoint " << frame_stencil_disjoint_displacement.size() << " total " << m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).size() << std::endl;

                assert( ((object_stencil_displacement.size() + frame_stencil_disjoint_displacement.size()) <=  m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).size() + 25)
                         && ((object_stencil_displacement.size() + frame_stencil_disjoint_displacement.size())  >= m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).size() - 25)
                );

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

void OpticalFlow::save_flow_vector() {

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
    for (unsigned sensor_index = 0; sensor_index < Dataset::SENSOR_COUNT; sensor_index++) {

        unsigned FRAME_COUNT = (unsigned) m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(
                sensor_index).size();
        assert(FRAME_COUNT > 0);
        cv::Mat image_02_frame = cv::Mat::zeros(Dataset::m_frame_size, CV_32FC3);
        sprintf(sensor_index_folder_suffix, "%02d", m_evaluation_list.at(sensor_index));
        std::cout << "saving flow files in flow/ for sensor_index  " << sensor_index_folder_suffix << std::endl;

        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

            char file_name_input_image[50];
            char file_name_input_image_interpolated[50];
            ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                    (0).at(current_frame_index).frame_no;

            sprintf(file_name_input_image, "000%03d_10.png", image_frame_count);
            sprintf(file_name_input_image_interpolated, "interpolated_000%03d_10.png", image_frame_count);

            std::string flow_path = m_flow_occ_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
            std::string plot_path = m_plots_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;

            std::string flow_path_interpolated = m_flow_occ_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image_interpolated;
            std::string plot_path_interpolated = m_plots_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image_interpolated;

            if ( m_opticalFlowName == "ground_truth" ) {
                flow_path = GroundTruthScene::m_ground_truth_flow_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
                plot_path = GroundTruthScene::m_ground_truth_plot_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
            }

            FlowImageExtended F_png_write(Dataset::m_frame_size.width, Dataset::m_frame_size.height);
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
                    F_png_write.setObjectId(pts.x, pts.y, (obj_index));
                }
            }

            F_png_write.write(flow_path);
            F_png_write.writeColor(plot_path, max_magnitude);

            FlowImageExtended F_png_write_interpolated(F_png_write);

            for (auto obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++) {
                if (m_opticalFlowName != "ground_truth") {
                    unsigned CLUSTER_COUNT_DISJOINT_DATA = (unsigned) list_of_current_objects.at(
                            obj_index)->get_object_stencil_point_disjoint_displacement().at(sensor_index).at(
                            current_frame_index).size();

                    for (auto cluster_index = 0; cluster_index < CLUSTER_COUNT_DISJOINT_DATA; cluster_index++) {

                        cv::Point2f pts = list_of_current_objects.at(obj_index)->
                                get_object_stencil_point_disjoint_displacement().at(sensor_index).at(
                                current_frame_index).at(
                                cluster_index).first;

                        cv::Point2f displacement = list_of_current_objects.at(obj_index)->
                                get_object_stencil_point_disjoint_displacement().at(sensor_index).at(
                                current_frame_index).at(
                                cluster_index).second;

                        max_magnitude = std::max((float) cv::norm(displacement), max_magnitude);

                        displacement.x = 0;
                        displacement.y = 0;

                        F_png_write_interpolated.setFlowU(pts.x, pts.y, displacement.x);
                        F_png_write_interpolated.setFlowV(pts.x, pts.y, displacement.y);
                        F_png_write_interpolated.setObjectId(pts.x, pts.y, -1);
                    }
                }
            }
            // interpolate only for algorithm

            F_png_write_interpolated.interpolateBackground();
            F_png_write_interpolated.write(flow_path_interpolated);
            F_png_write_interpolated.writeColor(plot_path_interpolated, max_magnitude);
        }
    }

    std::cout << "end of saving " + m_resultordner + " flow files in an image" << std::endl;

}



void OpticalFlow::generate_sroi_intersections() {

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the position in a png file

    std::vector<Objects *> list_of_current_objects;

    list_of_current_objects = m_ptr_list_simulated_objects;

    char sensor_index_folder_suffix[50];
    for (unsigned sensor_index = 0; sensor_index < Dataset::SENSOR_COUNT; sensor_index++) {

        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > multiframe_stencil_displacement(m_ptr_list_simulated_objects.size());
        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > multiframe_stencil_displacement_interpolated(m_ptr_list_simulated_objects.size());

        unsigned FRAME_COUNT = (unsigned) m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(
                sensor_index).size();
        assert(FRAME_COUNT > 0);
        cv::Mat image_02_frame = cv::Mat::zeros(Dataset::m_frame_size, CV_32FC3);
        sprintf(sensor_index_folder_suffix, "%02d", m_evaluation_list.at(sensor_index));

        std::cout << "generate intersection data for sensor_index  " << sensor_index_folder_suffix << std::endl;

        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

            char file_name_input_image[50];
            char file_name_input_image_interpolated[50];
            ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                    (0).at(current_frame_index).frame_no;

            sprintf(file_name_input_image, "000%03d_10.png", image_frame_count);
            sprintf(file_name_input_image_interpolated, "interpolated_000%03d_10.png", image_frame_count);

            std::string flow_path = m_flow_occ_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
            std::string plot_path = m_plots_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;

            std::string flow_path_interpolated = m_flow_occ_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image_interpolated;
            std::string plot_path_interpolated = m_plots_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image_interpolated;

            if ( m_opticalFlowName == "ground_truth" ) {
                flow_path = GroundTruthScene::m_ground_truth_flow_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
                plot_path = GroundTruthScene::m_ground_truth_plot_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
            }

            FlowImageExtended F_png_write(Dataset::m_frame_size.width, Dataset::m_frame_size.height);
            std::cout << "current_frame_index " << current_frame_index << std::endl;
            float max_magnitude = 0.0;

            std::vector<std::pair<cv::Point2f, cv::Point2f> > intersection_of_algorithm_and_sroi;
            std::vector<std::pair<cv::Point2f, cv::Point2f> > intersection_of_interpolated_algorithm_and_sroi;

            for (auto obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++) {

                std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >  special_roi_object = m_ptr_list_gt_objects.at(obj_index)->get_object_special_region_of_interest();

                // sroi pixels
                // does eroi contains sroi coordinates? It should have because we are expanding eroi with new interpolated values. So, what is the final value?
                std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > entire_roi_object = list_of_current_objects.at(obj_index)->get_object_stencil_point_displacement();
                MyIntersection intersection;
                intersection.find_intersection_pair(entire_roi_object.at(sensor_index).at(current_frame_index).begin(), entire_roi_object.at(sensor_index).at(current_frame_index).end(), special_roi_object.at(sensor_index).at(current_frame_index).begin(), special_roi_object.at(sensor_index).at(current_frame_index).end());
                intersection_of_algorithm_and_sroi = intersection.getResultIntersectingPair();
                bool isSorted = std::is_sorted(entire_roi_object.at(sensor_index).at(current_frame_index).begin(), entire_roi_object.at(sensor_index).at(current_frame_index).end(), PairPointsSort<float>());
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
                //cv::imshow("algorithm_sroi", tempImage);
                //cv::waitKey(0);
                cv::destroyAllWindows();



                std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > entire_roi_object_interpolated = list_of_current_objects.at(obj_index)->get_object_interpolated_stencil_point_displacement();

                // sroi interpolated pixels
                // does eroi_interpolated contains sroi coordinates? It should have because we are expanding eroi with new interpolated values. So, what is the final value?

                MyIntersection intersection_interpolated_eroi_sroi_objects;

                intersection_interpolated_eroi_sroi_objects.find_intersection_pair(entire_roi_object_interpolated.at(sensor_index).at(current_frame_index).begin(), entire_roi_object_interpolated.at(sensor_index).at(current_frame_index).end(), special_roi_object.at(sensor_index).at(current_frame_index).begin(), special_roi_object.at(sensor_index).at(current_frame_index).end());
                intersection_of_interpolated_algorithm_and_sroi = intersection_interpolated_eroi_sroi_objects.getResultIntersectingPair();
                {
                    bool isSorted = std::is_sorted(entire_roi_object_interpolated.at(sensor_index).at(current_frame_index).begin(), entire_roi_object_interpolated.at(sensor_index).at(current_frame_index).end(), PairPointsSort<float>());
                    assert(isSorted);
                    bool isSorted_sroi = std::is_sorted(special_roi_object.at(sensor_index).at(current_frame_index).begin(), special_roi_object.at(sensor_index).at(current_frame_index).end(), PairPointsSort<float>());
                    assert(isSorted_sroi);

                }
                //assert(intersection_of_algorithm_and_sroi.size() > 0);
                // Validate
                cv::Mat tempImageInterpolated(Dataset::m_frame_size, CV_8UC3);
                tempImageInterpolated = cv::Scalar::all(255);
                for ( auto it = intersection_of_interpolated_algorithm_and_sroi.begin(); it != intersection_of_interpolated_algorithm_and_sroi.end(); it++) {
                    cv::circle(tempImageInterpolated, (*it).first, 1, cv::Scalar(255,0,0));
                }
                //cv::imshow("interpolated_algorithm_sroi", tempImageInterpolated);
                //cv::waitKey(0);
                cv::destroyAllWindows();

                multiframe_stencil_displacement.at(obj_index).push_back(intersection_of_algorithm_and_sroi);
                multiframe_stencil_displacement_interpolated.at(obj_index).push_back(intersection_of_interpolated_algorithm_and_sroi);

            }


        }
        for ( ushort obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++) {

            m_ptr_list_simulated_objects.at(obj_index)->push_back_object_intersection_sroi(multiframe_stencil_displacement.at(obj_index));
            m_ptr_list_simulated_objects.at(obj_index)->push_back_object_intersection_sroi_interpolated(multiframe_stencil_displacement_interpolated.at(obj_index));

        }
    }



    std::cout << "end of generation " + m_resultordner + " intersection data" << std::endl;

}




void OpticalFlow::rerun_optical_flow_algorithm() {

    for ( ushort sensor_index = 0; sensor_index < Dataset::SENSOR_COUNT; sensor_index++ ) {

        unsigned FRAME_COUNT = (unsigned)m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(sensor_index).size();
        assert(FRAME_COUNT>0);
        char sensor_index_folder_suffix[50];
        sprintf(sensor_index_folder_suffix, "%02d", m_evaluation_list.at(sensor_index));
        std::cout << "saving algorithm flow files in flow/ for sensor_index  " << sensor_index << std::endl;

        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > multiframe_stencil_displacement(m_ptr_list_simulated_objects.size());
        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > multiframe_stencil_disjoint_displacement(m_ptr_list_simulated_objects.size());

        std::vector<std::vector<std::vector<bool> >  > multiframe_visibility(m_ptr_list_simulated_objects.size());

        std::cout << "rerun algorithm results will be stored in " << m_resultordner << std::endl;

        for (ushort current_frame_index=0; current_frame_index < Dataset::MAX_ITERATION_RESULTS; current_frame_index++) {

            std::vector<cv::Point2f> frame_next_pts_array, displacement_array;

            char file_name_input_image[50];
            ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                    (0).at(current_frame_index).frame_no;

            sprintf(file_name_input_image, "interpolated_000%03d_10.png", image_frame_count);
            std::string flow_path = m_flow_occ_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;

            // read flow vector
            cv::Mat interpolatedFrame = cv::imread(flow_path, CV_LOAD_IMAGE_UNCHANGED);
            if ( interpolatedFrame.empty() ) {
                std::cerr << flow_path << " not found" << std::endl;
                throw ("No image file found error");
            }
            cv::cvtColor(interpolatedFrame, interpolatedFrame, CV_RGB2BGR);

            for ( auto row=0; row < Dataset::m_frame_size.height; row++) {
                for ( auto col=0; col < Dataset::m_frame_size.width; col++) {

                    if ( (interpolatedFrame.at<cv::Vec3w>(row,col)[2] != 0 ) && (interpolatedFrame.at<cv::Vec3w>(row,col)[2] != 32704) ) { //never initialised value ( -512 ) && no algorithm detected value ( -1 )
                        //std::cout << interpolatedFrame.at<cv::Vec3w>(row,col)[0] << " " << interpolatedFrame.at<cv::Vec3w>(row,col)[1] << " " << interpolatedFrame.at<cv::Vec3w>(row,col)[2] << std::endl;
                        frame_next_pts_array.push_back(cv::Point2f(col, row));
                        displacement_array.push_back(cv::Point2f(((interpolatedFrame.at<cv::Vec3w>(row,col)[0]-32768.0f)/64.0f), ((interpolatedFrame.at<cv::Vec3w>(row,col)[1]-32768.0f)/64.0f)));

                    }
                }
            }

            if (current_frame_index) {  // Calculate only on second or subsequent images.

                std::cout << "current_frame " << image_frame_count << std::endl;

                /// execute optical flow algorithms and get the flow vectors
                /*
                for (unsigned i = 0; i < frame_next_pts_array.size(); i++) {
                    //cv::circle(image_02_frame, frame_next_pts_array[i], 1, cv::Scalar(0, 255, 0), 1, 8);
                    cv::arrowedLine(image_02_frame, frame_prev_pts_array[i], frame_next_pts_array[i], cv::Scalar(0,255,0), 1, 8, 0, 0.5);
                }*/
                /// put data in object_stencil_displacement
                std::vector<std::pair<cv::Point2f,cv::Point2f > > dummy;
                common_flow_frame(sensor_index, current_frame_index, frame_next_pts_array, displacement_array, multiframe_stencil_displacement, multiframe_stencil_disjoint_displacement, multiframe_visibility);

            } else {

                std::cout << "skipping first frame frame count " << current_frame_index << std::endl;

                for ( ushort obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++ ) {
                    multiframe_stencil_displacement.at(obj_index).push_back({{std::make_pair(cv::Point2f(0, 0),cv::Point2f(0, 0))}});
                    multiframe_stencil_disjoint_displacement.at(obj_index).push_back({{std::make_pair(cv::Point2f(0, 0),cv::Point2f(0, 0))}});
                    multiframe_visibility.at(obj_index).push_back({{false}});
                }

            }
        }

        for ( ushort obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++) {

            m_ptr_list_simulated_objects.at(obj_index)->push_back_object_interpolated_stencil_point_displacement_pixel_visibility(multiframe_stencil_displacement.at(obj_index), multiframe_visibility.at(obj_index));
            m_ptr_list_simulated_objects.at(obj_index)->push_back_object_interpolated_stencil_point_disjoint_displacement_pixel_visibility(multiframe_stencil_disjoint_displacement.at(obj_index), multiframe_visibility.at(obj_index));

        }

    }

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

