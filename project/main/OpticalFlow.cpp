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
#include "GenerateGroundTruthScene.h"
#include <gnuplot-iostream/gnuplot-iostream.h>

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


void OpticalFlow::save_flow_vector() {

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the position in a png file
    std::vector<Objects *> ptr_list_of_current_objects;

    if (m_opticalFlowName == "ground_truth") {
        for ( auto i = 0; i < m_ptr_list_gt_objects.size(); i++) {
            ptr_list_of_current_objects.push_back(static_cast<GroundTruthObjects*>(m_ptr_list_gt_objects.at(i)));
        }
    } else {

        for ( auto i = 0; i < get_simulated_objects_ptr_list().size(); i++) {
            ptr_list_of_current_objects.push_back(static_cast<SimulatedObjects*>(get_simulated_objects_ptr_list().at(i)));
        }
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
                flow_path_interpolated = GroundTruthScene::m_ground_truth_flow_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image_interpolated;
                plot_path_interpolated = GroundTruthScene::m_ground_truth_plot_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image_interpolated;
            }

            FlowImageExtended F_png_write(Dataset::m_frame_size.width, Dataset::m_frame_size.height);
            std::cout << "current_frame_index " << current_frame_index << std::endl;
            float max_magnitude = 0.0;

            for (auto obj_index = 0; obj_index < ptr_list_of_current_objects.size(); obj_index++) {

                unsigned CLUSTER_COUNT = (unsigned) ptr_list_of_current_objects.at(
                        obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).size();

                for (auto cluster_index = 0; cluster_index < CLUSTER_COUNT; cluster_index++) {

                    cv::Point2f pts = ptr_list_of_current_objects.at(obj_index)->
                            get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).at(
                            cluster_index).first;

                    cv::Point2f displacement = ptr_list_of_current_objects.at(obj_index)->
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

            cv::Mat readColorPlot = cv::imread(plot_path, CV_LOAD_IMAGE_COLOR);

            FlowImageExtended F_png_write_interpolated(F_png_write);

            if (m_opticalFlowName != "ground_truth") {
                for (auto obj_index = 0; obj_index < ptr_list_of_current_objects.size(); obj_index++) {

                    // get ground truth sroi area
                    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >  special_roi_object = m_ptr_list_gt_objects.at(obj_index)->get_object_special_region_of_interest();
                    for ( auto it = special_roi_object.at(sensor_index).at(current_frame_index).begin(); it != special_roi_object.at(sensor_index).at(current_frame_index).end(); it++ ) {
                        cv::Vec3b current_pixel_value = readColorPlot.at<cv::Vec3b>((*it).first);
                        if ( current_pixel_value == cv::Vec3b(0,0,0)) {
                            readColorPlot.at<cv::Vec3b>((*it).first) = cv::Vec3b(255,255,255);
                        } else {
                            readColorPlot.at<cv::Vec3b>((*it).first) = cv::Vec3b(127,127,127);
                        }
                    }
                    cv::imwrite(plot_path, readColorPlot);

                    unsigned CLUSTER_COUNT_DISJOINT_DATA = (unsigned) ptr_list_of_current_objects.at(
                            obj_index)->get_object_stencil_point_disjoint_displacement().at(sensor_index).at(
                            current_frame_index).size();

                    for (auto cluster_index = 0; cluster_index < CLUSTER_COUNT_DISJOINT_DATA; cluster_index++) {

                        cv::Point2f pts = ptr_list_of_current_objects.at(obj_index)->
                                get_object_stencil_point_disjoint_displacement().at(sensor_index).at(
                                current_frame_index).at(
                                cluster_index).first;

                        cv::Point2f displacement = ptr_list_of_current_objects.at(obj_index)->
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
                // interpolate only for algorithm
                F_png_write_interpolated.interpolateBackground();
                F_png_write_interpolated.write(flow_path_interpolated);
                F_png_write_interpolated.writeColor(plot_path_interpolated, max_magnitude);
            }
        }
    }
    std::cout << "end of saving " + m_resultordner + " flow files in an image" << std::endl;
}

/**
 * This function generates the intersection of ground truth special roi and entire roi from the optical flow algorithm
 * The second step of this function is to geneate the intersection of ground truth special roi and the interpolated roi from from the interpolation algorithm
 */

void OpticalFlow::generate_sroi_intersections() {

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the position in a png file

    std::vector<Objects *> ptr_list_of_current_objects;

    for ( auto i = 0; i < get_simulated_objects_ptr_list().size(); i++) {
        ptr_list_of_current_objects.push_back(static_cast<SimulatedObjects*>(get_simulated_objects_ptr_list().at(i)));
    }

    ptr_list_of_current_objects = ptr_list_of_current_objects;

    char sensor_index_folder_suffix[50];
    for (unsigned sensor_index = 0; sensor_index < Dataset::SENSOR_COUNT; sensor_index++) {

        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > multiframe_stencil_displacement_sroi(m_ptr_list_gt_objects.size());
        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > multiframe_stencil_displacement_sroi_interpolated(m_ptr_list_gt_objects.size());

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
            std::string gt_image_path = m_GroundTruthImageLocation.string() + "_" + sensor_index_folder_suffix + "/" + file_name_input_image;

            std::string flow_path_interpolated = m_flow_occ_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image_interpolated;
            std::string plot_path_interpolated = m_plots_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image_interpolated;

            if ( m_opticalFlowName == "ground_truth" ) {
                flow_path = GroundTruthScene::m_ground_truth_flow_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
                plot_path = GroundTruthScene::m_ground_truth_plot_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
            }

            std::cout << "current_frame_index " << current_frame_index << std::endl;

            std::vector<std::pair<cv::Point2f, cv::Point2f> > intersection_of_algorithm_and_sroi;
            std::vector<std::pair<cv::Point2f, cv::Point2f> > intersection_of_interpolated_algorithm_and_sroi;

            cv::Mat tempImage(Dataset::m_frame_size, CV_8UC3);
            tempImage = cv::imread(gt_image_path, CV_LOAD_IMAGE_COLOR);

            cv::Mat tempImageInterpolated(Dataset::m_frame_size, CV_8UC3);
            tempImageInterpolated = cv::imread(gt_image_path, CV_LOAD_IMAGE_COLOR);

            for (auto obj_index = 0; obj_index < ptr_list_of_current_objects.size(); obj_index++) {

                std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >  special_roi_object = m_ptr_list_gt_objects.at(obj_index)->get_object_special_region_of_interest();
                // sroi pixels
                // does eroi contains sroi coordinates? It should have because we are expanding eroi with new interpolated values. So, what is the final value?
                std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > entire_roi_object = ptr_list_of_current_objects.at(obj_index)->get_object_stencil_point_displacement();
                MyIntersection intersection;

                if ( ( special_roi_object.at(sensor_index).at(current_frame_index).size() > 0 ) && current_frame_index > 0 ) {

                    for ( auto it = entire_roi_object.at(sensor_index).at(current_frame_index).begin(); it != entire_roi_object.at(sensor_index).at(current_frame_index).end(); it++) {
                        cv::circle(tempImage, (*it).first, 1, cv::Scalar(obj_index*255,255,0));
                    }

                    for ( auto it = special_roi_object.at(sensor_index).at(current_frame_index).begin(); it != special_roi_object.at(sensor_index).at(current_frame_index).end(); it++) {
                        cv::circle(tempImage, (*it).first, 1, cv::Scalar(255,255,255));
                    }

                    intersection.find_intersection_pair(entire_roi_object.at(sensor_index).at(current_frame_index).begin(), entire_roi_object.at(sensor_index).at(current_frame_index).end(), special_roi_object.at(sensor_index).at(current_frame_index).begin(), special_roi_object.at(sensor_index).at(current_frame_index).end());
                    intersection_of_algorithm_and_sroi = intersection.getResultIntersectingPair();
                    bool isSorted = std::is_sorted(entire_roi_object.at(sensor_index).at(current_frame_index).begin(), entire_roi_object.at(sensor_index).at(current_frame_index).end(), PairPointsSort<float>());
                    assert(isSorted);
                    bool isSorted_sroi = std::is_sorted(special_roi_object.at(sensor_index).at(current_frame_index).begin(), special_roi_object.at(sensor_index).at(current_frame_index).end(), PairPointsSort<float>());
                    assert(isSorted_sroi);

                    // Its completely possible that no data is present in the sroi region.
                    //assert(intersection_of_algorithm_and_sroi.size() > 0);
                    // Validate
                    for ( auto it = intersection_of_algorithm_and_sroi.begin(); it != intersection_of_algorithm_and_sroi.end(); it++) {
                        cv::circle(tempImage, (*it).first, 1, cv::Scalar(127,127,127));
                    }
                    //cv::imshow("intersection_algorithm_sroi", tempImage);
                    //cv::waitKey(0);
                    cv::destroyAllWindows();
                }
                multiframe_stencil_displacement_sroi.at(obj_index).push_back(intersection_of_algorithm_and_sroi);

//----------------------------------------------------------------------------------------------------------------------
                std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > entire_roi_object_interpolated = ptr_list_of_current_objects.at(obj_index)->get_object_interpolated_stencil_point_displacement();

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
                for ( auto it = intersection_of_interpolated_algorithm_and_sroi.begin(); it != intersection_of_interpolated_algorithm_and_sroi.end(); it++) {
                    cv::circle(tempImageInterpolated, (*it).first, 1, cv::Scalar(obj_index*255,255,0));
                }
                //cv::imshow("interpolated_algorithm_sroi", tempImageInterpolated);
                //cv::waitKey(0);
                cv::destroyAllWindows();

                multiframe_stencil_displacement_sroi_interpolated.at(obj_index).push_back(intersection_of_interpolated_algorithm_and_sroi);

            }

        }
        for ( ushort obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++) {

            get_simulated_objects_ptr_list().at(obj_index)->push_back_object_sroi(multiframe_stencil_displacement_sroi.at(obj_index));
            get_simulated_objects_ptr_list().at(obj_index)->push_back_object_sroi_interpolated(multiframe_stencil_displacement_sroi_interpolated.at(obj_index));

        }
    }

    std::cout << "end of generation " + m_resultordner + " intersection data" << std::endl;

}



/**
 * This function reads the interpolated_flow image and creates the necessary interpolated vectors in the Object class
 */
void OpticalFlow::rerun_optical_flow_algorithm_interpolated() {

    std::vector<Objects *> ptr_list_of_current_objects;

    if (m_opticalFlowName == "ground_truth") {
        for ( auto i = 0; i < m_ptr_list_gt_objects.size(); i++) {
            ptr_list_of_current_objects.push_back(static_cast<GroundTruthObjects*>(m_ptr_list_gt_objects.at(i)));
        }
    } else {
        for ( auto i = 0; i < m_ptr_list_gt_objects.size(); i++) {
            ptr_list_of_current_objects.push_back(static_cast<SimulatedObjects*>(get_simulated_objects_ptr_list().at(i)));
        }
    }

    for ( ushort sensor_index = 0; sensor_index < Dataset::SENSOR_COUNT; sensor_index++ ) {

        if ( m_opticalFlowName != "ground_truth") {

            unsigned FRAME_COUNT = Dataset::MAX_ITERATION_RESULTS;
            assert(FRAME_COUNT > 0);
            char sensor_index_folder_suffix[50];
            sprintf(sensor_index_folder_suffix, "%02d", m_evaluation_list.at(sensor_index));
            std::cout << "saving algorithm flow files in flow/ for sensor_index  " << sensor_index << std::endl;

            std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > multiframe_stencil_displacement_interpolated(
                    ptr_list_of_current_objects.size());

            std::vector<std::vector<std::vector< GROUND_TRUTH_CONTOURS > > > multiframe_contour_stencil_displacement_interpolated(
                    ptr_list_of_current_objects.size());

            std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > multiframe_stencil_disjoint_displacement_interpolated(
                    ptr_list_of_current_objects.size());

            std::vector<std::vector<std::vector<bool> > > multiframe_visibility(ptr_list_of_current_objects.size());

            std::cout << "rerun algorithm results will be stored in " << m_resultordner << std::endl;

            for (ushort current_frame_index = 0;
                 current_frame_index < Dataset::MAX_ITERATION_RESULTS; current_frame_index++) {

                std::vector<cv::Point2f> frame_next_pts_array, displacement_array;

                char file_name_input_image[50];
                ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                        (0).at(current_frame_index).frame_no;

                sprintf(file_name_input_image, "interpolated_000%03d_10.png", image_frame_count);
                std::string flow_path =
                        m_flow_occ_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;

                // read flow vector
                cv::Mat interpolatedFrame = cv::imread(flow_path, CV_LOAD_IMAGE_UNCHANGED);
                if (interpolatedFrame.empty()) {
                    std::cerr << flow_path << " not found" << std::endl;
                    throw ("No image file found error");
                }
                cv::cvtColor(interpolatedFrame, interpolatedFrame, CV_RGB2BGR);

                for (auto row = 0; row < Dataset::m_frame_size.height; row++) {
                    for (auto col = 0; col < Dataset::m_frame_size.width; col++) {

                        if ((interpolatedFrame.at<cv::Vec3w>(row, col)[2] != 0) &&
                            (interpolatedFrame.at<cv::Vec3w>(row, col)[2] !=
                             32704)) { //never initialised value ( -512 ) && no algorithm detected value ( -1 )
                            //std::cout << interpolatedFrame.at<cv::Vec3w>(row,col)[0] << " " << interpolatedFrame.at<cv::Vec3w>(row,col)[1] << " " << interpolatedFrame.at<cv::Vec3w>(row,col)[2] << std::endl;
                            frame_next_pts_array.push_back(cv::Point2f(col, row));
                            displacement_array.push_back(
                                    cv::Point2f(((interpolatedFrame.at<cv::Vec3w>(row, col)[0] - 32768.0f) / 64.0f),
                                                ((interpolatedFrame.at<cv::Vec3w>(row, col)[1] - 32768.0f) /
                                                 64.0f)));

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
                    std::vector<std::pair<cv::Point2f, cv::Point2f> > dummy;
                    common_flow_frame(sensor_index, current_frame_index, frame_next_pts_array, displacement_array,
                                      multiframe_stencil_displacement_interpolated, multiframe_contour_stencil_displacement_interpolated,
                                      multiframe_stencil_disjoint_displacement_interpolated, multiframe_visibility);

                } else {

                    std::cout << "skipping first frame frame count " << current_frame_index << std::endl;

                    for (ushort obj_index = 0; obj_index < ptr_list_of_current_objects.size(); obj_index++) {
                        multiframe_stencil_displacement_interpolated.at(obj_index).push_back(
                                {{std::make_pair(cv::Point2f(0, 0), cv::Point2f(0, 0))}});
                        multiframe_stencil_disjoint_displacement_interpolated.at(obj_index).push_back(
                                {{std::make_pair(cv::Point2f(0, 0), cv::Point2f(0, 0))}});
                        multiframe_visibility.at(obj_index).push_back({{false}});
                    }

                }
            }

            for (ushort obj_index = 0; obj_index < ptr_list_of_current_objects.size(); obj_index++) {

                ptr_list_of_current_objects.at(
                        obj_index)->push_back_object_interpolated_stencil_point_displacement_pixel_visibility(
                        multiframe_stencil_displacement_interpolated.at(obj_index),
                        multiframe_visibility.at(obj_index));
                ptr_list_of_current_objects.at(
                        obj_index)->push_back_object_interpolated_stencil_point_disjoint_displacement_pixel_visibility(
                        multiframe_stencil_disjoint_displacement_interpolated.at(obj_index),
                        multiframe_visibility.at(obj_index));
            }
        }
        else {
            for ( ushort obj_index = 0; obj_index < ptr_list_of_current_objects.size(); obj_index++) {

                ptr_list_of_current_objects.at(obj_index)->push_back_object_interpolated_stencil_point_displacement_pixel_visibility(m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index), m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_visibility().at(sensor_index));
                //ptr_list_of_current_objects.at(obj_index)->push_back_object_interpolated_stencil_point_disjoint_displacement_pixel_visibility(multiframe_stencil_disjoint_displacement_interpolated.at(obj_index), multiframe_visibility.at(obj_index));
            }
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

