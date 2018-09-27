

#include <opencv/cv.hpp>
#include <chrono>
#include "OpticalFlow.h"
#include "SortandIntersect.h"
#include "GenerateGroundTruthScene.h"


void OpticalFlow::frame_stencil_displacement_region_of_interest_method(ushort sensor_index, ushort current_frame_index, const std::vector<cv::Point2f> &frame_next_pts_array,
        const std::vector<cv::Point2f>  &displacement_array, ushort obj_index, std::vector<std::pair<cv::Point2f, cv::Point2f> > &object_stencil_displacement,
        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > &object_contour_stencil_displacement,
        std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_stencil_disjoint_displacement, std::vector<bool> &frame_stencil_visibility,
        const std::vector<cv::Point2f> &all_moving_objects_in_frame,
        const cv::Mat& depth_02_frame) {


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
            //cv::imshow("first_level_refined", tempImage);
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

                ushort depth = depth_02_frame.depth();  // if CV_8U=0 or CV_8S=1 ?

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
            if ( gt_frame_stencil_displacement_from_depth.size()==0 ) {
                // this object is not visible
                //m_ptr_list_gt_objects.at(obj_index)->set_object_base_visibility(sensor_index, current_frame_index, false);
                m_ptr_list_gt_objects.at(obj_index)->set_object_extrapolated_visibility(sensor_index, current_frame_index, false);
                visibility = false;
            }
            // to do assert only when vires reported true visibility, but depth returned false visiblity assert(gt_frame_stencil_displacement_from_depth.size()>0);
            // Validate
            tempImage = cv::Scalar::all(255);
            for ( auto it = gt_frame_stencil_displacement_from_depth.begin(); it != gt_frame_stencil_displacement_from_depth.end(); it++) {
                cv::circle(tempImage, (*it), 1, cv::Scalar(255,255,0));
            }
            //cv::imshow("depth_level_refined", tempImage);
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

            if (m_noise == "blue_sky"  || m_noise == "heavy_snow") {

                // Intersection between ground truth stencil and the algorithm stencil.

                cv::Mat tempImage(Dataset::m_frame_size, CV_8UC3, cv::Scalar(255,255,255));

                object_stencil_displacement.resize(frame_next_pts_array.size());
                frame_stencil_visibility.resize(frame_next_pts_array.size());

                object_stencil_displacement.clear();
                frame_stencil_visibility.clear();

                assert(get_simulated_objects_ptr_list().size() == m_ptr_list_gt_objects.size());
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
                    //cv::circle(tempImage, (*it).first, 1, cv::Scalar(0,0,255));
                }
                //cv::imshow("all_algo", tempImage);
                //cv::waitKey(0);

                //---------------------------------------------------------------------------------------------------------
                // Look for only those pixels that lie within the ground truth stencil of this particular object
                for ( ushort obj_contour_index = 0; obj_contour_index < object_contour_stencil_displacement.size(); obj_contour_index++) {

                    tempImage = cv::Scalar::all(255);
                    MyIntersection myIntersectionContour;
                    myIntersectionContour.find_intersection_pair(entire_frame_algorithm_result_pts_displacement.begin(), entire_frame_algorithm_result_pts_displacement.end(),
                            m_ptr_list_gt_objects.at(obj_index)->get_object_contour_region_of_interest().at(sensor_index).at(current_frame_index).at(obj_contour_index).begin(),
                            m_ptr_list_gt_objects.at(obj_index)->get_object_contour_region_of_interest().at(sensor_index).at(current_frame_index).at(obj_contour_index).end());
                    object_contour_stencil_displacement.at(obj_contour_index) = myIntersectionContour.getResultIntersectingPair();

                    for ( auto it = object_contour_stencil_displacement.at(obj_contour_index).begin(); it != object_contour_stencil_displacement.at(obj_contour_index).end(); it++) {
                        cv::circle(tempImage, (*it).first, 1, cv::Scalar(0,0,255));
                    }
                    //cv::imshow("algo_contours_" + std::to_string(obj_contour_index), tempImage);
                    //cv::waitKey(0);

                }

                cv::destroyAllWindows();

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
                    //cv::circle(tempImage, (*it).first, 1, cv::Scalar(255,0,0));
                }
                //cv::imshow("disjoint", tempImage);
                //cv::waitKey(0);
                cv::destroyAllWindows();

                std::cout << "found " << object_stencil_displacement.size() << " disjoint " << frame_stencil_disjoint_displacement.size() << " total " << m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).size() << std::endl;

                /*
                assert( ((object_stencil_displacement.size() + frame_stencil_disjoint_displacement.size()) <=  m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).size() + 25)
                         && ((object_stencil_displacement.size() + frame_stencil_disjoint_displacement.size())  >= m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement().at(sensor_index).at(current_frame_index).size() - 25)
                );
                */

                //InterpolateData interpolateData;
                //interpolateData.interpolateBackground(object_stencil_displacement, frame_stencil_disjoint_displacement);

            } else {

                // this should contain the intersection between entire_frame_algorithm_result_pts_displacement and algorithm_result_pts_displacement
                // so we need to store the data.

                /*
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
                 */
            }
        }

        std::cout << "stencil size = " << object_stencil_displacement.size() << " " << frame_next_pts_array.size()
                  << std::endl;

        if ( visibility ) {
            if (m_opticalFlowName == "ground_truth") {
            assert(object_stencil_displacement.size() != 0);
            }
            // It is completely fine if the algorithm fails to detect any object points
        }
        // TODO scratch : if object_stencil_displacement does not work

    }

    else {

        object_stencil_displacement.push_back(std::make_pair(cv::Point2f(0, 0), cv::Point2f(0, 0)));
        frame_stencil_visibility.push_back(visibility);

    }

}


void OpticalFlow::common_flow_frame(ushort sensor_index, ushort current_frame_index, const std::vector<cv::Point2f> &frame_next_pts_array, const std::vector<cv::Point2f>  &displacement_array,std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &multiframe_stencil_displacement, std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &multiframe_stencil_disjoint_displacement, std::vector<std::vector<std::vector<bool> >  > &multiframe_stencil_visibility,
                                    std::vector<cv::Point2f> all_moving_objects_in_frame) {

    char sensor_index_folder_suffix[50];
    sprintf(sensor_index_folder_suffix, "%02d", sensor_index);

    ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
            (0).at(current_frame_index).frame_no;

    // DEPTH READ
    char file_name_input_image_depth[50];
    sprintf(file_name_input_image_depth, "depth_000%03d_10.png", image_frame_count);
    std::string input_image_path_depth = GroundTruthScene::m_ground_truth_depth_path.string()  + sensor_index_folder_suffix + "/" + file_name_input_image_depth;
    cv::Mat depth_02_frame = cv::imread(input_image_path_depth, CV_LOAD_IMAGE_GRAYSCALE);
    if ( depth_02_frame.empty() ) {
        std::cerr << input_image_path_depth << " not found" << std::endl;
        throw ("No image file found error");
    }

    auto START_BENCHMARK
    for (ushort obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++) {

        std::vector<std::pair<cv::Point2f, cv::Point2f> > object_stencil_displacement;
        std::vector<std::pair<cv::Point2f, cv::Point2f> > frame_stencil_disjoint_displacement;
        std::vector<bool> frame_stencil_visibility;

        std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > >  object_contours = m_ptr_list_gt_objects.at(obj_index)->get_object_contour_region_of_interest();
        ushort CONTOUR_COUNT;

        if ( m_opticalFlowName== "ground_truth") {
            CONTOUR_COUNT = 0;
        } else {
            CONTOUR_COUNT = object_contours.at(sensor_index).at(current_frame_index).size();
        }
        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > object_contour_stencil_displacement(CONTOUR_COUNT);


        frame_stencil_displacement_region_of_interest_method(sensor_index, current_frame_index, frame_next_pts_array, displacement_array, obj_index, object_stencil_displacement, object_contour_stencil_displacement, frame_stencil_disjoint_displacement,frame_stencil_visibility,  all_moving_objects_in_frame, depth_02_frame);

        multiframe_stencil_displacement.at(obj_index).push_back(object_stencil_displacement);
        multiframe_stencil_disjoint_displacement.at(obj_index).push_back(frame_stencil_disjoint_displacement);
        multiframe_stencil_visibility.at(obj_index).push_back(frame_stencil_visibility);

    }
    PRINT_BENCHMARK(frame_stencil_displacement_time_required)
}

