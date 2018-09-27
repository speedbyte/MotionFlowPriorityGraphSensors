
#include <vector>
#include <cmath>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>
#include <map>
#include <chrono>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <png++/png.hpp>
#include <gnuplot-iostream/gnuplot-iostream.h>

#include "Dataset.h"
#include "GroundTruthFlow.h"
#include "kbhit.h"

#include <unordered_map>
#include <bits/unordered_map.h>

#include "Dataset.h"
#include "GenerateGroundTruthScene.h"
#include "SortandIntersect.h"


//Creating a movement path. The path is stored in a x and y vector


void GroundTruthFlow::prepare_groundtruth_flow_directories(std::string noise, ushort fps, ushort stepSize) {

    m_GroundTruthImageLocation = Dataset::m_dataset_gtpath.string() + noise;

    m_resultordner="/ground_truth";

    m_generatepath = Dataset::m_dataset_gtpath.string() + m_resultordner;

    if (!Dataset::m_dataset_basepath.compare(CPP_DATASET_PATH) || !Dataset::m_dataset_basepath.compare(VIRES_DATASET_PATH)) {

        prepare_directories_common();

    }
}


void GroundTruthFlow::generate_flow_vector() {

    std::cout << "ground truth flow will be stored in " << m_generatepath << std::endl;

    char sensor_index_folder_suffix[50];

    for (ushort sensor_index = 0; sensor_index < Dataset::SENSOR_COUNT; sensor_index++) {

        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > multiframe_stencil_displacement(
                m_ptr_list_gt_objects.size());
        std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > > multiframe_contour_stencil_displacement(
                m_ptr_list_gt_objects.size());
        std::vector<std::vector<std::vector<bool> > > multiframe_visibility(m_ptr_list_gt_objects.size());
        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > multiframe_stencil_disjoint_displacement(
                m_ptr_list_gt_objects.size());

        unsigned FRAME_COUNT = (unsigned) m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(
                sensor_index).size();
        assert(FRAME_COUNT > 0);

        sprintf(sensor_index_folder_suffix, "%02d", sensor_index);

        std::cout << "generating " + m_resultordner + " flow vector for sensor_index  " << sensor_index << std::endl;

        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

            char file_name_image_output[50];
            ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                    (0).at(current_frame_index).frame_no;
            sprintf(file_name_image_output, "000%03d_10.png", image_frame_count);

            std::cout << "current_frame " << image_frame_count << std::endl;

            std::string frame_difference_path = GroundTruthScene::m_ground_truth_framedifference_path.string() + sensor_index_folder_suffix + "/" + file_name_image_output;
            cv::Mat frameDifference = cv::imread(frame_difference_path, CV_LOAD_IMAGE_ANYCOLOR);
            if ( frameDifference.data == NULL ) {
                std::cout << "no image found, exiting" << std::endl;
                throw;
            }
            // new method - divide final into contours
            std::vector<std::vector<cv::Point> > contours;
            cv::findContours(frameDifference, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

            cv::Point2f max_val = (*std::max_element(contours.at(0).begin(), contours.at(0).end(), PointsSort<int>()));

            std::vector<cv::Point2f> all_moving_objects_in_frame;
            for (unsigned j = 0; j < frameDifference.cols; j += 1) {
                for (unsigned k = 0; k < frameDifference.rows; k += 1) {
                    if ( frameDifference.at<char>(k,j) == 0 ) {
                        all_moving_objects_in_frame.push_back(
                                (cv::Point2f(j, k)));
                    }
                }
            }

            // dummy declare frame_next_pts_array. Just to maintain the arguement list
            std::vector<cv::Point2f> frame_next_pts_array, displacement_array;

            common_flow_frame(sensor_index, current_frame_index, frame_next_pts_array, displacement_array,
                              multiframe_stencil_displacement, multiframe_contour_stencil_displacement, multiframe_stencil_disjoint_displacement, multiframe_visibility, all_moving_objects_in_frame);

        }

        for (ushort obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++) {

            m_ptr_list_gt_objects.at(obj_index)->push_back_object_stencil_point_displacement_pixel_visibility(
                    multiframe_stencil_displacement.at(obj_index), multiframe_visibility.at(obj_index));
        }

        cv::destroyAllWindows();
    }

    std::cout << "end of saving " + m_resultordner + " flow files in an vector" << std::endl;

}





void GroundTruthFlow::find_ground_truth_object_special_region_of_interest() {

    // Intersection between pair of objects. Total visible pixels is known. This metric will show how many
    // pixels lie on the occlusion boundary.
    // intersection coordinates, convert to Mat, fill with 0, and then find the contour. This is the occlusion boundary.
    // subtract area for evaluation from the object farther away.
    // total = visible part object front + visible part object back
    // boundary = common
    // invisible part = total object part back - visible part object back
    // object_stencil_displacement contains ground truth - all pixels are present in the vector
    // Intersection bounding box

    // depends on flow image and flow stencil

    // -----


    std::vector<Objects *> ptr_list_of_current_objects;
    std::vector<Objects *> ptr_list_of_copied_gt_objects;
    std::vector<Objects *> ptr_list_of_copied_simulated_objects;
    for ( auto i = 0; i < m_ptr_list_gt_objects.size(); i++) {
        ptr_list_of_copied_gt_objects.push_back(static_cast<Objects*>(m_ptr_list_gt_objects.at(i)));
    }

    if (m_opticalFlowName == "ground_truth") {
        ptr_list_of_current_objects = ptr_list_of_copied_gt_objects;
    } else {

        for ( auto i = 0; i < get_simulated_objects_ptr_list().size(); i++) {
            ptr_list_of_copied_simulated_objects.push_back(static_cast<Objects*>(get_simulated_objects_ptr_list().at(i)));
        }

        ptr_list_of_current_objects = ptr_list_of_copied_simulated_objects;
    }

    char sensor_index_folder_suffix[50];

    std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > > all_sensors_object_special_region_of_interest(m_ptr_list_gt_objects.size());

    for (unsigned sensor_index = 0; sensor_index < Dataset::SENSOR_COUNT; sensor_index++) {


        unsigned FRAME_COUNT = (unsigned) m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(
                sensor_index).size();
        assert(FRAME_COUNT > 0);

        cv::Mat image_02_frame = cv::Mat::zeros(Dataset::m_frame_size, CV_32FC3);
        sprintf(sensor_index_folder_suffix, "%02d", m_evaluation_list.at(sensor_index));

        std::cout << "begin distance calculation between objects" << sensor_index_folder_suffix << std::endl;

        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > all_frame_object_special_region_of_interest(m_ptr_list_gt_objects.size());


        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {


            std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > frame_object_special_region_of_interest(m_ptr_list_gt_objects.size());

            char file_name_input_image[50];
            ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                    (0).at(current_frame_index).frame_no;

            sprintf(file_name_input_image, "000%03d_10.png", image_frame_count);
            std::string flow_path = m_generatepath.parent_path().string() + "/flow_occ_" + sensor_index_folder_suffix + "/" + file_name_input_image;
            //std::string kitti_path = m_plots_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
            std::string gt_image_path = m_GroundTruthImageLocation.string() + "_" + sensor_index_folder_suffix + "/" + file_name_input_image;

            std::cout << "current_frame_index  " << current_frame_index << std::endl;

            std::vector<std::pair<Objects *, Objects *> > list_of_gt_objects_combination;

            getCombination(ptr_list_of_copied_gt_objects, list_of_gt_objects_combination);

            cv::Mat flow_image = cv::imread(flow_path, CV_LOAD_IMAGE_UNCHANGED);
            cv::cvtColor(flow_image, flow_image, CV_RGB2BGR);
            if ( flow_image.empty() ) {
                throw("No image found error");
            }

            cv::Mat intersection_image(Dataset::m_frame_size, CV_MAKE_TYPE(flow_image.depth(),1));
            int from_to[] = { 2,0 };  // copy the third channel ( channel 2 object id ) to the first channel of intersection_image
            cv::mixChannels(flow_image, intersection_image, from_to, 1); // the last parameter is the number of pairs

            for (ushort obj_combination_index = 0;
                 obj_combination_index < list_of_gt_objects_combination.size(); obj_combination_index++) {

                cv::Point2f gt_displacement_1 = list_of_gt_objects_combination.at(obj_combination_index).first->get_object_extrapolated_point_displacement().at(sensor_index).at(current_frame_index).second;
                cv::Point2f gt_displacement_2 = list_of_gt_objects_combination.at(obj_combination_index).second->get_object_extrapolated_point_displacement().at(sensor_index).at(current_frame_index).second;

                std::vector<std::pair<cv::Point2f, cv::Point2f> > frame_special_region_of_interest_1;
                std::vector<std::pair<cv::Point2f, cv::Point2f> > frame_special_region_of_interest_2;

                cv::Mat mask_object_1, mask_object_2;
                cv::Mat mask_object_1_dilated, mask_object_2_dilated;
                cv::Mat mask_object_1_eroded, mask_object_2_eroded, mask_object_1_eroded_pre, mask_object_2_eroded_pre;

                ushort val_1 = (ushort)(list_of_gt_objects_combination.at(obj_combination_index).first->getObjectId() * 64 + 32768) ;
                ushort val_2 = (ushort)(list_of_gt_objects_combination.at(obj_combination_index).second->getObjectId() * 64 + 32768);

                cv::inRange(intersection_image, val_1, val_1, mask_object_1);
                cv::inRange(intersection_image, val_2, val_2, mask_object_2);

                //cv::imshow("mask_1", mask_object_1);
                //cv::imshow("mask_2", mask_object_2);
                //cv::waitKey(0);

                mask_object_1_dilated = mask_object_1.clone();
                mask_object_2_dilated = mask_object_2.clone();

                for (int i=0; i<5; i++) {
                    cv::dilate(mask_object_1_dilated, mask_object_1_dilated, cv::Mat());
                    cv::dilate(mask_object_2_dilated, mask_object_2_dilated, cv::Mat());
                }

                //cv::Mat final = mask_object_1_dilated & mask_object_2_dilated;
                //cv::Mat boundary = final & mask_object_2;

                cv::Mat special_region_of_interest_1 = mask_object_1 & mask_object_2_dilated; // elements in mask object 1
                cv::Mat special_region_of_interest_2 = mask_object_2 & mask_object_1_dilated; // elements in mask object 2

                for ( auto x = 0; x < special_region_of_interest_1.cols; x++ ) {
                    for ( auto y = 0; y < special_region_of_interest_1.rows; y++) {
                        // there is only one object per Mat. Hence we can safely scan the whole frame.
                        if ( special_region_of_interest_1.at<unsigned char>(y,x) != 0 ) {
                            frame_special_region_of_interest_1.push_back(std::make_pair(cv::Point2f(x,y), gt_displacement_1));
                        }
                    }
                }

                for ( auto x = 0; x < special_region_of_interest_2.cols; x++ ) {
                    for ( auto y = 0; y < special_region_of_interest_2.rows; y++) {
                        if ( special_region_of_interest_2.at<unsigned char>(y,x) != 0 ) {
                            frame_special_region_of_interest_2.push_back(std::make_pair(cv::Point2f(x,y), gt_displacement_2));
                        }
                    }
                }

                if ( frame_special_region_of_interest_1.size() || frame_special_region_of_interest_2.size()) {
                    // working of dilation, erosion, thinning, and findNonZero
                    // this can be omitted. just to validate if the algorithm has worked. otherwise finalImage has no use.
                    cv::Mat finalImage(Dataset::m_frame_size, CV_8UC3);
                    cv::Mat dummyImage(Dataset::m_frame_size, CV_8UC1, cv::Scalar(0));
                    std::vector<cv::Mat> to_merge = {special_region_of_interest_1, special_region_of_interest_2, dummyImage};
                    cv::merge(to_merge, finalImage);
                    //cv::imshow("sroi", finalImage);
                    //cv::waitKey(0);
                }
                // frame_object_special_region_of_interest_1 will contain all the intersection pixels for each object combination.
                // Hence this list will grow in case a single object has interesection points with multiple objects.
                // back_inserter simply pushes back the values and is an easy way to tackle appending an array.

                std::copy(frame_special_region_of_interest_1.begin(), frame_special_region_of_interest_1.end(), std::back_inserter(frame_object_special_region_of_interest.at(list_of_gt_objects_combination.at(obj_combination_index).first->getObjectId())));

                std::copy(frame_special_region_of_interest_2.begin(), frame_special_region_of_interest_2.end(), std::back_inserter(frame_object_special_region_of_interest.at(list_of_gt_objects_combination.at(obj_combination_index).second->getObjectId())));

                // alternative and efficient method is stated above. Leaving this just for reference.
                /*
                for ( auto x = 0; x < frame_special_region_of_interest_2.size(); x++) {
                    frame_object_special_region_of_interest.at(list_of_gt_objects_combination.at(obj_combination_index).first->getObjectId()).push_back(frame_special_region_of_interest_2.at(x));
                }
                for ( auto x = 0; x < frame_special_region_of_interest_1.size(); x++) {
                    frame_object_special_region_of_interest.at(list_of_gt_objects_combination.at(obj_combination_index).second->getObjectId()).push_back(frame_special_region_of_interest_1.at(x));
                }
                */
                // The distance calculation is tailored for the dataset with a car and a ped. Not a general case.
                // To make a general case every corner needs to be considered.
                region_of_interest_px_str region_of_interest_px_1 = list_of_gt_objects_combination.at(obj_combination_index).first->getExtrapolatedGroundTruthDetails().at(sensor_index).at(current_frame_index).m_region_of_interest_px;

                region_of_interest_px_str region_of_interest_px_2 = list_of_gt_objects_combination.at(obj_combination_index).second->getExtrapolatedGroundTruthDetails().at(sensor_index).at(current_frame_index).m_region_of_interest_px;

                float x_distance = (float)cv::norm(cv::Point2f((region_of_interest_px_1.x + region_of_interest_px_1.width_px - region_of_interest_px_2.x), region_of_interest_px_1.y - region_of_interest_px_2.y));

                if ( region_of_interest_px_1.x < region_of_interest_px_2.x + region_of_interest_px_2.width_px ) {
                    std::cout << "distance between object name "
                              << list_of_gt_objects_combination.at(obj_combination_index).first->getObjectName() <<
                              " and object name "
                              << list_of_gt_objects_combination.at(obj_combination_index).second->getObjectName() <<
                              " = " << x_distance << std::endl;
                }

                cv::Mat check_intersection(Dataset::m_frame_size, CV_8UC3, cv::Scalar(255,255,255));
                cv::line(check_intersection, cv::Point2f((region_of_interest_px_1.x + region_of_interest_px_1.width_px), region_of_interest_px_1.y), cv::Point2f((region_of_interest_px_2.x), region_of_interest_px_2.y), cv::Scalar(255,0,0) );

                //cv::imshow("distance", check_intersection);
                //cv::waitKey(0);

                // occlusion image
                // occlusion boundary
            }

            cv::Mat tempImage(Dataset::m_frame_size, CV_8UC3);
            tempImage = cv::imread(gt_image_path, CV_LOAD_IMAGE_COLOR);

            for ( ushort obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++ ) {

                all_frame_object_special_region_of_interest.at(obj_index).push_back(
                        frame_object_special_region_of_interest.at(obj_index));


                std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > gt_roi_object = m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement();

                // validate special region of interest
                // sroi pixels
                // does eroi contains sroi coordinates? It should have because we are expanding eroi with new interpolated values. So, what is the final value?
                std::vector<std::pair<cv::Point2f, cv::Point2f> > intersection_of_gt_and_sroi;
                std::vector<std::pair<cv::Point2f, cv::Point2f> > dummy(gt_roi_object.at(sensor_index).at(current_frame_index).size());

                MyIntersection intersection;
                intersection.find_intersection_pair(gt_roi_object.at(sensor_index).at(current_frame_index).begin(), gt_roi_object.at(sensor_index).at(current_frame_index).end(), frame_object_special_region_of_interest.at(obj_index).begin(), frame_object_special_region_of_interest.at(obj_index).end());
                intersection_of_gt_and_sroi = intersection.getResultIntersectingPair();
                bool isSorted = std::is_sorted(gt_roi_object.at(sensor_index).at(current_frame_index).begin(), gt_roi_object.at(sensor_index).at(current_frame_index).end(), PairPointsSort<float>());
                assert(isSorted);
                bool isSorted_sroi = std::is_sorted(frame_object_special_region_of_interest.at(obj_index).begin(), frame_object_special_region_of_interest.at(obj_index).end(), PairPointsSort<float>());
                assert(isSorted_sroi);

                //assert(intersection_of_algorithm_and_sroi.size() > 0);

                // Validate
                for ( auto it = frame_object_special_region_of_interest.at(obj_index).begin(); it != frame_object_special_region_of_interest.at(obj_index).end(); it++) {
                    cv::circle(tempImage, (*it).first, 1, cv::Scalar(obj_index*255,255,0));
                }
            }

            //cv::imshow("gt_sroi", tempImage);
            //cv::waitKey(0);
            cv::destroyAllWindows();

        }

        for ( ushort obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++ ) {
            m_ptr_list_gt_objects.at(obj_index)->push_back_object_sroi(all_frame_object_special_region_of_interest.at(obj_index));
            m_ptr_list_gt_objects.at(obj_index)->push_back_object_sroi_interpolated(all_frame_object_special_region_of_interest.at(obj_index));
        }
        // -----
    }

    // special region of interest to be set to object ids in the variable m_object_sroi
    for ( ushort obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++ ) {

        // TODO: find complement of frame_object_special_region_of_interest_1 of an object id and all_points from groundtruth_object by writing a minus operator between the two types
        const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &groundtruthobject1_all_points = m_ptr_list_gt_objects.at(obj_index)->get_object_stencil_point_displacement();
        m_ptr_list_gt_objects.at(obj_index)->setDisjointSpecialRegionOfInterest(all_sensors_object_special_region_of_interest.at(obj_index));

    }

    // validate woth getSpecialRegionOfInterest

}

void GroundTruthFlow::find_ground_truth_object_contour_region_of_interest() {

    // Intersection between pair of objects. Total visible pixels is known. This metric will show how many
    // pixels lie on the occlusion boundary.
    // intersection coordinates, convert to Mat, fill with 0, and then find the contour. This is the occlusion boundary.
    // subtract area for evaluation from the object farther away.
    // total = visible part object front + visible part object back
    // boundary = common
    // invisible part = total object part back - visible part object back
    // object_stencil_displacement contains ground truth - all pixels are present in the vector
    // Intersection bounding box

    // depends on flow image and flow stencil

    // -----

    std::vector<Objects *> ptr_list_of_current_objects;
    std::vector<Objects *> ptr_list_of_copied_gt_objects;
    std::vector<Objects *> ptr_list_of_copied_simulated_objects;
    for ( auto i = 0; i < m_ptr_list_gt_objects.size(); i++) {
        ptr_list_of_copied_gt_objects.push_back(static_cast<Objects*>(m_ptr_list_gt_objects.at(i)));
    }

    if (m_opticalFlowName == "ground_truth") {
        ptr_list_of_current_objects = ptr_list_of_copied_gt_objects;
    } else {

        for ( auto i = 0; i < get_simulated_objects_ptr_list().size(); i++) {
            ptr_list_of_copied_simulated_objects.push_back(static_cast<Objects*>(get_simulated_objects_ptr_list().at(i)));
        }

        ptr_list_of_current_objects = ptr_list_of_copied_simulated_objects;
    }

    char sensor_index_folder_suffix[50];

    for (unsigned sensor_index = 0; sensor_index < Dataset::SENSOR_COUNT; sensor_index++) {


        unsigned FRAME_COUNT = (unsigned) m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(
                sensor_index).size();
        assert(FRAME_COUNT > 0);

        cv::Mat image_02_frame = cv::Mat::zeros(Dataset::m_frame_size, CV_32FC3);
        sprintf(sensor_index_folder_suffix, "%02d", m_evaluation_list.at(sensor_index));

        std::cout << "begin generating ground truth object contours" << sensor_index_folder_suffix << std::endl;

        std::vector<std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > > all_frame_object_contour_region_of_interest(m_ptr_list_gt_objects.size());

        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {


            std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > frame_object_contour_region_of_interest(m_ptr_list_gt_objects.size());

            char file_name_input_image[50];
            ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                    (0).at(current_frame_index).frame_no;

            sprintf(file_name_input_image, "000%03d_10.png", image_frame_count);
            std::string flow_path = m_generatepath.parent_path().string() + "/flow_occ_" + sensor_index_folder_suffix + "/" + file_name_input_image;
            //std::string kitti_path = m_plots_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
            std::string gt_image_path = m_GroundTruthImageLocation.string() + "_" + sensor_index_folder_suffix + "/" + file_name_input_image;

            std::cout << "current_frame_index  " << current_frame_index << std::endl;

            cv::Mat flow_image = cv::imread(flow_path, CV_LOAD_IMAGE_UNCHANGED);
            cv::cvtColor(flow_image, flow_image, CV_RGB2BGR);
            if ( flow_image.empty() ) {
                throw("No image found error");
            }

            cv::Mat intersection_image(Dataset::m_frame_size, CV_MAKE_TYPE(flow_image.depth(),1));
            int from_to[] = { 2,0 };  // copy the third channel ( channel 2 object id ) to the first channel of intersection_image
            cv::mixChannels(flow_image, intersection_image, from_to, 1); // the last parameter is the number of pairs

            for ( ushort obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++ ) {

                cv::Point2f gt_displacement_1 = m_ptr_list_gt_objects.at(obj_index)->get_object_extrapolated_point_displacement().at(sensor_index).at(current_frame_index).second;

                std::vector<std::pair<cv::Point2f, cv::Point2f> > frame_special_region_of_interest;

                cv::Mat mask_object;
                cv::Mat mask_object_eroded, mask_object_eroded_pre;

                ushort val = (ushort)(m_ptr_list_gt_objects.at(obj_index)->getObjectId() * 64 + 32768) ;

                cv::inRange(intersection_image, val, val, mask_object);

                std::vector<cv::Mat> contours;

                cv::Mat mask_object_new;
                cv::medianBlur ( mask_object, mask_object_new, 11);
                //cv::bilateralFilter( mask_object, mask_object_new, 19, 7, 7, 4);
                //cv::fastNlMeansDenoising(mask_object, mask_object_new, 10, 21, 51 );
                cv::imshow("denoise", mask_object_new);
                cv::imshow("noise", mask_object);
                //cv::waitKey(0);
                cv::destroyAllWindows();
                //cv::fastNlMeansDenoising(mask_object_2, mask_object_2); //, float h=3, int templateWindowSize=7, int searchWindowSize=21 )¶

                mask_object_eroded = mask_object_new.clone();
                cv::Mat sectioned_contours;

                std::vector<std::vector<cv::Point> > contours_vector;
                cv::findContours(mask_object, contours_vector, CV_RETR_LIST, CV_CHAIN_APPROX_NONE,
                        cv::Point(cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(
                                current_frame_index).m_object_location_camera_px.location_x_px),
                                cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(
                                        current_frame_index).m_object_location_camera_px.location_y_px)));


                cv::Mat contourImage(mask_object.size(), CV_8UC3, cv::Scalar(0,0,0));
                for ( ushort contour_index = 0; contour_index < contours_vector.size(); contour_index++) {
                    cv::drawContours(contourImage, contours, contour_index, cv::Scalar(255,255,255));
                    // cv2.drawContours(des,[cnt],0,255,-1)
                    break;
                }

                cv::imshow("con", contourImage);
                cv::waitKey(0);

                cv::Mat results;
                do {

                    mask_object_eroded_pre = mask_object_eroded.clone();

                    for (int i = 0; i < 5; i++) {
                        cv::erode(mask_object_eroded, mask_object_eroded, cv::Mat());
                    }

                    cv::compare(mask_object_eroded, mask_object_eroded_pre, results, CV_CMP_NE);
                    contours.push_back(results.clone());

                    //cv::imshow("final", results);
                    //cv::waitKey(0);

                } while (cv::countNonZero(results) > 1);

                std::cout << "number of contours in the object = " << contours.size() << std::endl;

                std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > frame_contour_region_of_interest(contours.size());

                for ( ushort object_contour_index = 0; object_contour_index < contours.size(); object_contour_index++) {
                    if ( contours.at(object_contour_index).data == NULL  ) {
                        throw;
                    }
                    //cv::imshow("contour", contours.at(object_contour_index));
                    //cv::waitKey(0);
                    for ( auto x = 0; x < results.cols; x++ ) {
                        for ( auto y = 0; y < results.rows; y++) {
                        // there is only one object per Mat. Hence we can safely scan the whole frame.
                            if ( contours.at(object_contour_index).at<char>(y,x) != 0 ) {
                                frame_contour_region_of_interest.at(object_contour_index).push_back(std::make_pair(cv::Point2f(x,y), gt_displacement_1));
                            }
                        }
                    }
                }

                frame_object_contour_region_of_interest.at(obj_index) = frame_contour_region_of_interest;

            }

            for ( ushort obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++ ) {

                all_frame_object_contour_region_of_interest.at(obj_index).push_back(
                        frame_object_contour_region_of_interest.at(obj_index));

            }
        }


        for ( ushort obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++ ) {

            m_ptr_list_gt_objects.at(obj_index)->push_back_object_croi(all_frame_object_contour_region_of_interest.at(obj_index));

        }
    }
}


/*
        //cv::Vec3f *datagt_next_ptsr = flowFrame.gt_next_ptsr<cv::Vec3f>(0); // pointer to the first channel of the first element in the
        // first row. The r, g b  value of single pixels are continous.
        float *array = (float *)malloc(3*sizeof(float)*Dataset::m_frame_size.width*Dataset::m_frame_size.height);
        cv::MatConstIterator_<cv::Vec3f> it = roi.begin<cv::Vec3f>();
        for (unsigned i = 0; it != roi.end<cv::Vec3f>(); it++ ) {
            for ( unsigned j = 0; j < 3; j++ ) {
                *(array + i ) = (*it)[j];
                i++;
            }
        }
        FlowImageExtended temp = FlowImageExtended(array, Dataset::m_frame_size.width, Dataset::m_frame_size.height );
        F_png_write = temp;

 */


