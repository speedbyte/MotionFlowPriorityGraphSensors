#include <opencv/cv.hpp>
#include "OpticalFlow.h"
#include "SortandIntersect.h"


void OpticalFlow::find_ground_truth_flow_occlusion_boundary(ushort SENSOR_COUNT) {

    // Intersection between pair of objects. Total visible pixels is known. This metric will show how many
    // pixels lie on the occlusion boundary.
    // intersection coordinates, convert to Mat, fill with 0, and then find the contour. This is the occlusion boundary.
    // subtract area for evaluation from the object farther away.
    // total = visible part object front + visible part object back
    // boundary = common
    // invisible part = total object part back - visible part object back
    // frame_stencil_displacement contains ground truth - all pixels are present in the vector
    // Intersection bounding box

    // -----


    std::vector<Objects *> list_of_current_objects;
    std::vector<Objects *> ptr_list_of_derived_objects;
    for ( auto i = 0; i < m_ptr_list_gt_objects.size(); i++) {
        ptr_list_of_derived_objects.push_back(static_cast<Objects*>(m_ptr_list_gt_objects.at(i)));
    }

    if (m_opticalFlowName == "ground_truth") {
        list_of_current_objects = ptr_list_of_derived_objects;
    } else {
        list_of_current_objects = m_ptr_list_simulated_objects;
    }

    char sensor_index_folder_suffix[50];

    std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > > all_sensors_object_special_region_of_interest(m_ptr_list_gt_objects.size());

    for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {


        unsigned FRAME_COUNT = (unsigned) m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(
                sensor_index).size();
        assert(FRAME_COUNT > 0);

        cv::Mat image_02_frame = cv::Mat::zeros(Dataset::m_frame_size, CV_32FC3);
        sprintf(sensor_index_folder_suffix, "%02d", m_evaluation_list.at(sensor_index));

        std::cout << "begin distance calculation between objects" << sensor_index_folder_suffix << std::endl;

        std::vector<std::vector<std::vector<cv::Point2f> > > all_frame_object_special_region_of_interest(m_ptr_list_gt_objects.size());


        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

            std::vector<std::vector<cv::Point2f> > frame_object_special_region_of_interest_1(m_ptr_list_gt_objects.size());
            std::vector<std::vector<cv::Point2f> > frame_object_special_region_of_interest_2(m_ptr_list_gt_objects.size());

            char file_name_input_image[50];
            ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                    (0).at(current_frame_index).frame_no;

            sprintf(file_name_input_image, "000%03d_10.png", image_frame_count);
            std::string flow_path = m_flow_occ_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
            //std::string kitti_path = m_plots_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;

            std::cout << "current_frame_index  " << current_frame_index << std::endl;

            std::vector<std::pair<Objects *, Objects *> > list_of_current_objects_combination;
            std::vector<std::pair<Objects *, Objects *> > list_of_gt_objects_combination;
            std::vector<std::pair<Objects *, Objects *> > list_of_simulated_objects_combination;


            if (m_opticalFlowName == "ground_truth") {
                getCombination(ptr_list_of_derived_objects, list_of_gt_objects_combination);
                list_of_current_objects_combination = list_of_gt_objects_combination;

            } else {
                getCombination(m_ptr_list_simulated_objects, list_of_simulated_objects_combination);
                list_of_current_objects_combination = list_of_simulated_objects_combination;
            }

            cv::Mat flow_image = cv::imread(flow_path, CV_LOAD_IMAGE_COLOR);
            cv::Mat intersection_image(Dataset::m_frame_size, CV_8UC1);
            int from_to[] = { 2,0 };
            cv::mixChannels(flow_image, intersection_image, from_to, 1);



            for (ushort obj_combination_index = 0;
                 obj_combination_index < list_of_current_objects_combination.size(); obj_combination_index++) {
                std::cout << "distance between object name "
                          << list_of_current_objects_combination.at(obj_combination_index).first->getObjectName() <<
                          " and object name "
                          << list_of_current_objects_combination.at(obj_combination_index).second->getObjectName()
                          << "\n";

                std::vector<cv::Point2f> frame_special_region_of_interest_1;
                std::vector<cv::Point2f> frame_special_region_of_interest_2;


                const std::vector<std::pair<cv::Point2f, cv::Point2f>> &groundtruthobject1 = list_of_current_objects_combination.at(
                        obj_combination_index).first->get_object_stencil_point_displacement().at
                        (sensor_index).at(current_frame_index);

                const std::vector<std::pair<cv::Point2f, cv::Point2f>> &groundtruthobject2 = list_of_current_objects_combination.at(
                        obj_combination_index).second->get_object_stencil_point_displacement().at
                        (sensor_index).at(current_frame_index);


                // This is tailored for the dataset with a car and a ped. Not a general case
                region_of_interest_px_str region_of_interest_px_1 = list_of_current_objects_combination.at(obj_combination_index).first->getExtrapolatedGroundTruthDetails().at(sensor_index).at(current_frame_index).m_region_of_interest_px;

                region_of_interest_px_str region_of_interest_px_2 = list_of_current_objects_combination.at(obj_combination_index).second->getExtrapolatedGroundTruthDetails().at(sensor_index).at(current_frame_index).m_region_of_interest_px;

                float x_distance = (float)cv::norm(cv::Point2f((region_of_interest_px_1.x + region_of_interest_px_1.width_px - region_of_interest_px_2.x), region_of_interest_px_1.y - region_of_interest_px_2.y));

                cv::Mat mask_object_1, mask_object_2;
                cv::Mat mask_object_1_dilated, mask_object_2_dilated;

                ushort val_1 = (ushort)(list_of_current_objects_combination.at(obj_combination_index).first->getObjectId() + 127);
                ushort val_2 = (ushort)(list_of_current_objects_combination.at(obj_combination_index).second->getObjectId() + 127);

                cv::inRange(intersection_image, val_1, val_1, mask_object_1);
                cv::inRange(intersection_image, val_2, val_2, mask_object_2);

                mask_object_1_dilated = mask_object_1.clone();
                mask_object_2_dilated = mask_object_2.clone();

                for (int i=0; i<5; i++) {
                    cv::dilate(mask_object_1_dilated, mask_object_1_dilated, cv::Mat());
                    cv::dilate(mask_object_2_dilated, mask_object_2_dilated, cv::Mat());
                }

                cv::Mat final = mask_object_1_dilated & mask_object_2_dilated;

                cv::Mat boundary = final & mask_object_2;
                cv::Mat special_region_of_interest_1 = mask_object_2 & mask_object_1_dilated;
                cv::Mat special_region_of_interest_2 = mask_object_1 & mask_object_2_dilated;

                // working of dilation, erosion, thinning, and findNonZero

                cv::Mat finalImage(Dataset::m_frame_size, CV_8UC3, cv::Scalar(0,0,0));
                cv::Mat dummyImage(Dataset::m_frame_size, CV_8UC1, cv::Scalar(0));
                std::vector<cv::Mat> to_merge = {special_region_of_interest_1, special_region_of_interest_2, dummyImage};
                cv::merge(to_merge, finalImage);

                //cv::imshow("intersection_1", special_region_of_interest_1);
                //cv::imshow("intersection_2", special_region_of_interest_2);
                //cv::imshow("merged", finalImage);
                //cv::waitKey(0);

                if ( region_of_interest_px_1.x < region_of_interest_px_2.x + region_of_interest_px_2.width_px ) {
                    std::cout << "distance betwen objects = " << x_distance << std::endl;
                }

                for ( auto x = 0; x < special_region_of_interest_1.cols; x++ ) {
                    for ( auto y = 0; y < special_region_of_interest_1.rows; y++) {
                        if ( special_region_of_interest_1.at<unsigned char>(y,x) != 0 ) {
                            frame_special_region_of_interest_1.push_back(cv::Point2f(x,y));
                        }
                    }
                }

                for ( auto x = 0; x < special_region_of_interest_2.cols; x++ ) {
                    for ( auto y = 0; y < special_region_of_interest_2.rows; y++) {
                        if ( special_region_of_interest_2.at<unsigned char>(y,x) != 0 ) {
                            frame_special_region_of_interest_2.push_back(cv::Point2f(x,y));
                        }
                    }
                }

                /*
                for ( auto x = 0; x < frame_special_region_of_interest_1.size(); x++) {
                    frame_object_special_region_of_interest_1.at(list_of_current_objects_combination.at(obj_combination_index).first->getObjectId()).push_back(frame_special_region_of_interest_1.at(x));
                }
                for ( auto x = 0; x < frame_special_region_of_interest_2.size(); x++) {
                    frame_object_special_region_of_interest_1.at(list_of_current_objects_combination.at(obj_combination_index).second->getObjectId()).push_back(frame_special_region_of_interest_2.at(x));
                }
                 */

                std::copy(frame_special_region_of_interest_1.begin(), frame_special_region_of_interest_1.end(), std::back_inserter(frame_object_special_region_of_interest_1.at(list_of_current_objects_combination.at(obj_combination_index).first->getObjectId())));

                std::copy(frame_special_region_of_interest_2.begin(), frame_special_region_of_interest_2.end(), std::back_inserter(frame_object_special_region_of_interest_1.at(list_of_current_objects_combination.at(obj_combination_index).second->getObjectId())));


                MyIntersection myIntersection_gt_object_pairs;

                std::vector<std::pair<cv::Point2f, cv::Point2f>> intersection_ground_truth_objects(
                        groundtruthobject1.size());
                intersection_ground_truth_objects.clear();

                myIntersection_gt_object_pairs.find_intersection_pair(groundtruthobject1.begin(), groundtruthobject1.end(),
                                                                 groundtruthobject2.begin(), groundtruthobject2.end(),
                                                                 intersection_ground_truth_objects.begin());
                intersection_ground_truth_objects = myIntersection_gt_object_pairs.getResultPair();
                //myIntersection_gt_object_pairs.showResult();
                std::cout << "occlusion intersection between two objects" << intersection_ground_truth_objects.size() << std::endl;

                cv::Mat check_intersection(Dataset::m_frame_size, CV_8UC3, cv::Scalar(255,255,255));

                cv::line(check_intersection, cv::Point2f((region_of_interest_px_1.x + region_of_interest_px_1.width_px), region_of_interest_px_1.y), cv::Point2f((region_of_interest_px_2.x), region_of_interest_px_2.y), cv::Scalar(255,0,0) );

                for ( auto it = intersection_ground_truth_objects.begin(); it != intersection_ground_truth_objects.end(); it++) {
                    cv::circle(check_intersection, (*it).first, 1, cv::Scalar(0,255,0));
                }

                //cv::imshow("int", check_intersection);
                //cv::waitKey(0);

                // occlusion image
                // occlusion boundary
            }
            for ( ushort obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++ ) {

                all_frame_object_special_region_of_interest.at(obj_index).push_back(
                        frame_object_special_region_of_interest_1.at(obj_index));
            }
        }

        for ( ushort obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++ ) {
            all_sensors_object_special_region_of_interest.at(obj_index).push_back(all_frame_object_special_region_of_interest.at(obj_index));
        }
        // -----
    }
    // special region of interest to be set to object ids in the variable m_special_region_of_interest
    for ( ushort obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++ ) {
        m_ptr_list_gt_objects.at(obj_index)->setSpecialRegionOfInterest(all_sensors_object_special_region_of_interest.at(obj_index));
    }

    // validate woth getSpecialRegionOfInterest

}


