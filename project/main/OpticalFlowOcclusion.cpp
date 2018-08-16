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
    for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

        unsigned FRAME_COUNT = (unsigned) m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(
                sensor_index).size();
        assert(FRAME_COUNT > 0);

        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

            std::cout << "current_frame_index " << current_frame_index << std::endl;

            std::vector<std::pair<Objects *, Objects *> > list_of_current_objects_combination;
            std::vector<std::pair<Objects *, Objects *> > list_of_gt_objects_combination;
            std::vector<std::pair<Objects *, Objects *> > list_of_simulated_objects_combination;

            if (m_opticalFlowName == "ground_truth") {
                getCombination(m_ptr_list_gt_objects, list_of_gt_objects_combination);
                list_of_current_objects_combination = list_of_gt_objects_combination;

            } else {
                getCombination(m_ptr_list_simulated_objects, list_of_simulated_objects_combination);
                list_of_current_objects_combination = list_of_simulated_objects_combination;
            }

            for (ushort obj_combination_index = 0;
                 obj_combination_index < list_of_current_objects_combination.size(); obj_combination_index++) {
                std::cout << "collision between object name "
                          << list_of_current_objects_combination.at(obj_combination_index).first->getObjectName() <<
                          " and object name "
                          << list_of_current_objects_combination.at(obj_combination_index).second->getObjectName()
                          << "\n";

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

                std::cout << "distance betwen objects = " << x_distance;

                cv::Mat image_02_frame(Dataset::m_frame_size, CV_8UC3, cv::Scalar(255,255,255));

                MyIntersection myIntersection_gt_object_pairs;

                std::vector<std::pair<cv::Point2f, cv::Point2f>> intersection_ground_truth_objects(
                        groundtruthobject1.size());
                intersection_ground_truth_objects.clear();

                myIntersection_gt_object_pairs.find_intersection(groundtruthobject1.begin(), groundtruthobject1.end(),
                                                                 groundtruthobject2.begin(), groundtruthobject2.end(),
                                                                 intersection_ground_truth_objects.begin());
                intersection_ground_truth_objects = myIntersection_gt_object_pairs.getResult();
                //myIntersection_gt_object_pairs.showResult();
                std::cout << "occlusion intersection between two objects " << intersection_ground_truth_objects.size() << std::endl;

                cv::Mat check_intersection(Dataset::m_frame_size, CV_8UC3, cv::Scalar(255,255,255));

                cv::line(check_intersection, cv::Point2f((region_of_interest_px_1.x + region_of_interest_px_1.width_px), region_of_interest_px_1.y), cv::Point2f((region_of_interest_px_2.x), region_of_interest_px_2.y), cv::Scalar(255,0,0) );

                for ( auto it = groundtruthobject1.begin(); it != groundtruthobject1.end(); it++) {
                    cv::circle(check_intersection, (*it).first, 1, cv::Scalar(255,0,0));
                }

                for ( auto it = groundtruthobject2.begin(); it != groundtruthobject2.end(); it++) {
                    cv::circle(check_intersection, (*it).first, 1, cv::Scalar(0,0,255));
                }

                cv::imshow("int", check_intersection);
                cv::waitKey(0);

                // occlusion image
                // occlusion boundary
            }

        }
        // -----
    }
}


