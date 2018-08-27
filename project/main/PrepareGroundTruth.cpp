
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

#include "datasets.h"
#include "PrepareGroundTruth.h"
#include "kbhit.h"

#include <unordered_map>
#include <bits/unordered_map.h>

#include "Dataset.h"
#include "GroundTruthScene.h"
#include "SortandIntersect.h"


//Creating a movement path. The path is stored in a x and y vector

using namespace std::chrono;

void PrepareGroundTruth::prepare_directories(ushort SENSOR_COUNT, std::string noise, ushort fps, ushort stepSize) {

    m_GroundTruthImageLocation = Dataset::m_dataset_gtpath.string() + "/blue_sky";

    m_resultordner="/ground_truth";

    m_generatepath = Dataset::m_dataset_gtpath.string() + m_resultordner;

    if (!Dataset::m_dataset_basepath.compare(CPP_DATASET_PATH) || !Dataset::m_dataset_basepath.compare(VIRES_DATASET_PATH)) {

        prepare_directories_common(SENSOR_COUNT);

    }
}


void PrepareGroundTruth::generate_flow_vector(ushort SENSOR_COUNT) {

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

        std::cout << "saving " + m_resultordner + " flow files in flow/ for sensor_index  " << sensor_index << std::endl;

        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

            std::cout << "current_frame_index " << current_frame_index << std::endl;

            char file_name_image_output[50];
            ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                    (0).at(current_frame_index).frame_no;
            sprintf(file_name_image_output, "000%03d_10.png", image_frame_count);

            std::string frame_difference_path = Dataset::m_dataset_gtpath.string() + "/frame_difference_"  + sensor_index_folder_suffix + "/" + file_name_image_output;
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
                              multiframe_stencil_displacement, multiframe_visibility, all_moving_objects_in_frame);

        }

        for (ushort obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++) {

            m_ptr_list_gt_objects.at(obj_index)->push_back_object_stencil_point_displacement_pixel_visibility(
                    multiframe_stencil_displacement.at(obj_index), multiframe_visibility.at(obj_index));
        }

        cv::destroyAllWindows();
    }

    std::cout << "end of saving " + m_resultordner + " flow files in an vector" << std::endl;

}

void PrepareGroundTruth::generate_depth_images(ushort SENSOR_COUNT) {

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the position in a png file

    auto tic_all = steady_clock::now();

    char sensor_index_folder_suffix[50];
    for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

        unsigned FRAME_COUNT = (unsigned)m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(sensor_index).size();
        assert(FRAME_COUNT>0);
        cv::Mat depth_02_frame;
        sprintf(sensor_index_folder_suffix, "%02d", m_evaluation_list.at(sensor_index));
        std::cout << "start depth files for sensor_index  " << sensor_index_folder_suffix << std::endl;

        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

            char file_name_input_image_depth[50];
            std::cout << "current_frame_index " << current_frame_index << std::endl;
            ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                    (0).at(current_frame_index).frame_no;
            sprintf(file_name_input_image_depth, "depth_000%03d_10.png", image_frame_count);
            std::string input_image_path = m_GroundTruthImageLocation.string() + "_" + sensor_index_folder_suffix + "/" + file_name_input_image_depth;
            depth_02_frame = cv::imread(input_image_path, CV_LOAD_IMAGE_ANYCOLOR);

            if ( depth_02_frame.data == NULL ) {
                std::cerr << input_image_path << " not found" << std::endl;
                throw ("No image file found error");
            }

            ushort obj_index = 1;
            printf("validating depth image for object id %d - depth %u at cog %u %u\n", m_ptr_list_gt_objects.at(obj_index)->getObjectId(), depth_02_frame.at<unsigned char>(cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(m_evaluation_list.at(sensor_index)).at(current_frame_index).m_object_location_camera_px.cog_px.y), cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(m_evaluation_list.at(sensor_index)).at(current_frame_index).m_object_location_camera_px.cog_px.x)), cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(m_evaluation_list.at(sensor_index)).at(current_frame_index).m_object_location_camera_px.cog_px.x), cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(m_evaluation_list.at(sensor_index)).at(current_frame_index).m_object_location_camera_px.cog_px.y));

            //cv::imshow("depth_image", depth_02_frame);
            //cv::waitKey(0);
        }
    }

    std::cout << "end of validating ground truth depth files " << std::endl;

}

void PrepareGroundTruth::find_ground_truth_flow_occlusion_boundary(ushort SENSOR_COUNT) {

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


