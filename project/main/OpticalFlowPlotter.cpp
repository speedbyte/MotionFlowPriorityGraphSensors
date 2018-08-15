#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>
#include "OpticalFlow.h"


void OpticalFlow::plot_stencil(ushort SENSOR_COUNT) {

    std::vector<Objects*> list_of_current_objects;

    unsigned COUNT;
    if ( m_opticalFlowName == "ground_truth") {
        list_of_current_objects = m_ptr_list_gt_objects;
        COUNT = 1;
    }
    else {
        list_of_current_objects = m_ptr_list_simulated_objects;
        COUNT = (unsigned)list_of_current_objects.at(
                0)->get_list_object_dataprocessing_mean_centroid_displacement().size();
    }

    char sensor_index_folder_suffix[50];

    std::cout << "visualise stencil algorithm at " << m_generatepath.string() + "stencil/" << std::endl;


    cv::Mat image_data_and_shape;

    cv::Mat tempGroundTruthImage(Dataset::m_frame_size, CV_8UC3);
    FlowImageExtended F_png_write;

    ushort datafilter_index = 0;

    for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

        tempGroundTruthImage = cv::Scalar::all(255);
        F_png_write = FlowImageExtended(Dataset::m_frame_size.width, Dataset::m_frame_size.height);

        std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > sensor_frame_shape_points;
        std::map<std::pair<float, float>, int> scenario_displacement_occurence;

        sprintf(sensor_index_folder_suffix, "%02d", sensor_index);

        unsigned FRAME_COUNT = (unsigned) list_of_current_objects.at(0)
                ->get_object_stencil_point_displacement().at(sensor_index).size();

        assert(FRAME_COUNT > 0);

        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

            char file_name_image_output[50];
            std::string output_image_file_with_path;
            ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                    (0).at(current_frame_index).frame_no;
            sprintf(file_name_image_output, "000%03d_10.png", image_frame_count);
            output_image_file_with_path = m_plots_path.string() + sensor_index_folder_suffix + "/" + file_name_image_output;

            //---------------------------------------------------------------------------------
            tempGroundTruthImage = cv::imread(output_image_file_with_path, cv::IMREAD_COLOR);
            if ( tempGroundTruthImage.data == NULL ) {
                std::cout << "no image found, exiting" << std::endl;
                throw;
            }

            std::cout << "current_frame_index " << current_frame_index << " for datafilter_index " << datafilter_index<< std::endl;

            for (ushort obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++) {

                if (list_of_current_objects.at(obj_index)->get_object_extrapolated_visibility().at(sensor_index).at(current_frame_index) ) {

                    cv::Point2f pts_gt = m_ptr_list_gt_objects.at(
                            obj_index)->get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index).at(sensor_index).at(current_frame_index).mean_pts;

                    cv::Point2f mean_displacement_gt = m_ptr_list_gt_objects.at(
                            obj_index)->get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index).at(sensor_index).at(current_frame_index).mean_displacement;

                    cv::Point2f next_pts = cv::Point2f(pts_gt.x + mean_displacement_gt.x*10, pts_gt.y + mean_displacement_gt.y*10);

                    cv::arrowedLine(tempGroundTruthImage, pts_gt, next_pts, cv::Scalar(0,0,255), 2);

                    if (m_opticalFlowName == "ground_truth") {


                    }
                    else {

                        cv::Point2f algo_pts = m_ptr_list_simulated_objects.at(
                                obj_index)->get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index).at(sensor_index).at(current_frame_index).mean_pts;

                        cv::Point2f algo_displacement = m_ptr_list_simulated_objects.at(
                                obj_index)->get_list_object_dataprocessing_mean_centroid_displacement().at(datafilter_index).at(sensor_index).at(current_frame_index).mean_displacement;

                        cv::Point2f next_pts = cv::Point2f(algo_pts.x + algo_displacement.x*10, algo_pts.y + algo_displacement.y*10);

                        cv::arrowedLine(tempGroundTruthImage, algo_pts, next_pts, cv::Scalar(0, 255, 0), 2);

                    }
                }
            }
            cv::imshow("ww", tempGroundTruthImage);
            cv::waitKey(100);
            cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
        }
    }
}

