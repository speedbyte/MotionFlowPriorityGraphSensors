//
// Created by veikas on 08.02.18.
//

#include <iostream>
#include <map>
#include "Objects.h"
#include "Utils.h"
#include "DataProcessingAlgorithm.h"
#include <chrono>

void Objects::push_back_object_stencil_point_displacement_pixel_visibility(
        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > sensor_stencil_movement,  std::vector<std::vector<bool> > sensor_stencil_visibility ) {

    m_object_stencil_point_displacement.push_back(sensor_stencil_movement);
    m_object_stencil_visibility.push_back(sensor_stencil_visibility);

}

void Objects::push_back_object_interpolated_stencil_point_displacement_pixel_visibility(
        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > sensor_stencil_movement,  std::vector<std::vector<bool> > sensor_stencil_visibility ) {

    m_object_interpolated_stencil_point_displacement.push_back(sensor_stencil_movement);
    m_object_interpolated_stencil_visibility.push_back(sensor_stencil_visibility);

}

void Objects::push_back_object_sroi(
        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > sensor_stencil_movement) {

    m_object_sroi.push_back(sensor_stencil_movement);

}

void Objects::push_back_object_sroi_interpolated(
        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > sensor_stencil_movement) {

    m_object_sroi_interpolated.push_back(sensor_stencil_movement);

}

void Objects::assign_object_stencil_point_displacement_pixel_visibility(
        std::vector<std::pair<cv::Point2f, cv::Point2f> > sensor_stencil_movement,  std::vector<bool>  sensor_stencil_visibility, ushort sensor_index, ushort current_frame_index ) {

    m_object_stencil_point_displacement.at(sensor_index).at(current_frame_index) = sensor_stencil_movement;
    m_object_stencil_visibility.at(sensor_index).at(current_frame_index) = sensor_stencil_visibility;

}

void Objects::push_back_object_stencil_point_disjoint_displacement_pixel_visibility(
        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > sensor_stencil_disjoint_movement,  std::vector<std::vector<bool> > sensor_stencil_disjoint_visibility ) {

    m_object_stencil_point_disjoint_displacement.push_back(sensor_stencil_disjoint_movement);
    //m_object_stencil_visibility.push_back(sensor_stencil_visibility);

}

void Objects::push_back_object_interpolated_stencil_point_disjoint_displacement_pixel_visibility(
        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > sensor_stencil_disjoint_movement,  std::vector<std::vector<bool> > sensor_stencil_disjoint_visibility ) {

    m_object_interpolated_stencil_point_disjoint_displacement.push_back(sensor_stencil_disjoint_movement);
    //m_object_stencil_visibility.push_back(sensor_stencil_visibility);

}



void Objects::generate_edge_contour(std::string post_processing_algorithm) {

    for ( int sensor_index = 0; sensor_index < Dataset::SENSOR_COUNT; sensor_index++ ) {

        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > sensor_edge_movement;

        char sensor_index_folder_suffix[50];
        char file_name_input_image[50];

        sprintf(sensor_index_folder_suffix, "%02d", sensor_index);

        std::string temp_result_edge_path;

        std::vector<std::pair<cv::Point2f, cv::Point2f> > next_pts_array;

        std::cout << "making a edge contour on the basis of " << post_processing_algorithm << " for object " << m_objectName << " for sensor index " << sensor_index << std::endl;

        for (ushort current_frame_index=0; current_frame_index < Dataset::MAX_ITERATION_RESULTS; current_frame_index++) {
            //draw new ground truth flow.


            cv::Mat objectEdgeFrame( Dataset::m_frame_size, CV_8UC1 );
            objectEdgeFrame = cv::Scalar_<char>(0);

            ushort vires_frame_count = m_object_extrapolated_all.at(0).at(current_frame_index).frame_no;
            sprintf(file_name_input_image, "000%03d_10.png", vires_frame_count);

            temp_result_edge_path = Dataset::m_dataset_gtpath.string() + "ground_truth/edge_" + sensor_index_folder_suffix + "/" + file_name_input_image;

            cv::Mat edge_02_frame = cv::imread(temp_result_edge_path, CV_LOAD_IMAGE_COLOR);
            if ( edge_02_frame.data == NULL ) {
                std::cerr << temp_result_edge_path << " not found" << std::endl;
                throw ("No image file found error");
            }

            cv::Mat edge_02_frame_gray;
            cv::cvtColor(edge_02_frame, edge_02_frame_gray, CV_RGB2GRAY);

            // Calculate optical generate_flow_frame map using LK algorithm
            std::cout << "current_frame_index " << current_frame_index << std::endl;

            if ( current_frame_index > 0 ) {

                    objectEdgeFrame = cv::Scalar_<char>(0);
                    bool visibility = m_object_extrapolated_visibility.at(sensor_index).at(current_frame_index);
                    if ( visibility ) {

                        // This is for the base model
                        std::vector<std::pair<cv::Point2f, cv::Point2f> >  edge_movement;

                        next_pts_array = m_object_stencil_point_displacement.at(sensor_index).at(current_frame_index);

                        //std::cout << roi_offset.x + col_index << std::endl;
                        auto COUNT = next_pts_array.size();
                        std::cout << "base count " << COUNT << std::endl;
                        for ( ushort next_pts_index = 0; next_pts_index < COUNT; next_pts_index++ ) {

                            //printf("gray %u\n", edge_02_frame_gray.at<char>(cvRound(next_pts_array.at(next_pts_index).first.y), cvRound(next_pts_array.at(next_pts_index).first.x)));
                            if ( edge_02_frame_gray.at<char>(cvRound(next_pts_array.at(next_pts_index).first.y), cvRound(next_pts_array.at(next_pts_index).first.x)) != 0 ) {

                                objectEdgeFrame.at<char>(cvRound(next_pts_array.at(next_pts_index).first.y), cvRound(next_pts_array.at(next_pts_index).first.x)) = 255;
                                edge_movement.push_back(
                                        std::make_pair(next_pts_array.at(next_pts_index).first,
                                                       next_pts_array.at(next_pts_index).second));
                                //std::cout << "jayy " << next_pts_array.at(next_pts_index).first.x << " " << next_pts_array.at(next_pts_index).first.y << std::endl;

                            }
                            else {
                                //std::cout << "nopes " << next_pts_array.at(next_pts_index).first.x << " " << next_pts_array.at(next_pts_index).first.y << std::endl;
                            }
                        }

                        //cv::namedWindow("edge", CV_WINDOW_AUTOSIZE);
                        //cv::imshow("edge", objectEdgeFrame);
                        //cv::waitKey(0);
                        cv::destroyAllWindows();


                        auto edge_points = edge_movement.size();
                        std::cout << edge_points << std::endl;
                        //assert(edge_points != 0);

                        sensor_edge_movement.push_back(edge_movement);

                    }
                    else {
                            sensor_edge_movement.push_back(
                                    {{std::make_pair(cv::Point2f(0, 0), cv::Point2f(0, 0))}});
                    }
            }
            else {
                    sensor_edge_movement.push_back(
                            {{std::make_pair(cv::Point2f(0, 0), cv::Point2f(0, 0))}});
            }
        }

        m_object_edge_point_displacement.push_back(sensor_edge_movement);
    }
}

void Objects::generate_object_mean_centroid_displacement(std::string post_processing_algorithm) {

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            multiframe_dataprocessing_stencil_points_displacement_sensor_fusion_mean;

    std::vector<std::pair<cv::Point2f, cv::Point2f> >
            frame_dataprocessing_stencil_points_displacement_sensor_fusion_mean;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            sensor_multiframe_centroid_displacement_sensor_fusion_mean;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            sensor_multiframe_dataprocessing_stencil_points_displacement_sensor_fusion_mean;

    if ( Dataset::m_dataprocessing_map["NoAlgorithm"]) {
        NoAlgorithm noAlgorithm;
        noAlgorithm.common(this, post_processing_algorithm);
        m_list_object_dataprocessing_mean_centroid_displacement.push_back(noAlgorithm.get_object_dataprocessing_mean_centroid_displacement());
        m_list_object_dataprocessing_stencil_points_displacement.push_back(noAlgorithm.get_object_dataprocessing_stencil_point_displacement());
    }

    if ( Dataset::m_dataprocessing_map["SimpleAverage"]) {
        SimpleAverage simpleAverage;
        simpleAverage.common(this, post_processing_algorithm);
        m_list_object_dataprocessing_mean_centroid_displacement.push_back(
                simpleAverage.get_object_dataprocessing_mean_centroid_displacement());
        m_list_object_dataprocessing_stencil_points_displacement.push_back(
                simpleAverage.get_object_dataprocessing_stencil_point_displacement());
    }

    if ( Dataset::m_dataprocessing_map["MovingAverage"]) {
        MovingAverage movingAverage;
        movingAverage.common(this, post_processing_algorithm);
        m_list_object_dataprocessing_mean_centroid_displacement.push_back(
                movingAverage.get_object_dataprocessing_mean_centroid_displacement());
        m_list_object_dataprocessing_stencil_points_displacement.push_back(
                movingAverage.get_object_dataprocessing_stencil_point_displacement());
    }

    if ( Dataset::m_dataprocessing_map["VotedMean"]) {
        VotedMean votedMean;
        votedMean.common(this, post_processing_algorithm);
        m_list_object_dataprocessing_mean_centroid_displacement.push_back(
                votedMean.get_object_dataprocessing_mean_centroid_displacement());
        m_list_object_dataprocessing_stencil_points_displacement.push_back(
                votedMean.get_object_dataprocessing_stencil_point_displacement());
    }

    if ( post_processing_algorithm != "ground_truth" ) {
        //generate_edge_contour(post_processing_algorithm);
        //RankedMean rankedMean;
        //rankedMean.common(this);
        //m_list_object_dataprocessing_mean_centroid_displacement.push_back(rankedMean.get_object_dataprocessing_mean_centroid_displacement());
        //m_list_object_dataprocessing_stencil_points_displacement.push_back(rankedMean.get_object_dataprocessing_stencil_point_displacement());

    }

    generate_object_mean_lineparameters(post_processing_algorithm);

}

void Objects::generate_object_mean_lineparameters( std::string post_processing_algorithm) {


    /// BEWARE !! I am in Cartesian co-ordinate system here.


    unsigned COUNT;
    if ( post_processing_algorithm == "ground_truth") {
        COUNT = 1;
    }
    else {
        COUNT = m_list_object_dataprocessing_mean_centroid_displacement.size();
    }

    std::vector<std::vector<std::vector<cv::Point2f > > > list_object_line_parameters;


    for ( unsigned datafilter_index = 0; datafilter_index < COUNT; datafilter_index++ ) {

        std::vector<std::vector<cv::Point2f > > sensor_line_parameters;

        for (unsigned sensor_index = 0; sensor_index < Dataset::SENSOR_COUNT; sensor_index++) {

            std::cout << "generate_object_mean_lineparameters for sensor_index " << sensor_index << " for datafilter_index " << datafilter_index << " for object name " << m_objectName << " " << std::endl;

            std::vector<cv::Point2f > frame_line_parameters;

            const unsigned long FRAME_COUNT =
                    m_list_object_dataprocessing_mean_centroid_displacement.at(datafilter_index).at
                            (sensor_index).size();

            for (unsigned current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {
// gt_displacement

                if (m_object_extrapolated_visibility.at(sensor_index).at(current_frame_index) == true) {

                    if ( current_frame_index > 0 ) {
                        cv::Point2f next_pts = m_list_object_dataprocessing_mean_centroid_displacement.at(datafilter_index).at
                                (sensor_index).at(current_frame_index).mean_pts;
                        cv::Point2f mean_displacement_vector =
                                m_list_object_dataprocessing_mean_centroid_displacement.at(datafilter_index).at
                                        (sensor_index).at(current_frame_index).mean_displacement;

                        float m, c;
                        //Invert Y co-ordinates to match Cartesian co-ordinate system. This is just for the line angle and
                        //collision points. This is because, the camera origin are upper left and Cartesian is lower left
                        mean_displacement_vector.y = -mean_displacement_vector.y;
                        next_pts.y = -next_pts.y;
                        m = mean_displacement_vector.y / mean_displacement_vector.x;
                        c = next_pts.y - m * next_pts.x;  // c = y - mx
                        
                        assert(c!=0);

                        if ((int) c == 0) {
                            c += 0.001;
                        }

                        //float d = (float) sqrt((double) mean_displacement_vector.x * mean_displacement_vector.x +
                        //                       (double) mean_displacement_vector.y * mean_displacement_vector.y);
                        //mean_displacement_vector.x /= d; // normalized vector in x
                        //mean_displacement_vector.y /= d; // normalized vector in y

                        cv::Point2f pt2;

                        assert(std::isinf(m) == 0);

                        if (std::isinf(m)) {
                            if (mean_displacement_vector.y > 0.0f) {  // going up
                                pt2.x = next_pts.x;
                                pt2.y = Dataset::m_frame_size.height;
                            } else {  // going down
                                pt2.x = next_pts.x;
                                pt2.y = 0;
                            }
                        } else if (m == 0) {
                            //std::cout << current_frame_index << " " << next_pts<<  " " << m << " " << mean_displacement_vector << std::endl;
                            if (std::signbit(m)) { //  going left
                                pt2.x = 0;
                                pt2.y = next_pts.y;
                            } else {  // going right
                                pt2.x = Dataset::m_frame_size.width;
                                pt2.y = next_pts.y;
                            }
                        }

                        if (mean_displacement_vector.y > 0.0f) {
                            pt2.x = (Dataset::m_frame_size.height - c) / m; //
                            pt2.y = Dataset::m_frame_size.height;
                        } else if (mean_displacement_vector.y < 0.0f) {
                            pt2.x = (-c / m); //
                            pt2.y = 0;
                        }

                        frame_line_parameters.push_back(cv::Point2f(m, c));

                    } else {
                        frame_line_parameters.push_back(cv::Point2f(0.0f, 0.0f));
                    }

                }
                else {
                    frame_line_parameters.push_back(cv::Point2f(0.0f, 0.0f));
                }

            }

            //std::cout << frame_line_parameters << std::endl;

            sensor_line_parameters.push_back(frame_line_parameters);
        }
        list_object_line_parameters.push_back(sensor_line_parameters);
    }
    m_list_object_line_parameters = list_object_line_parameters;
    assert(m_list_object_line_parameters.at(0).at(0).size() == m_object_stencil_point_displacement.at(0).size());
    std::cout << "line done" << std::endl;
}
