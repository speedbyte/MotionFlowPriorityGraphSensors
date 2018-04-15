
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
#include "GroundTruthFlow.h"
#include "kbhit.h"

#include <unordered_map>
#include <bits/unordered_map.h>

#include "Dataset.h"
#include "GroundTruthScene.h"


//Creating a movement path. The path is stored in a x and y vector

using namespace std::chrono;


void GroundTruthFlow::prepare_directories() {

    mImageabholOrt = Dataset::getGroundTruthPath().string() + "/none/";

    m_resultordner="/generated";

    m_generatepath = Dataset::getGroundTruthPath().string() + m_resultordner;

    if (!Dataset::getDatasetPath().compare(CPP_DATASET_PATH) || !Dataset::getDatasetPath().compare(VIRES_DATASET_PATH)) {

        std::cout << "prepare gt_flow directories" << std::endl;

        OpticalFlow::prepare_directories();

    }
}

void GroundTruthFlow::generate_flow_frame() {

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the position in a png file

    prepare_directories();

    cv::Mat tempGroundTruthImage;
    tempGroundTruthImage.create(Dataset::getFrameSize(), CV_8UC3);
    assert(tempGroundTruthImage.channels() == 3);


    std::map<std::string, double> time_map = {{"generate_single_flow_image", 0},
                                              {"generate_all_flow_image", 0}};

    auto tic_all = steady_clock::now();

    std::cout << "ground truth flow will be stored in " << m_generatepath << std::endl;

    char frame_skip_folder_suffix[50];
    cv::FileStorage fs;
    fs.open(m_flow_occ_path.string() + frame_skip_folder_suffix + "/" + "gt_flow.yaml",
            cv::FileStorage::WRITE);


    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

        sprintf(frame_skip_folder_suffix, "%02d", frame_skip);
        std::cout << "saving ground truth flow files for frame_skip " << frame_skip << std::endl;

        unsigned FRAME_COUNT = (unsigned)m_list_gt_objects.at(0)->get_obj_extrapolated_shape_pixel_point_pixel_displacement().at(frame_skip-1).size();
        assert(FRAME_COUNT>0);

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
            
            char file_name_input_image[50], file_name_image_edge[50];

            auto tic = steady_clock::now();

            std::cout << "frame_count " << frame_count << std::endl;

            sprintf(file_name_input_image, "000%03d_10.png", frame_count*frame_skip);
            sprintf(file_name_image_edge, "000%03d_10_edge.png", frame_count*frame_skip);

            std::string temp_gt_flow_image_path = m_flow_occ_path.string() + frame_skip_folder_suffix + "/" +
                    file_name_input_image;

            std::string temp_result_edge_path = m_flow_occ_path.string() + frame_skip_folder_suffix + "/" +
                                                  file_name_image_edge;

            fs << "frame_count" << frame_count;

            FlowImageExtended F_png_write( Dataset::getFrameSize().width, Dataset::getFrameSize().height);

            cv::Mat tempMatrix;
            tempMatrix.create(Dataset::getFrameSize(), CV_32FC3);
            tempMatrix = cv::Scalar_<unsigned>(255,255,255);
            assert(tempMatrix.channels() == 3);

            for (unsigned obj_index = 0; obj_index < m_list_gt_objects.size(); obj_index++) {

                // object image_data_and_shape

                int width = cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_width_m);
                int height = cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_height_m);

                if ( m_list_gt_objects.at(obj_index)->get_obj_extrapolated_visibility().at(frame_skip - 1).at(frame_count) == true ) {

                    // gt_displacement
                    cv::Point2f next_pts = m_list_gt_objects.at(obj_index)->get_obj_extrapolated_pixel_position_pixel_displacement().at(frame_skip-1).at(frame_count).first;
                    cv::Point2f displacement = m_list_gt_objects.at(obj_index)->get_obj_extrapolated_pixel_position_pixel_displacement().at(frame_skip-1).at(frame_count).second;

                    cv::Mat roi;
                    roi = tempMatrix.
                            colRange(cvRound(next_pts.x), cvRound(next_pts.x + width)).
                            rowRange(cvRound(next_pts.y), cvRound(next_pts.y + height));
                    //bulk storage
                    //roi = cv::Scalar(displacement.x, displacement.y,
                    //                 static_cast<float>(m_list_gt_objects.at(obj_index)->getObjectId()));
                    roi = cv::Scalar(displacement.x, displacement.y,
                                     static_cast<float>(1.0f));

                }
            }

            //Create png Matrix with 3 channels: x displacement. y displacment and ObjectId
            // v corresponds to next row.
            for (int32_t row = 0; row < Dataset::getFrameSize().height; row++) { // rows
                for (int32_t col = 0; col < Dataset::getFrameSize().width; col++) {  // cols
                    if (tempMatrix.at<cv::Vec3f>(row, col)[2] > 0.5 ) {
                        /*inline void setFlowU (const int32_t u,const int32_t v,const float val) {
                            data_[3*(v*width_+u)+0] = val;
                        }*/
                        F_png_write.setFlowU(col, row, tempMatrix.at<cv::Vec3f>(row, col)[0]);
                        F_png_write.setFlowV(col, row, tempMatrix.at<cv::Vec3f>(row, col)[1]);
                        //F_png_write.setObjectId(col, row, tempMatrix.at<cv::Vec3f>(row, col)[2]);
                        F_png_write.setValid(col, row, true);
                        //position.store_in_yaml(fs, cv::Point2f(row, column), cv::Point2f(xValue, yValue) );
                    }
                }
            }

            F_png_write.writeExtended(temp_gt_flow_image_path);
            CannyEdgeDetection(temp_gt_flow_image_path, temp_result_edge_path);

            auto toc = steady_clock::now();
            time_map["generate_single_flow_image"] = duration_cast<milliseconds>(toc - tic).count();


        }
        fs.release();
    }

    std::cout << "end of saving ground truth flow files " << std::endl;

    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
    auto toc_all = steady_clock::now();
    time_map["generate_all_flow_image"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth flow generation time - " << time_map["generate_all_flow_image"] << "ms" << std::endl;

}

void GroundTruthFlow::generate_edge_contour() {

    for ( int frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++ ) {

        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > outer_edge_movement(m_list_gt_objects.size());

        char frame_skip_folder_suffix[50];
        char file_name_input_image[50];

        std::cout << "edge counter results will be stored in " << m_resultordner << std::endl;

        /*
        if ( frame_types == video_frames) {
            cv::VideoCapture cap;
            cap.open(Dataset::getGroundTruthPath().string() + "image_02/movement.avi");
            if (!cap.isOpened()) {
                std::cout << "Could not initialize capturing...\n";
                return;
            }
        }*/
        sprintf(frame_skip_folder_suffix, "%02d", frame_skip);
        std::string results_flow_matrix_str = m_flow_occ_path.string() + "/" +
                                              frame_skip_folder_suffix + "/" + "result_flow.yaml";

        const int MAX_COUNT = 5000;

        auto tic = steady_clock::now();
        auto toc = steady_clock::now();

        std::string temp_result_flow_path;

        std::cout << "creating edge files for frame_skip " << frame_skip << std::endl;
        std::vector<std::pair<cv::Point2f, cv::Point2f> > next_pts_array;

        for (ushort frame_count=0; frame_count < MAX_ITERATION_RESULTS; frame_count++) {
            //draw new ground truth flow.

            if ( frame_count*frame_skip >= MAX_ITERATION_RESULTS) {
                break;
            }

            sprintf(file_name_input_image, "000%03d_10_edge.png", frame_count*frame_skip);

            temp_result_flow_path = m_flow_occ_path.string() + frame_skip_folder_suffix + "/" + file_name_input_image;

            cv::Mat edge_02_frame = cv::imread(temp_result_flow_path, CV_LOAD_IMAGE_COLOR);
            if ( edge_02_frame.data == NULL ) {
                std::cerr << temp_result_flow_path << " not found" << std::endl;
                throw ("No image file found error");
            }

            cv::Mat edge_02_frame_gray;
            cv::cvtColor(edge_02_frame, edge_02_frame_gray, CV_RGB2GRAY);

            // Calculate optical generate_flow_frame map using LK algorithm
            std::cout << "frame_count " << frame_count << std::endl;

            if ( frame_count > 0 ) {
                for ( ushort obj_index = 0; obj_index < m_list_gt_objects.size(); obj_index++ ) {

                    bool visibility = m_list_gt_objects.at(obj_index)->get_obj_extrapolated_visibility().at(frame_skip-1).at(frame_count);
                    if ( visibility ) {

                        // This is for the base model
                        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> >  > edge_movement(m_list_gt_objects.size());

                        assert(m_list_gt_objects.size() == m_list_gt_objects.size());

                        next_pts_array = m_list_gt_objects.at(obj_index)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(frame_skip-1).at(frame_count);

                        //cv::namedWindow("edge", CV_WINDOW_AUTOSIZE);
                        //cv::imshow("edge", edge_02_frame_gray);
                        //cv::waitKey(0);

                        //std::cout << roi_offset.x + col_index << std::endl;
                        auto COUNT = next_pts_array.size();
                        std::cout << "making a edge contour on the basis of groundtruth object " << m_list_gt_objects.at(obj_index)->getObjectId() << std::endl;
                        std::cout << "base count " << COUNT << std::endl;
                        for ( ushort next_pts_index = 0; next_pts_index < COUNT; next_pts_index++ ) {

                            //printf("gray %u\n", edge_02_frame_gray.at<char>(cvRound(next_pts_array.at(next_pts_index).first.y), cvRound(next_pts_array.at(next_pts_index).first.x)));
                            if ( edge_02_frame_gray.at<char>(cvRound(next_pts_array.at(next_pts_index).first.y), cvRound(next_pts_array.at(next_pts_index).first.x)) != 0 ) {
                                edge_movement.at(obj_index).push_back(
                                        std::make_pair(next_pts_array.at(next_pts_index).first,
                                                       next_pts_array.at(next_pts_index).second));
                            }
                        }

                        auto new_edge_size = edge_movement.at(obj_index).size();
                        std::cout << new_edge_size << std::endl;
                        //assert(new_edge_size != 0);

                        outer_edge_movement.at(obj_index).push_back(edge_movement.at(obj_index));

                    }
                    else {

                        for ( ushort obj_index = 0; obj_index < m_list_gt_objects.size(); obj_index++ ) {
                            outer_edge_movement.at(obj_index).push_back(
                                    {{std::make_pair(cv::Point2f(0, 0), cv::Point2f(0, 0))}});
                        }
                    }
                }
            }
            else {
                for ( ushort obj_index = 0; obj_index < m_list_gt_objects.size(); obj_index++ ) {
                    outer_edge_movement.at(obj_index).push_back(
                            {{std::make_pair(cv::Point2f(0, 0), cv::Point2f(0, 0))}});
                }
            }
        }

        for ( ushort obj_index = 0; obj_index < m_list_gt_objects.size(); obj_index++) {
            m_list_gt_objects.at(obj_index)->generate_obj_extrapolated_edge_pixel_point_pixel_displacement(outer_edge_movement.at(obj_index));
        }
    }
}


void GroundTruthFlow::generate_shape_points() {

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the position in a png file

    std::vector<std::pair<Objects*, Objects*> > list_of_gt_objects_combination;

    getCombination(m_list_gt_objects, list_of_gt_objects_combination);

    std::map<std::string, double> time_map = {{"generate",     0},
                                              {"generate_flow", 0}};

    auto tic = steady_clock::now();
    auto toc = steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();

    char frame_skip_folder_suffix[50];
    cv::FileStorage fs;

    for ( ushort i = 0; i < list_of_gt_objects_combination.size(); i ++ ) {
        std::cout << "shape between object id " << list_of_gt_objects_combination.at(i).first->getObjectId() <<
                  " and object id "
                  << list_of_gt_objects_combination.at(i).second->getObjectId()<< "\n";
    }

    std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > > list_frame_skip_shape_points;

    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

        std::vector<std::vector<std::vector<cv::Point2f> > > outer_frame_skip_shape_points;

        sprintf(frame_skip_folder_suffix, "%02d", frame_skip);

        std::cout << "generating shape points in OpticalFlow.cpp for " << m_resultordner << " " << frame_skip
                  << std::endl;
        for (unsigned data_processing_index = 0; data_processing_index < 1; data_processing_index++) {

            std::vector<std::vector<cv::Point2f> > frame_shape_points;

            unsigned FRAME_COUNT = (unsigned) m_list_gt_objects.at(0)
                    ->get_shape_parameters().at(frame_skip - 1).at(data_processing_index).size();

            assert(FRAME_COUNT > 0);

            for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {


                std::cout << "frame_count " << frame_count << std::endl;

                fs << "frame_count" << frame_count;

                cv::Point2f shape_average = {0, 0};
                std::vector<cv::Point2f> shape_points(m_list_gt_objects.size());
                std::vector<cv::Point2f> shape_points_average;


                for (unsigned i = 0; i < m_list_gt_objects.size(); i++) {


                    auto CLUSTER_COUNT_GT = m_list_gt_objects.at(
                            i)->get_obj_extrapolated_shape_pixel_point_pixel_displacement().at
                            (frame_skip - 1).at(frame_count).size();

                    if (m_list_gt_objects.at(i)->get_obj_extrapolated_mean_visibility().at(frame_skip - 1).at(frame_count) == true) {

                        float vollTreffer = 0;

                        float baseTreffer;

                        if (m_resultordner ==
                            "/generated") {  // this is unncecessary, because this function is in GroundTruthFlow, still I will leave this.
                            vollTreffer = CLUSTER_COUNT_GT;
                            baseTreffer = CLUSTER_COUNT_GT;
                        }
                        shape_points.at(i) = (cv::Point2f(vollTreffer, baseTreffer));

                        std::cout << "vollTreffer for object " << m_list_gt_objects.at(i)->getObjectId() << " = "
                                  << vollTreffer << std::endl;
                        std::cout << "baseTreffer for object " << m_list_gt_objects.at(i)->getObjectId() << " = "
                                  << baseTreffer << std::endl;

                        shape_average.x += shape_points.at(i).x;
                        shape_average.y += shape_points.at(i).y;

                    } else {
                        std::cout << "visibility of object " << m_list_gt_objects.at(i)->getObjectId() << " = " <<
                                  m_list_gt_objects.at(i)->get_obj_extrapolated_mean_visibility().at(frame_skip
                                                                                                     - 1)
                                          .at(frame_count)
                                  << " and hence not generating any shape points for this object " << std::endl;

                        shape_points.at(i) = (cv::Point2f(0, CLUSTER_COUNT_GT));
                        shape_average.x += shape_points.at(i).x;
                        shape_average.y += shape_points.at(i).y;

                    }
                }

                shape_average.x = shape_average.x / m_list_gt_objects.size();
                shape_average.y = shape_average.y / m_list_gt_objects.size();

                shape_points_average.push_back(shape_average);

                frame_shape_points.push_back(shape_points_average);
            }
            outer_frame_skip_shape_points.push_back(frame_shape_points);
        }
        list_frame_skip_shape_points.push_back(outer_frame_skip_shape_points);
    }
    m_frame_skip_shape_points = list_frame_skip_shape_points;
    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
    toc_all = steady_clock::now();
    time_map["generate_flow"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << m_resultordner + " flow generation time - " << time_map["generate_flow"] << "ms" << std::endl;
}


void GroundTruthFlow::visualiseStencil(void) {

    std::cout << "visualise stencil at " << m_generatepath.string() + "/stencil/" << std::endl;

    char file_name_image_output[50], file_name_input_image[50];

    cv::Mat image_data_and_shape;


    const ushort max_frame_skip = 1; // image is generated only once irrespective of skips.
    cv::Mat tempGroundTruthImage(Dataset::getFrameSize(), CV_8UC3);
    FlowImageExtended F_png_write;

    for ( int frame_skip = 1; frame_skip <= max_frame_skip; frame_skip++ ) {

        for (ushort frame_count = 0; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_IMAGES; frame_count++)
        {

            std::cout << "frame_count " << frame_count << std::endl;
            std::string output_image_file_with_path;

            /*---------------------------------------------------------------------------------*/
            tempGroundTruthImage = cv::Scalar::all(0);
            sprintf(file_name_input_image, "000%03d_10.png", frame_count*frame_skip);

            std::string input_image_file_with_path = mImageabholOrt.string() + "/" +
                                                     file_name_input_image;

            tempGroundTruthImage = cv::imread(input_image_file_with_path, CV_LOAD_IMAGE_COLOR);

            sprintf(file_name_image_output, "000%03d_10_bb.png", frame_count * frame_skip);
            output_image_file_with_path =
                    m_generatepath.string() + "/stencil/" + file_name_image_output;

            auto tic = steady_clock::now();
            auto toc = tic;
            float time_elapsed;

            for (unsigned obj_index = 0; obj_index < m_list_gt_objects.size(); obj_index++) {

                if ((m_list_gt_objects.at(obj_index)->get_obj_base_visibility().at(frame_count))) {

                    float columnBegin = m_list_gt_objects.at(obj_index)->get_obj_extrapolated_pixel_position_pixel_displacement().at
                            (frame_skip-1).at(frame_count).first.x;
                    float rowBegin = m_list_gt_objects.at(obj_index)->get_obj_extrapolated_pixel_position_pixel_displacement().at
                            (frame_skip-1).at(frame_count).first.y;

                    int width = cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_width_m);
                    int height = cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_height_m);

                    float offset_x = m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_offset_m.offset_x;
                    float offset_y = m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_offset_m.offset_y;


                    cv::Rect boundingbox = cv::Rect(
                            cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    frame_skip - 1).at(frame_count).m_object_location_px.location_x_m),
                            cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    frame_skip - 1).at(frame_count).m_object_location_px.location_y_m),
                            cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    frame_skip - 1).at(frame_count).m_object_dimensions_px.dim_width_m),
                            cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    frame_skip - 1).at(frame_count).m_object_dimensions_px.dim_height_m));

                    cv::Rect boundingbox2 = cv::Rect(columnBegin-offset_x*width, rowBegin+offset_y*height, width, height+offset_y*height );
                    //cv::Rect boundingbox3 = cv::Rect(columnBegin, rowBegin+height, width, height);

                    cv::rectangle(tempGroundTruthImage, boundingbox, cv::Scalar(0, 255, 0), 1, 8, 0);
                    cv::rectangle(tempGroundTruthImage, boundingbox2, cv::Scalar(0, 0, 255), 1, 8, 0);
                    //cv::rectangle(tempGroundTruthImage, boundingbox3, cv::Scalar(255, 0, 0), 1, 8, 0);

                    cv::Point2f pts_mean = m_list_gt_objects.at(obj_index)->get_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement().at(frame_skip-1).
                            at(0).at(frame_count).first;
                    cv::Point2f pts_basic = m_list_gt_objects.at(obj_index)->get_obj_extrapolated_pixel_position_pixel_displacement().at(
                            frame_skip - 1).at(frame_count).first;

                    cv::Point2f displacement = m_list_gt_objects.at(obj_index)->get_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement().at(frame_skip - 1).at(
                            0).at(frame_count).second;

                    cv::Point2f next_pts = cv::Point2f(pts_basic.x + displacement.x*10, pts_basic.y + displacement.y*10);

                    cv::arrowedLine(tempGroundTruthImage, pts_basic, next_pts, cv::Scalar(0,0,255), 2);


                }
            }
            cv::imwrite(output_image_file_with_path, tempGroundTruthImage);

            /*---------------------------------------------------------------------------------*/
            tempGroundTruthImage = cv::Scalar::all(255);

            sprintf(file_name_image_output, "000%03d_10_pixel_gt.png", frame_count * frame_skip);
            output_image_file_with_path =
                    m_generatepath.string() + "/stencil/" + file_name_image_output;

            for (unsigned obj_index = 0; obj_index < m_list_gt_objects.size(); obj_index++) {

                if ((m_list_gt_objects.at(obj_index)->get_obj_base_visibility().at(frame_count))
                        ) {

                    const unsigned CLUSTER_SIZE = (unsigned) m_list_gt_objects.at(
                            obj_index)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                            (frame_skip - 1).at(frame_count).size();
                    for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {

                        cv::Point2f pts = m_list_gt_objects.at(
                                        obj_index)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_point).first;


                        cv::circle(tempGroundTruthImage, pts, 1.5, cv::Scalar(0, 255, 0), 1, 8);

                    }
                }
            }
            //cv::imwrite(output_image_file_with_path, tempGroundTruthImage);

            /*---------------------------------------------------------------------------------*/
            tempGroundTruthImage = cv::Scalar::all(255);
            F_png_write = FlowImageExtended(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

            sprintf(file_name_image_output, "000%03d_10_flow_gt.png", frame_count * frame_skip);
            output_image_file_with_path =
                    m_generatepath.string() + "/stencil/" + file_name_image_output;

            for (unsigned obj_index = 0; obj_index < m_list_gt_objects.size(); obj_index++) {

                if ((m_list_gt_objects.at(obj_index)->get_obj_extrapolated_visibility().at(frame_skip - 1).at(
                        frame_count)
                )) {

                    unsigned CLUSTER_SIZE = (unsigned) m_list_gt_objects.at(
                            obj_index)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                            (frame_skip - 1).at(frame_count).size();
                    for (unsigned cluster_point = 0; cluster_point < CLUSTER_SIZE; cluster_point++) {


                        cv::Point2f pts = m_list_gt_objects.at(
                                        obj_index)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_point).first;

                        cv::Point2f displacement = m_list_gt_objects.at(
                                        obj_index)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_point).second;


                        cv::Point2f next_pts = cv::Point2f(pts.x + displacement.x, pts.y + displacement.y);

                        F_png_write.setFlowU(pts.x, pts.y, displacement.x);
                        F_png_write.setFlowV(pts.x, pts.y, displacement.y);
                        F_png_write.setValid(pts.x, pts.y, true);

                        cv::arrowedLine(tempGroundTruthImage, pts, next_pts, cv::Scalar(0, 255, 0), 1, 8, 0, 0.25);

                    }
                }
            }

            //F_png_write.writeExtended(output_image_file_with_path);
            //cv::imwrite(output_image_file_with_path, tempGroundTruthImage);

            //cv::namedWindow("bb", CV_WINDOW_AUTOSIZE);
            //cv::imshow("bb", tempGroundTruthImageBase);
            //cv::waitKey(0);

        }
    }
}


/*
        //cv::Vec3f *datagt_next_ptsr = tempMatrix.gt_next_ptsr<cv::Vec3f>(0); // pointer to the first channel of the first element in the
        // first row. The r, g b  value of single pixels are continous.
        float *array = (float *)malloc(3*sizeof(float)*Dataset::getFrameSize().width*Dataset::getFrameSize().height);
        cv::MatConstIterator_<cv::Vec3f> it = roi.begin<cv::Vec3f>();
        for (unsigned i = 0; it != roi.end<cv::Vec3f>(); it++ ) {
            for ( unsigned j = 0; j < 3; j++ ) {
                *(array + i ) = (*it)[j];
                i++;
            }
        }
        FlowImageExtended temp = FlowImageExtended(array, Dataset::getFrameSize().width, Dataset::getFrameSize().height );
        F_png_write = temp;

 */