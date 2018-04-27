//
// Created by veikas on 06.02.18.
//

#include <map>
#include <chrono>
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>
#include "OpticalFlow.h"
#include "FlowImageExtended.h"

using namespace std::chrono;

void OpticalFlow::CannyEdgeDetection(std::string temp_result_flow_path, std::string temp_result_edge_path) {

    cv::Mat src, src_gray;
    cv::Mat dst, detected_edges, blurred_image;

    int edgeThresh = 1;
    int lowThreshold=15;
    int const max_lowThreshold = 100;
    int ratio = 10;
    int kernel_size = 3;
    std::string window_name = "Edge Map";
    std::string path;
    src = cv::imread(temp_result_flow_path);

    if( !src.data ) {
        std::cout << "no image found";
        exit(-1);
    }

    /// Create a matrix of the same type and size as src (for dst)
    dst.create( src.size(), src.type() );

    /// Convert the image to grayscale
    cv::cvtColor( src, src_gray, CV_BGR2GRAY );

    /// Create a window
    //cv::namedWindow( window_name, CV_WINDOW_AUTOSIZE );

    /// Create a Trackbar for user to enter threshold
    //cv::createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

    /// Reduce noise with a kernel 3x3
    cv::blur( src_gray, blurred_image, cv::Size(3,3) );

    /// Canny detector
    cv::Canny( blurred_image, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

    /// Using Canny's output as a mask, we display our result
    dst = cv::Scalar::all(0);

    src.copyTo( dst, detected_edges);
    cv::imwrite( temp_result_edge_path, dst );

    //cv::imshow( window_name, dst);

    /// Wait until user exit program by pressing a key
    //cv::waitKey(0);
}


void OpticalFlow::prepare_directories() {

    char char_dir_append[20];

    if (boost::filesystem::exists(m_generatepath)) {
        system(("rm -rf " + m_generatepath.string()).c_str());
    }
    boost::filesystem::create_directories(m_generatepath);

    boost::filesystem::path path;

    for (int i = 1; i < MAX_SKIPS; ++i) {

        sprintf(char_dir_append, "%02d", i);

        m_collision_obj_path = m_generatepath.string() + "/collision_obj_";
        path =  m_collision_obj_path.string() + char_dir_append;
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

        // post processing step
        boost::filesystem::path bbox_dir = m_generatepath.string() + "/stencil/";
        if (boost::filesystem::exists(m_generatepath)) {
            system(("rm -rf " + bbox_dir.string()).c_str());
        }
        boost::filesystem::create_directories(bbox_dir);
    }
}

void OpticalFlow::getCombination( const std::vector<Objects *> &m_list_objects, std::vector<std::pair<Objects*, Objects* > > &list_of_objects_combination) {
    std::vector<Objects*>::const_iterator objectIterator = m_list_objects.begin();
    std::vector<Objects*>::const_iterator  objectIteratorNext;

    for ( ; objectIterator < m_list_objects.end() ; objectIterator++ ) {
        for ( objectIteratorNext = objectIterator+1; objectIteratorNext < m_list_objects.end();
                objectIteratorNext++) {

            list_of_objects_combination.push_back(std::make_pair(((*objectIterator)),
                    ((*objectIteratorNext))));
        }
    }
}

void OpticalFlow::generate_edge_contour() {


    std::vector<Objects*> list_of_current_objects;

    if ( m_resultordner == "/ground_truth") {
        list_of_current_objects = m_list_gt_objects;
    }
    else {
        list_of_current_objects = m_list_simulated_objects;
    }

    for ( int frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++ ) {

        std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > outer_edge_movement(list_of_current_objects.size());

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

        std::string temp_result_flow_path;

        std::cout << "creating edge files for frame_skip " << frame_skip << std::endl;
        std::vector<std::pair<cv::Point2f, cv::Point2f> > next_pts_array;

        for (ushort frame_count=0; frame_count < MAX_ITERATION_RESULTS; frame_count++) {
            //draw new ground truth flow.

            if ( frame_count*frame_skip >= MAX_ITERATION_RESULTS) {
                break;
            }

            cv::Mat objectEdgeFrame( Dataset::getFrameSize(), CV_8UC1 );
            objectEdgeFrame = cv::Scalar_<char>(0);


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
                for ( ushort obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++ ) {

                    objectEdgeFrame = cv::Scalar_<char>(0);
                    bool visibility = list_of_current_objects.at(obj_index)->get_obj_extrapolated_visibility().at(frame_skip-1).at(frame_count);
                    if ( visibility ) {

                        // This is for the base model
                        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> >  > edge_movement(list_of_current_objects.size());

                        assert(list_of_current_objects.size() == list_of_current_objects.size());

                        next_pts_array = list_of_current_objects.at(obj_index)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(frame_skip-1).at(frame_count);

                        //std::cout << roi_offset.x + col_index << std::endl;
                        auto COUNT = next_pts_array.size();
                        std::cout << "making a edge contour on the basis of " << m_resultordner << " " << list_of_current_objects.at(obj_index)->getObjectId() << std::endl;
                        std::cout << "base count " << COUNT << std::endl;
                        for ( ushort next_pts_index = 0; next_pts_index < COUNT; next_pts_index++ ) {

                            //printf("gray %u\n", edge_02_frame_gray.at<char>(cvRound(next_pts_array.at(next_pts_index).first.y), cvRound(next_pts_array.at(next_pts_index).first.x)));
                            if ( edge_02_frame_gray.at<char>(cvRound(next_pts_array.at(next_pts_index).first.y), cvRound(next_pts_array.at(next_pts_index).first.x)) != 0 ) {

                                objectEdgeFrame.at<char>(cvRound(next_pts_array.at(next_pts_index).first.y), cvRound(next_pts_array.at(next_pts_index).first.x)) = 255;
                                edge_movement.at(obj_index).push_back(
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
                        //cv::waitKey(2000);


                        auto new_edge_size = edge_movement.at(obj_index).size();
                        std::cout << new_edge_size << std::endl;
                        assert(new_edge_size != 0);

                        outer_edge_movement.at(obj_index).push_back(edge_movement.at(obj_index));

                    }
                    else {

                        for ( ushort obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++ ) {
                            outer_edge_movement.at(obj_index).push_back(
                                    {{std::make_pair(cv::Point2f(0, 0), cv::Point2f(0, 0))}});
                        }
                    }
                }
            }
            else {
                for ( ushort obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++ ) {
                    outer_edge_movement.at(obj_index).push_back(
                            {{std::make_pair(cv::Point2f(0, 0), cv::Point2f(0, 0))}});
                }
            }
        }

        for ( ushort obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++) {
            list_of_current_objects.at(obj_index)->generate_obj_extrapolated_edge_pixel_point_pixel_displacement(outer_edge_movement.at(obj_index));
        }
    }
}


void OpticalFlow::generate_shape_points() {

    std::vector<Objects*> list_of_current_objects;

    unsigned COUNT;
    if ( m_resultordner == "/ground_truth") {
        COUNT = 1;
        list_of_current_objects = m_list_gt_objects;
    }
    else {
        COUNT = DATA_PROCESSING_COUNT;
        list_of_current_objects = m_list_simulated_objects;
    }

    char frame_skip_folder_suffix[50];

    std::vector<std::map<std::pair<float, float>, int> > outer_frame_skip_scenario_displacement_occurence;

    for (unsigned data_processing_index = 0; data_processing_index < COUNT; data_processing_index++) {

        std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > > outer_frame_skip_shape_points;

        for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

            std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > outer_frame_shape_points;
            std::map<std::pair<float, float>, int> scenario_displacement_occurence;

            sprintf(frame_skip_folder_suffix, "%02d", frame_skip);

            std::cout << "generating shape points in OpticalFlow.cpp for " << m_resultordner << " " << frame_skip
                    << " for dataprocessing " << data_processing_index << std::endl;

            unsigned FRAME_COUNT = (unsigned) list_of_current_objects.at(0)
                    ->get_list_obj_shape_parameters().at(data_processing_index).at(frame_skip - 1).size();

            assert(FRAME_COUNT > 0);

            for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

                std::cout << "frame_count " << frame_count << " for data_processing_index " << data_processing_index<< std::endl;


                cv::Point2f shape_average = {0, 0};
                std::vector<std::pair<cv::Point2i, cv::Point2f>> frame_shape_points;

                for (ushort obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++) {

                    auto CLUSTER_COUNT_GT = m_list_gt_objects.at(
                            obj_index)->get_list_obj_shape_parameters().at(0).at(frame_skip - 1).at(frame_count).size();

                    auto CLUSTER_COUNT_ALGO = list_of_current_objects.at(
                            obj_index)->get_list_obj_shape_parameters().at(data_processing_index).at(frame_skip - 1).at(frame_count).size();

                    if (list_of_current_objects.at(obj_index)->get_obj_extrapolated_mean_visibility().at(frame_skip - 1).at(frame_count) ) {

                        // Instances of CLUSTER_COUNT_ALGO in CLUSTER_COUNT_GT

                        float vollTreffer = 0;

                        float baseTreffer;

                        cv::Point2f gt_displacement = m_list_gt_objects.at(obj_index)->get_obj_extrapolated_pixel_position_pixel_displacement().at
                                (frame_skip-1).at(frame_count).second;

                        auto dist_gt = cv::norm(gt_displacement);
                        auto angle_gt = std::tanh(gt_displacement.y / gt_displacement.x);



                        if (m_resultordner == "/ground_truth") {

                            vollTreffer = CLUSTER_COUNT_GT;
                            baseTreffer = CLUSTER_COUNT_GT;

                            if ( data_processing_index < 0 ) {

                                if ( scenario_displacement_occurence.count(std::make_pair(std::round(gt_displacement.x*DISPLACEMENT_ROUND_OFF)/DISPLACEMENT_ROUND_OFF,std::round(gt_displacement.y*DISPLACEMENT_ROUND_OFF)/DISPLACEMENT_ROUND_OFF)) ) {
                                    scenario_displacement_occurence[std::make_pair(std::round(gt_displacement.x*DISPLACEMENT_ROUND_OFF)/DISPLACEMENT_ROUND_OFF,std::round(gt_displacement.y*DISPLACEMENT_ROUND_OFF)/DISPLACEMENT_ROUND_OFF)] = scenario_displacement_occurence[std::make_pair(std::round(gt_displacement.x*DISPLACEMENT_ROUND_OFF)/DISPLACEMENT_ROUND_OFF,std::round(gt_displacement.y*DISPLACEMENT_ROUND_OFF)/DISPLACEMENT_ROUND_OFF)] + CLUSTER_COUNT_GT;
                                }
                                else {
                                    scenario_displacement_occurence[std::make_pair(std::round(gt_displacement.x*DISPLACEMENT_ROUND_OFF)/DISPLACEMENT_ROUND_OFF,std::round(gt_displacement.y*DISPLACEMENT_ROUND_OFF)/DISPLACEMENT_ROUND_OFF)] = CLUSTER_COUNT_GT;
                                }

                                /*
                                if ( scenario_displacement_occurence.count(std::make_pair(65535,65535)) ) {
                                    scenario_displacement_occurence[std::make_pair(65535,65535)] = scenario_displacement_occurence[std::make_pair(65535,65535)] + 1;
                                }
                                else {
                                    scenario_displacement_occurence[std::make_pair(65535,65535)] = 1;
                                }*/
                            }
                        }
                        else {
                            for (auto cluster_count = 0; cluster_count < CLUSTER_COUNT_ALGO; cluster_count++) {
                                
                                cv::Point2f algo_displacement = list_of_current_objects.at(obj_index)->
                                                get_list_obj_shape_parameters().at(data_processing_index
                                                ).at(frame_skip - 1).at(frame_count).at(cluster_count).second;

                                if ( data_processing_index == 0 ) {
                                    if ( scenario_displacement_occurence.count(std::make_pair(std::round(algo_displacement.x*10)/10,std::round(algo_displacement.y*10)/10)) ) {
                                        scenario_displacement_occurence[std::make_pair(std::round(algo_displacement.x*10)/10,std::round(algo_displacement.y*10)/10)] = scenario_displacement_occurence[std::make_pair(std::round(algo_displacement.x*10)/10,std::round(algo_displacement.y*10)/10)] + 1;
                                    }
                                    else {
                                        scenario_displacement_occurence[std::make_pair(std::round(algo_displacement.x*10)/10,std::round(algo_displacement.y*10)/10)] = 1;
                                    }

                                    /*
                                    if ( scenario_displacement_occurence.count(std::make_pair(65535,65535)) ) {
                                        scenario_displacement_occurence[std::make_pair(65535,65535)] = scenario_displacement_occurence[std::make_pair(65535,65535)] + 1;
                                    }
                                    else {
                                        scenario_displacement_occurence[std::make_pair(65535,65535)] = 1;
                                    }*/

                                }

                                auto dist_algo = cv::norm(algo_displacement);
                                auto dist_err = std::abs(dist_gt - dist_algo);

                                auto angle_algo = std::tanh(algo_displacement.y/algo_displacement.x);

                                auto angle_err = angle_algo - angle_gt;
                                auto angle_err_dot = std::cosh(
                                        algo_displacement.dot(gt_displacement) / (dist_gt * dist_algo));

                                //assert(angle_err_dot==angle_err);


                                /*if (dist_err < DISTANCE_ERROR_TOLERANCE) {
                                    vollTreffer++;
                                }*/

                                if (((std::abs(angle_err*180/CV_PI)) < ANGLE_ERROR_TOLERANCE)) {
                                    vollTreffer++;
                                }

                            }
                            /*
                            auto x_coordinates = list_of_current_objects.at(
                                    obj_index)->get_list_obj_shape_parameters().at
                                    (frame_skip - 1).at(data_processing_index).at(frame_count).at(j).first.x;
                            auto y_coordinates = list_of_current_objects.at(
                                    obj_index)->get_list_obj_shape_parameters().at
                                    (frame_skip - 1).at(data_processing_index).at(frame_count).at(j).first.y;

                            if ((x_coordinates > (columnBegin - width / STENCIL_GRID_EXTENDER)) &&
                                (x_coordinates < (columnBegin + width + width / STENCIL_GRID_EXTENDER)) &&
                                (y_coordinates > (rowBegin - height / STENCIL_GRID_EXTENDER)) &&
                                (y_coordinates < (rowBegin + height + height / STENCIL_GRID_EXTENDER))
                                    ) {
                                vollTreffer++;
                            }
                        }*/

                            baseTreffer = ((float) CLUSTER_COUNT_GT);
                        }
                        frame_shape_points.push_back(std::make_pair(cv::Point2i(frame_count, 0), cv::Point2f(vollTreffer, baseTreffer)));

                        std::cout << "vollTreffer for object " << list_of_current_objects.at(obj_index)->getObjectId() << " = "
                                << vollTreffer << std::endl;
                        std::cout << "baseTreffer for object " << list_of_current_objects.at(obj_index)->getObjectId() << " = "
                                << baseTreffer << std::endl;

                    } else {
                        std::cout << "visibility of object " << list_of_current_objects.at(obj_index)->getObjectId() << " = " <<
                                list_of_current_objects.at(obj_index)->get_obj_extrapolated_mean_visibility().at(frame_skip
                                                - 1)
                                        .at(frame_count)
                                << " and hence not generating any shape points for this object " << std::endl;

                        frame_shape_points.push_back(std::make_pair(cv::Point2i(frame_count,0), cv::Point2f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));

                    }
                }

                outer_frame_shape_points.push_back(frame_shape_points);
            }
            outer_frame_skip_shape_points.push_back(outer_frame_shape_points);


            outer_frame_skip_scenario_displacement_occurence.push_back(scenario_displacement_occurence);

        }
        m_frame_skip_shape_points.push_back(outer_frame_skip_shape_points);
    }

    m_frame_skip_scenario_displacement_occurence = outer_frame_skip_scenario_displacement_occurence;

    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
}

void OpticalFlow::generate_mean_displacement_points() {

    std::vector<Objects*> list_of_current_objects;

    unsigned COUNT;
    if ( m_resultordner == "/ground_truth") {
        COUNT = 1;
        list_of_current_objects = m_list_gt_objects;
    }
    else {
        COUNT = DATA_PROCESSING_COUNT;
        list_of_current_objects = m_list_simulated_objects;
    }

    char frame_skip_folder_suffix[50];


    for (unsigned data_processing_index = 0; data_processing_index < COUNT; data_processing_index++) {

        std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > > outer_frame_skip_mean_displacement_points;

        for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

            std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > outer_frame_mean_displacement_points;

            sprintf(frame_skip_folder_suffix, "%02d", frame_skip);

            std::cout << "generating displacement points in OpticalFlow.cpp for " << m_resultordner << " " << frame_skip
                      << " for dataprocessing " << data_processing_index << std::endl;

            unsigned FRAME_COUNT = (unsigned) list_of_current_objects.at(0)
                    ->get_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement().at(data_processing_index).at(frame_skip - 1).size();

            assert(FRAME_COUNT > 0);

            for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

                std::cout << "frame_count " << frame_count << " for data_processing_index " << data_processing_index<< std::endl;

                std::vector<std::pair<cv::Point2i, cv::Point2f> > frame_mean_displacement_points;

                for (ushort obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++) {

                    cv::Point2i dimension = {
                            cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_width_m),
                            cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_height_m)
                    };

                    if (list_of_current_objects.at(obj_index)->get_obj_extrapolated_mean_visibility().at(frame_skip - 1).at(frame_count) && frame_count != 0 ) {

                        cv::Point2f displacement = list_of_current_objects.at(obj_index)->
                                get_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement().at(data_processing_index
                        ).at(frame_skip - 1).at(frame_count).second;


                        frame_mean_displacement_points.push_back(std::make_pair(dimension, displacement));

                        std::cout << "mean displacement for object with dimension" << dimension << " = "
                                  << displacement << std::endl;

                    } else {
                        std::cout << "visibility of object " << list_of_current_objects.at(obj_index)->getObjectId() << " = " <<
                                  list_of_current_objects.at(obj_index)->get_obj_extrapolated_mean_visibility().at(frame_skip
                                                                                                                   - 1)
                                          .at(frame_count)
                                  << " and hence not generating any displacement points for this object " << std::endl;

                        frame_mean_displacement_points.push_back(std::make_pair(dimension, cv::Point2f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));

                    }
                }

                outer_frame_mean_displacement_points.push_back(frame_mean_displacement_points);
            }

            outer_frame_skip_mean_displacement_points.push_back(outer_frame_mean_displacement_points);

        }
        m_frame_skip_mean_displacement_points.push_back(outer_frame_skip_mean_displacement_points);
    }
    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
}



void OpticalFlow::generate_collision_points() {

    std::vector<std::pair<Objects*, Objects*> > list_of_gt_objects_combination;
    std::vector<std::pair<Objects*, Objects* > > list_of_simulated_objects_combination;

    getCombination(m_list_gt_objects, list_of_gt_objects_combination);
    getCombination(m_list_simulated_objects, list_of_simulated_objects_combination);


    char frame_skip_folder_suffix[50];

    std::vector<Objects*> list_of_current_objects;
    std::vector<std::pair<Objects*, Objects* > > list_of_current_objects_combination;


    unsigned COUNT;
    if ( m_resultordner == "/ground_truth") {
        COUNT = 1;
        list_of_current_objects = m_list_gt_objects;
        list_of_current_objects_combination = list_of_gt_objects_combination;
    }
    else {
        COUNT = DATA_PROCESSING_COUNT;
        list_of_current_objects = m_list_simulated_objects;
        list_of_current_objects_combination = list_of_simulated_objects_combination;
    }

    for ( ushort obj_index = 0; obj_index < list_of_current_objects_combination.size(); obj_index++ ) {
        std::cout << "collision between object name " << list_of_current_objects_combination.at(obj_index).first->getObjectName() <<
                " and object name "
                << list_of_current_objects_combination.at(obj_index).second->getObjectName()<< "\n";
    }


    for ( unsigned data_processing_index = 0; data_processing_index < COUNT; data_processing_index++ ) {

        std::vector<std::vector<std::vector<cv::Point2f> > > outer_frame_skip_collision_points;
        std::vector<std::vector<std::vector<cv::Point2f> > > outer_frame_skip_line_angles;

        for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

            std::vector<std::vector<cv::Point2f> > outer_frame_collision_points;
            std::vector<std::vector<cv::Point2f> > outer_frame_line_angles;

            sprintf(frame_skip_folder_suffix, "%02d", frame_skip);

            std::cout << "generating collision points in OpticalFlow.cpp for " << m_resultordner << " " << frame_skip
                    << " for dataprocessing " << data_processing_index << std::endl;

            unsigned FRAME_COUNT = (unsigned) list_of_current_objects.at(0)
                    ->get_line_parameters().at(0).at(frame_skip - 1).size();
            
            assert(FRAME_COUNT > 0);

            for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

                std::cout << "frame_count " << frame_count << " for data_processing_index " << data_processing_index<< std::endl;


                std::vector<cv::Point2f> frame_collision_points;
                std::vector<cv::Point2f> frame_collision_points_average;
                std::vector<cv::Point2f> frame_line_angles;

                char file_name_image[50];

                sprintf(file_name_image, "000%03d_10.png", frame_count * frame_skip);
                std::string temp_collision_image_path =
                        m_collision_obj_path.string() + frame_skip_folder_suffix + "/" + file_name_image;


                FlowImageExtended F_png_write(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

                cv::Mat tempMatrix;
                tempMatrix.create(Dataset::getFrameSize(), CV_32FC3);
                tempMatrix = cv::Scalar_<unsigned>(255, 255, 255);
                assert(tempMatrix.channels() == 3);

                for (unsigned obj_index = 0;
                        obj_index < list_of_current_objects_combination.size(); obj_index++) {

                    if ((list_of_current_objects_combination.at(
                                    obj_index).first->get_obj_extrapolated_mean_visibility().at(
                                    frame_skip
                                            - 1)
                            .at(frame_count)) && (list_of_current_objects_combination.at(obj_index).second->
                                    get_obj_extrapolated_mean_visibility()
                            .at(frame_skip - 1)
                            .at(frame_count))) {

                        // First Freeze lineparamter1 and look for collision points
                        // Then freeze lineparameter2 and find collision point.
                        // Then push_back the two points in the vector

                        cv::Point2f lineparameters1 = list_of_current_objects_combination.at(
                                        obj_index).first->get_line_parameters().at(data_processing_index).at
                                        (frame_skip - 1)
                                .at(frame_count);

                        cv::Point2f temp_line_parameters1 = lineparameters1;


                        cv::Point2f lineparameters2 = list_of_gt_objects_combination.at(
                                obj_index).second->get_line_parameters().at(0).at(frame_skip - 1).at(frame_count);

                        std::cout << "object "
                                << list_of_current_objects_combination.at(obj_index).first->getObjectId()
                                << " = " <<
                                lineparameters1 << " and object " << list_of_gt_objects_combination.at(obj_index)
                                .second->getObjectId() << " = " << lineparameters2 << std::endl;

                        find_collision_points_given_two_line_parameters(lineparameters1,
                                lineparameters2, tempMatrix,
                                frame_collision_points);

                        lineparameters1 = list_of_current_objects_combination.at(
                                        obj_index).second->get_line_parameters().at(data_processing_index).at
                                        (frame_skip - 1)
                                .at(frame_count);

                        lineparameters2 = list_of_gt_objects_combination.at(obj_index).first->get_line_parameters
                                        ().at(0).at(frame_skip - 1)
                                .at(frame_count);

                        std::cout << "object "
                                << list_of_current_objects_combination.at(obj_index).second->getObjectId()
                                << " = " <<
                                lineparameters1 << " and object " << list_of_gt_objects_combination.at(obj_index)
                                .first->getObjectId() << " = " << lineparameters2 << std::endl;

                        find_collision_points_given_two_line_parameters(lineparameters1, lineparameters2,
                                tempMatrix, frame_collision_points);

                        if ( frame_count > 0 ) {
                            assert(temp_line_parameters1!=lineparameters1);
                        }

                        frame_line_angles.push_back(cv::Point2f(temp_line_parameters1.x,lineparameters1.x));


                    } else {

                        frame_collision_points.push_back(cv::Point2f(-1, -1));
                        frame_collision_points.push_back(cv::Point2f(-1, -1));
                        std::cout << "object "
                                << list_of_current_objects_combination.at(obj_index).first->getObjectId()
                                << " visibility = " <<
                                list_of_current_objects_combination.at(
                                                obj_index).first->get_obj_extrapolated_mean_visibility().at(
                                                frame_skip
                                                        - 1)
                                        .at(frame_count) << " and object "
                                << list_of_gt_objects_combination.at(obj_index)
                                        .second->getObjectId() << " visibility = "
                                << list_of_current_objects_combination.at(
                                                obj_index).second->get_obj_extrapolated_mean_visibility().at(
                                                frame_skip
                                                        - 1)
                                        .at(frame_count)
                                << " and hence not generating any collision points for this object combination "
                                << std::endl;
                    }
                }

                for (auto i = 0; i < frame_collision_points.size(); i = i + 2) {
                    if (frame_collision_points.at(i) != cv::Point2f(-1, -1) &&
                            frame_collision_points.at(i + 1) != cv::Point2f(-1, -1)) {
                        frame_collision_points_average.push_back(
                                cv::Point2f(
                                        ((frame_collision_points.at(i).x + frame_collision_points.at(i + 1).x) / 2),
                                        ((frame_collision_points.at(i).y + frame_collision_points.at(i + 1).y) /
                                                2)));
                    }
                    else {
                        /*frame_collision_points_average.push_back(
                                cv::Point2f(-1,-1));*/
                    }
                }

                //Create png Matrix with 3 channels: x mean_displacement. y displacment and ObjectId
                for (int32_t row = 0; row < Dataset::getFrameSize().height; row++) { // rows
                    for (int32_t column = 0; column < Dataset::getFrameSize().width; column++) {  // cols
                        if (tempMatrix.at<cv::Vec3f>(row, column)[2] > 0.5) {
                            F_png_write.setFlowU(column, row, tempMatrix.at<cv::Vec3f>(row, column)[1]);
                            F_png_write.setFlowV(column, row, tempMatrix.at<cv::Vec3f>(row, column)[0]);
                            F_png_write.setObjectId(column, row, tempMatrix.at<cv::Vec3f>(row, column)[2]);
                        }
                    }
                }

                F_png_write.writeExtended(temp_collision_image_path);
    
                outer_frame_collision_points.push_back(frame_collision_points_average);
                outer_frame_line_angles.push_back(frame_line_angles);
            }

            outer_frame_skip_collision_points.push_back(outer_frame_collision_points);
            outer_frame_skip_line_angles.push_back(outer_frame_line_angles);

        }
        m_list_frame_skip_collision_points.push_back(outer_frame_skip_collision_points);
        m_list_frame_skip_line_angles.push_back(outer_frame_skip_line_angles);
    }

    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
    std::cout << m_resultordner + " collision generation done"  << std::endl;
}


void OpticalFlow::find_collision_points_given_two_line_parameters(const cv::Point2f lineparameters1, const cv::Point2f lineparameters2,
        cv::Mat &tempMatrix, std::vector<cv::Point2f> &frame_collision_points) {
    // first fill rowco
    cv::Matx<float,2,2> coefficients (-lineparameters1.x,1,-lineparameters2.x,1);
    cv::Matx<float,2,1> rhs(lineparameters1.y,lineparameters2.y);


    cv::Matx<float,2,1> result_manual;
    if ( cv::determinant(coefficients ) != 0 ) {

        // solve linear equations
        result_manual = (cv::Matx<float,2,2>)coefficients.inv()*rhs;
        //result_manual = coefficients.solve(rhs);

        std::cout << "collision points x = " << result_manual(0,0) << " and y = " << result_manual(1,0) << std::endl ;
        frame_collision_points.push_back(cv::Point2f(result_manual(0,0), result_manual(1,0)));
    }
    else {
        std::cerr << "Determinant is singular" << std::endl;
        //assert ( cv::determinant(coefficients ) != 0 );
        //result_manual(0,0) = -5;
        //result_manual(1,0) = -5;
        frame_collision_points.push_back(cv::Point2f(-1, -1));
    }
};


void OpticalFlow::visualiseStencil(void) {

    std::cout << "visualise stencil at " << m_generatepath.string() + "stencil/" << std::endl;

    char file_name_image[50], file_name_image_output[50];

    cv::Mat image_data_and_shape;

    cv::Mat tempGroundTruthImage(Dataset::getFrameSize(), CV_8UC3);
    FlowImageExtended F_png_write;

    for ( unsigned data_processing_index = 0; data_processing_index < 4; data_processing_index++ ) {

        for (int frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

            for (ushort frame_count = 0; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_IMAGES; frame_count++) {

                std::cout << "frame_count " << frame_count << std::endl;
                std::string output_image_file_with_path;

                /*---------------------------------------------------------------------------------*/
                tempGroundTruthImage = cv::Scalar::all(0);

                sprintf(file_name_image_output, "000%03d_10_stencil_base_algo.png", frame_count * frame_skip);
                output_image_file_with_path =
                        m_generatepath.string() + "stencil/" + file_name_image_output;

                for (unsigned obj_index = 0; obj_index < m_list_simulated_objects_base.size(); obj_index++) {

                    if ((m_list_simulated_objects_base.at(obj_index)->get_obj_extrapolated_visibility().at(
                            frame_skip - 1).at(frame_count)
                    )) {

                        const unsigned CLUSTER_SIZE = (unsigned) m_list_simulated_objects_base.at(
                                obj_index)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                                (frame_skip - 1).at(frame_count).size();
                        for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                            cv::Point2f pts = m_list_simulated_objects_base.at(
                                            obj_index)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                            frame_skip - 1)
                                    .at(frame_count).at(cluster_index).first;

                            cv::circle(tempGroundTruthImage, pts, 1.5, cv::Scalar(255, 255, 255), 1, 8);

                        }
                    }
                }
                cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
                /*---------------------------------------------------------------------------------*/
                tempGroundTruthImage = cv::Scalar::all(255);

                sprintf(file_name_image_output, "000%03d_10_pixel_algo_%d.png", frame_count * frame_skip,
                        data_processing_index);
                output_image_file_with_path =
                        m_generatepath.string() + "stencil/" + file_name_image_output;

                for (unsigned obj_index = 0; obj_index < m_list_simulated_objects.size(); obj_index++) {

                    if ((m_list_simulated_objects.at(obj_index)->get_obj_extrapolated_visibility().at(
                            frame_skip - 1).at(frame_count)
                    )) {

                        const unsigned CLUSTER_SIZE = (unsigned) m_list_simulated_objects.at(
                                obj_index)->get_list_obj_shape_parameters().at(data_processing_index).at(frame_skip - 1).at(
                                frame_count).size();

                        for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                            cv::Point2f pts = m_list_simulated_objects.at(
                                    obj_index)->get_list_obj_shape_parameters().at(data_processing_index).at(
                                    frame_skip - 1).at(frame_count).at(cluster_index).first;
                            cv::circle(tempGroundTruthImage, pts, 1.5, cv::Scalar(0, 0, 255), 1, 8);

                        }
                    }
                }
                cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
                /*---------------------------------------------------------------------------------*/

                tempGroundTruthImage = cv::Scalar::all(255);
                F_png_write = FlowImageExtended(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

                sprintf(file_name_image_output, "000%03d_10_flow_base_%d.png", frame_count * frame_skip,
                        data_processing_index);
                output_image_file_with_path =
                        m_generatepath.string() + "stencil/" + file_name_image_output;

                for (unsigned obj_index = 0; obj_index < m_list_simulated_objects.size(); obj_index++) {

                    if ((m_list_simulated_objects.at(obj_index)->get_obj_extrapolated_visibility().at(
                            frame_skip - 1).at(
                            frame_count)
                    )) {

                        const unsigned CLUSTER_SIZE = (unsigned) m_list_simulated_objects.at(
                                obj_index)->get_list_obj_shape_parameters().at(data_processing_index).at(frame_skip - 1).at(
                                frame_count).size();

                        for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                            cv::Point2f pts = m_list_simulated_objects.at(
                                    obj_index)->get_list_obj_shape_parameters().at(data_processing_index).at(
                                    frame_skip - 1).at(frame_count).at(cluster_index).first;

                            cv::Point2f displacement = m_list_simulated_objects.at(
                                    obj_index)->get_list_obj_shape_parameters().at(data_processing_index).at(
                                    frame_skip - 1).at(frame_count).at(cluster_index).second;

                            cv::Point2f next_pts = cv::Point2f(pts.x + displacement.x, pts.y + displacement.y);

                            F_png_write.setFlowU(pts.x, pts.y, displacement.x);
                            F_png_write.setFlowV(pts.x, pts.y, displacement.y);
                            F_png_write.setValid(pts.x, pts.y, true);

                            //cv::arrowedLine(tempGroundTruthImage, pts, next_pts, cv::Scalar(0, 0, 255), 1, 8, 0, 0.25);
                        }
                    }
                }
                //cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
                F_png_write.writeExtended(output_image_file_with_path);
                /*---------------------------------------------------------------------------------*/

                tempGroundTruthImage = cv::Scalar::all(255);
                F_png_write = FlowImageExtended(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

                sprintf(file_name_image_output, "000%03d_10_flow_algo_%d.png", frame_count * frame_skip,
                        data_processing_index);
                output_image_file_with_path =
                        m_generatepath.string() + "stencil/" + file_name_image_output;

                for (unsigned obj_index = 0; obj_index < m_list_simulated_objects.size(); obj_index++) {

                    if ((m_list_simulated_objects.at(obj_index)->get_obj_extrapolated_visibility().at(
                            frame_skip - 1).at(
                            frame_count)
                    )) {

                        const unsigned CLUSTER_SIZE = (unsigned) m_list_simulated_objects.at(
                                obj_index)->get_list_obj_shape_parameters().at(data_processing_index).at(frame_skip - 1).at(
                                frame_count).size();

                        for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                            cv::Point2f pts = m_list_simulated_objects.at(
                                    obj_index)->get_list_obj_shape_parameters().at(data_processing_index).at(
                                    frame_skip - 1).at(frame_count).at(cluster_index).first;

                            cv::Point2f displacement = m_list_simulated_objects.at(
                                    obj_index)->get_list_obj_shape_parameters().at(data_processing_index).at(
                                    frame_skip - 1).at(frame_count).at(cluster_index).second;

                            cv::Point2f next_pts = cv::Point2f(pts.x + displacement.x, pts.y + displacement.y);

                            F_png_write.setFlowU(pts.x, pts.y, displacement.x);
                            F_png_write.setFlowV(pts.x, pts.y, displacement.y);
                            F_png_write.setValid(pts.x, pts.y, true);

                            //cv::arrowedLine(tempGroundTruthImage, pts, next_pts, cv::Scalar(0, 0, 255), 1, 8, 0, 0.25);
                        }
                    }
                }
                //cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
                F_png_write.writeExtended(output_image_file_with_path);
            }
        }
    }
    std::cout << "end visualise stencil at " << m_generatepath.string() + "stencil/" << std::endl;


    std::cout << "visualise stencil at " << m_generatepath.string() + "/stencil/" << std::endl;

    char file_name_input_image[50];

    for ( int frame_skip = 1; frame_skip <= MAX_SKIPS; frame_skip++ ) {

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

            for (unsigned obj_index = 0; obj_index < m_list_gt_objects.size(); obj_index++) {

                if ((m_list_gt_objects.at(obj_index)->get_obj_base_visibility().at(frame_count))) {

                    float columnBegin = m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (frame_skip-1).at(frame_count).m_region_of_interest_px.x;
                    float rowBegin = m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (frame_skip-1).at(frame_count).m_region_of_interest_px.y;

                    int width = cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_width_m);
                    int height = cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_height_m);

                    int width_roi = cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (frame_skip-1).at(frame_count).m_region_of_interest_px.width);
                    int height_roi = cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (frame_skip-1).at(frame_count).m_region_of_interest_px.height);

                    cv::Rect boundingbox = cv::Rect(
                            cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    frame_skip - 1).at(frame_count).m_object_location_px.location_x_m),
                            cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    frame_skip - 1).at(frame_count).m_object_location_px.location_y_m),
                            cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    frame_skip - 1).at(frame_count).m_object_dimensions_px.dim_width_m),
                            cvRound(m_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    frame_skip - 1).at(frame_count).m_object_dimensions_px.dim_height_m));

                    cv::Rect boundingbox2 = cv::Rect(columnBegin, rowBegin, width, height );
                    cv::Rect boundingbox3 = cv::Rect(columnBegin, rowBegin, width_roi, height_roi);

                    cv::rectangle(tempGroundTruthImage, boundingbox2, cv::Scalar(0, 255, 0), 1, 8, 0);
                    cv::rectangle(tempGroundTruthImage, boundingbox3, cv::Scalar(0, 0, 255), 1, 8, 0);
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
                    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                        cv::Point2f pts = m_list_gt_objects.at(
                                        obj_index)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_index).first;


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
                    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {


                        cv::Point2f pts = m_list_gt_objects.at(
                                        obj_index)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_index).first;

                        cv::Point2f displacement = m_list_gt_objects.at(
                                        obj_index)->get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at(
                                        frame_skip - 1)
                                .at(frame_count).at(cluster_index).second;


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
