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

    for (int i = 0; i < SENSOR_COUNT; ++i) {

        sprintf(char_dir_append, "%02d", i);

        m_collision_object_path = m_generatepath.string() + "/collision_object_";
        path =  m_collision_object_path.string() + char_dir_append;
        boost::filesystem::create_directories(path);

        m_flow_occ_path = m_generatepath.string() + "/flow_occ_";
        path =  m_flow_occ_path.string() + char_dir_append;
        boost::filesystem::create_directories(path);

        m_edge_path = m_generatepath.string() + "/edge_";
        path =  m_edge_path.string() + char_dir_append;
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


void OpticalFlow::generate_shape_points() {

    std::vector<Objects*> list_of_current_objects;

    unsigned COUNT;
    if ( m_resultordner == "/ground_truth") {
        COUNT = 1;
        list_of_current_objects = m_ptr_list_gt_objects;
    }
    else {
        COUNT = DATAFILTER_COUNT;
        list_of_current_objects = m_ptr_list_simulated_objects;
    }

    char sensor_index_folder_suffix[50];


    for (unsigned datafilter_index = 0; datafilter_index < COUNT; datafilter_index++) {

        std::vector<std::map<std::pair<float, float>, int> > outer_sensor_count_scenario_displacement_occurence;
        std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > > outer_sensor_count_shape_points;

        for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

            std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > outer_frame_shape_points;
            std::map<std::pair<float, float>, int> scenario_displacement_occurence;

            sprintf(sensor_index_folder_suffix, "%02d", sensor_index);

            std::cout << "generating shape points in OpticalFlow.cpp for " << m_resultordner << " " << sensor_index
                      << " for datafilter " << datafilter_index << std::endl;

            unsigned FRAME_COUNT = (unsigned) list_of_current_objects.at(0)
                    ->get_list_object_shapepoints_displacement().at(datafilter_index).at(sensor_index).size();

            assert(FRAME_COUNT > 0);

            for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

                std::cout << "frame_count " << frame_count << " for datafilter_index " << datafilter_index<< std::endl;

                std::vector<std::pair<cv::Point2i, cv::Point2f>> frame_shape_points;

                for (ushort obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++) {

                    auto CLUSTER_COUNT_GT = m_ptr_list_gt_objects.at(
                            obj_index)->get_list_object_shapepoints_displacement().at(0).at(sensor_index).at(frame_count).size();

                    auto CLUSTER_COUNT_ALGO = list_of_current_objects.at(
                            obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(sensor_index).at(frame_count).size();

                    if ( m_resultordner != "/ground_truth" ) {

                        assert( CLUSTER_COUNT_ALGO <= ((CLUSTER_COUNT_GT / mStepSize) + 25 )|| CLUSTER_COUNT_ALGO == CLUSTER_COUNT_GT );

                    }

                    if (list_of_current_objects.at(obj_index)->get_object_mean_visibility().at(sensor_index).at(frame_count) ) {

                        // Instances of CLUSTER_COUNT_ALGO in CLUSTER_COUNT_GT

                        float vollTreffer = 0;
                        float baseTreffer;

                        cv::Point2f gt_displacement = m_ptr_list_gt_objects.at(obj_index)->get_object_pixel_position_pixel_displacement().at
                                (sensor_index).at(frame_count).second;

                        auto dist_gt = cv::norm(gt_displacement);
                        auto angle_gt = std::tanh(gt_displacement.y / gt_displacement.x);

                        if (m_resultordner == "/ground_truth") {

                            vollTreffer = CLUSTER_COUNT_GT;
                            // this is the full resolution ! Because there is no stepSize in GroundTruth
                            baseTreffer = CLUSTER_COUNT_GT;

                        }
                        else {
                            for (auto cluster_count = 0; cluster_count < CLUSTER_COUNT_ALGO; cluster_count++) {

                                cv::Point2f algo_displacement = list_of_current_objects.at(obj_index)->
                                        get_list_object_shapepoints_displacement().at(datafilter_index
                                ).at(sensor_index).at(frame_count).at(cluster_count).second;

                                auto dist_algo = cv::norm(algo_displacement);
                                auto dist_err = std::abs(dist_gt - dist_algo);

                                auto angle_algo = std::tanh(algo_displacement.y/algo_displacement.x);

                                auto angle_err = std::abs(angle_algo - angle_gt);
                                auto angle_err_dot = std::cosh(
                                        algo_displacement.dot(gt_displacement) / (dist_gt * dist_algo));

                                //assert(angle_err_dot==angle_err);

                                if (
                                        (dist_err) < DISTANCE_ERROR_TOLERANCE &&
                                        (angle_err*180/CV_PI) < ANGLE_ERROR_TOLERANCE

                                        ) {
                                    vollTreffer++;
                                }

                            }

                            baseTreffer = ((float) CLUSTER_COUNT_GT) / mStepSize;
                        }
                        frame_shape_points.push_back(std::make_pair(cv::Point2i(frame_count, 0), cv::Point2f(vollTreffer, baseTreffer)));

                        std::cout << "vollTreffer for object " << list_of_current_objects.at(obj_index)->getObjectId() << " = "
                                  << vollTreffer << std::endl;
                        std::cout << "baseTreffer for object " << list_of_current_objects.at(obj_index)->getObjectId() << " = "
                                  << baseTreffer << std::endl;

                        assert(vollTreffer <= std::ceil(baseTreffer) + 20 );

                    } else {
                        std::cout << "visibility of object " << list_of_current_objects.at(obj_index)->getObjectId() << " = " <<
                                  list_of_current_objects.at(obj_index)->get_object_mean_visibility().at(sensor_index)
                                          .at(frame_count)
                                  << " and hence not generating any shape points for this object " << std::endl;

                        frame_shape_points.push_back(std::make_pair(cv::Point2i(frame_count,0), cv::Point2f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));

                    }
                }

                outer_frame_shape_points.push_back(frame_shape_points);
            }
            outer_sensor_count_shape_points.push_back(outer_frame_shape_points);
            outer_sensor_count_scenario_displacement_occurence.push_back(scenario_displacement_occurence);

            /* generate for every algorithm, an extra sensor */
            if ( sensor_index == (SENSOR_COUNT-1) ) {
                generate_shape_points_sensor_fusion(datafilter_index, outer_sensor_count_shape_points );
            }

        }

        m_sensor_count_shape_points.push_back(outer_sensor_count_shape_points);
        m_sensor_count_scenario_displacement_occurence = outer_sensor_count_scenario_displacement_occurence;

    }
    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
}


void OpticalFlow::generate_shape_points_sensor_fusion(const ushort &datafilter_index, std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > >  &outer_sensor_count_shape_points) {

    std::vector<Objects*> list_of_current_objects;

    if ( m_resultordner == "/ground_truth") {
        list_of_current_objects = m_ptr_list_gt_objects;
    }
    else {
        list_of_current_objects = m_ptr_list_simulated_objects;
    }

    std::vector<std::map<std::pair<float, float>, int> > outer_sensor_count_scenario_displacement_occurence;
    std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > outer_frame_shape_points;
    std::map<std::pair<float, float>, int> scenario_displacement_occurence;

    unsigned FRAME_COUNT = (unsigned) list_of_current_objects.at(0)
            ->get_list_object_shapepoints_displacement().at(datafilter_index).at(0).size();

    std::cout << "generating shape points in OpticalFlow.cpp for sensor fusion" << m_resultordner << " for datafilter " << datafilter_index << std::endl;

    for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

        assert(FRAME_COUNT > 0);

        std::vector<std::pair<cv::Point2i, cv::Point2f>> frame_shape_points;
        std::cout << "frame_count " << frame_count << " for datafilter_index " << datafilter_index<< std::endl;

        for (unsigned sensor_index = 0; sensor_index <= 0 ; sensor_index++) {

            for (ushort obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++) {

                auto CLUSTER_COUNT_GT = m_ptr_list_gt_objects.at(
                        obj_index)->get_list_object_shapepoints_displacement().at(0).at(0).at(frame_count).size();

                auto CLUSTER_COUNT_GT_2 = m_ptr_list_gt_objects.at(
                        obj_index)->get_list_object_shapepoints_displacement().at(0).at(1).at(frame_count).size();

                //CLUSTER_COUNT_GT =  ( CLUSTER_COUNT_GT + CLUSTER_COUNT_GT_2 ) /2;
                if (list_of_current_objects.at(obj_index)->get_object_mean_visibility().at(0).at(frame_count) ||
                        list_of_current_objects.at(obj_index)->get_object_mean_visibility().at(1).at(frame_count)) {

                    // Instances of CLUSTER_COUNT_ALGO in CLUSTER_COUNT_GT

                    float vollTreffer = 0;
                    float baseTreffer;

                    cv::Point2f gt_displacement = m_ptr_list_gt_objects.at(obj_index)->get_object_pixel_position_pixel_displacement().at
                            (0).at(frame_count).second;

                    cv::Point2f gt_displacement_2 = m_ptr_list_gt_objects.at(obj_index)->get_object_pixel_position_pixel_displacement().at
                            (1).at(frame_count).second;

                    //gt_displacement = (gt_displacement + gt_displacement_2)/2;

                    auto dist_gt = cv::norm(gt_displacement);
                    auto angle_gt = std::tanh(gt_displacement.y / gt_displacement.x);

                    auto dist_gt_2 = cv::norm(gt_displacement_2);
                    auto angle_gt_2 = std::tanh(gt_displacement_2.y / gt_displacement_2.x);


                    if (m_resultordner == "/ground_truth") {

                        vollTreffer = CLUSTER_COUNT_GT;
                        // this is the full resolution ! Because there is no stepSize in GroundTruth
                        baseTreffer = CLUSTER_COUNT_GT;

                    }
                    else {

                        if (list_of_current_objects.at(obj_index)->get_object_mean_visibility().at(0).at(frame_count)) {

                            auto CLUSTER_COUNT_ALGO = list_of_current_objects.at(
                                    obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(0).at(frame_count).size();

                            for (auto cluster_count = 0; cluster_count < CLUSTER_COUNT_ALGO; cluster_count++) {

                                cv::Point2f algo_displacement = list_of_current_objects.at(obj_index)->
                                        get_list_object_shapepoints_displacement().at(datafilter_index
                                ).at(0).at(frame_count).at(cluster_count).second;

                                auto dist_algo = cv::norm(algo_displacement);
                                auto dist_err = std::abs(dist_gt - dist_algo);

                                auto angle_algo = std::tanh(algo_displacement.y / algo_displacement.x);

                                auto angle_err = std::abs(angle_algo - angle_gt);
                                auto angle_err_dot = std::cosh(
                                        algo_displacement.dot(gt_displacement) / (dist_gt * dist_algo));

                                //assert(angle_err_dot==angle_err);

                                if (
                                        (dist_err) < DISTANCE_ERROR_TOLERANCE &&
                                                (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE

                                        ) {
                                    vollTreffer++;
                                }

                            }

                            baseTreffer = ((float) CLUSTER_COUNT_GT) / mStepSize;
                        }

                        else if (list_of_current_objects.at(obj_index)->get_object_mean_visibility().at(1).at(frame_count)) {

                            auto CLUSTER_COUNT_ALGO_2 = list_of_current_objects.at(
                                    obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(1).at(frame_count).size();

                            for (auto cluster_count = 0; cluster_count < CLUSTER_COUNT_ALGO_2; cluster_count++) {

                                cv::Point2f algo_displacement = list_of_current_objects.at(obj_index)->
                                        get_list_object_shapepoints_displacement().at(datafilter_index
                                ).at(1).at(frame_count).at(cluster_count).second;

                                auto dist_algo = cv::norm(algo_displacement);
                                auto dist_err = std::abs(dist_gt - dist_algo);

                                auto angle_algo = std::tanh(algo_displacement.y / algo_displacement.x);

                                auto angle_err = std::abs(angle_algo - angle_gt_2);
                                auto angle_err_dot = std::cosh(
                                        algo_displacement.dot(gt_displacement_2) / (dist_gt_2 * dist_algo));

                                //assert(angle_err_dot==angle_err);

                                if (
                                        (dist_err) < DISTANCE_ERROR_TOLERANCE &&
                                                (angle_err * 180 / CV_PI) < ANGLE_ERROR_TOLERANCE

                                        ) {
                                    vollTreffer++;
                                }

                            }

                            baseTreffer = ((float) CLUSTER_COUNT_GT_2) / mStepSize;
                        }

                    }
                    frame_shape_points.push_back(std::make_pair(cv::Point2i(frame_count, 0), cv::Point2f(vollTreffer, baseTreffer)));

                    std::cout << "vollTreffer for object " << list_of_current_objects.at(obj_index)->getObjectId() << " = "
                            << vollTreffer << std::endl;
                    std::cout << "baseTreffer for object " << list_of_current_objects.at(obj_index)->getObjectId() << " = "
                            << baseTreffer << std::endl;

                    assert(vollTreffer <= std::ceil(baseTreffer) + 20 );

                } else {
                    std::cout << "visibility of object " << list_of_current_objects.at(obj_index)->getObjectId() << " = " <<
                            list_of_current_objects.at(obj_index)->get_object_mean_visibility().at(sensor_index)
                                    .at(frame_count)
                            << " and hence not generating any shape points for this object " << std::endl;

                    frame_shape_points.push_back(std::make_pair(cv::Point2i(frame_count,0), cv::Point2f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));

                }
            }
        }
        outer_frame_shape_points.push_back(frame_shape_points);

    }
    outer_sensor_count_shape_points.push_back(outer_frame_shape_points);
    outer_sensor_count_scenario_displacement_occurence.push_back(scenario_displacement_occurence);


    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
}



void OpticalFlow::generate_mean_displacement_points() {

    std::vector<Objects*> list_of_current_objects;

    unsigned COUNT;
    if ( m_resultordner == "/ground_truth") {
        COUNT = 1;
        list_of_current_objects = m_ptr_list_gt_objects;
    }
    else {
        COUNT = DATAFILTER_COUNT;
        list_of_current_objects = m_ptr_list_simulated_objects;
    }

    char sensor_index_folder_suffix[50];


    for (unsigned datafilter_index = 0; datafilter_index < COUNT; datafilter_index++) {

        std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > > outer_sensor_count_mean_displacement_points;

        for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

            std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > outer_frame_mean_displacement_points;

            sprintf(sensor_index_folder_suffix, "%02d", sensor_index);

            std::cout << "generating displacement points in OpticalFlow.cpp for " << m_resultordner << " " << sensor_index
                      << " for datafilter " << datafilter_index << std::endl;

            unsigned FRAME_COUNT = (unsigned) list_of_current_objects.at(0)
                    ->get_list_object_mean_centroid_displacement().at(datafilter_index).at(sensor_index).size();

            assert(FRAME_COUNT > 0);

            for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

                std::cout << "frame_count " << frame_count << " for datafilter_index " << datafilter_index<< std::endl;

                std::vector<std::pair<cv::Point2i, cv::Point2f> > frame_mean_displacement_points;

                for (ushort obj_index = 0; obj_index < list_of_current_objects.size(); obj_index++) {

                    cv::Point2i dimension = {
                            cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(frame_count).m_object_dimensions_px.dim_width_m),
                            cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(frame_count).m_object_dimensions_px.dim_height_m)
                    };

                    if (list_of_current_objects.at(obj_index)->get_object_mean_visibility().at(sensor_index).at(frame_count) && frame_count != 0 ) {

                        cv::Point2f displacement = list_of_current_objects.at(obj_index)->
                                get_list_object_mean_centroid_displacement().at(datafilter_index
                        ).at(sensor_index).at(frame_count).second;


                        frame_mean_displacement_points.push_back(std::make_pair(dimension, displacement));

                        std::cout << "mean displacement for object with dimension" << dimension << " = "
                                  << displacement << std::endl;

                    } else {
                        std::cout << "visibility of object " << list_of_current_objects.at(obj_index)->getObjectId() << " = " <<
                                  list_of_current_objects.at(obj_index)->get_object_mean_visibility().at(sensor_index)
                                          .at(frame_count)
                                  << " and hence not generating any displacement points for this object " << std::endl;

                        frame_mean_displacement_points.push_back(std::make_pair(dimension, cv::Point2f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));

                    }
                }

                outer_frame_mean_displacement_points.push_back(frame_mean_displacement_points);
            }

            outer_sensor_count_mean_displacement_points.push_back(outer_frame_mean_displacement_points);

        }
        m_sensor_count_mean_displacement_points.push_back(outer_sensor_count_mean_displacement_points);
    }
    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
}



void OpticalFlow::generate_collision_points() {

    std::vector<Objects*> list_of_current_objects;
    std::vector<std::pair<Objects*, Objects* > > list_of_current_objects_combination;

    std::vector<std::pair<Objects*, Objects*> > list_of_gt_objects_combination;
    std::vector<std::pair<Objects*, Objects* > > list_of_simulated_objects_combination;

    getCombination(m_ptr_list_gt_objects, list_of_gt_objects_combination);
    getCombination(m_ptr_list_simulated_objects, list_of_simulated_objects_combination);

    unsigned COUNT;
    if ( m_resultordner == "/ground_truth") {
        COUNT = 1;
        list_of_current_objects = m_ptr_list_gt_objects;
        list_of_current_objects_combination = list_of_gt_objects_combination;
    }
    else {
        COUNT = DATAFILTER_COUNT;
        list_of_current_objects = m_ptr_list_simulated_objects;
        list_of_current_objects_combination = list_of_simulated_objects_combination;
    }

    char sensor_index_folder_suffix[50];

    for ( ushort obj_index = 0; obj_index < list_of_current_objects_combination.size(); obj_index++ ) {
        std::cout << "collision between object name " << list_of_current_objects_combination.at(obj_index).first->getObjectName() <<
                " and object name "
                << list_of_current_objects_combination.at(obj_index).second->getObjectName()<< "\n";
    }


    for ( unsigned datafilter_index = 0; datafilter_index < COUNT; datafilter_index++ ) {

        std::vector<std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > > outer_sensor_count_collision_points;
        std::vector<std::vector<std::vector<cv::Point2f> > > outer_sensor_count_line_angles;

        for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

            std::vector<std::vector<std::pair<cv::Point2i, cv::Point2f>> > outer_frame_collision_points;
            std::vector<std::vector<cv::Point2f> > outer_frame_line_angles;

            sprintf(sensor_index_folder_suffix, "%02d", sensor_index);

            std::cout << "generating collision points in OpticalFlow.cpp for " << m_resultordner << " " << sensor_index
                    << " for datafilter " << datafilter_index << std::endl;

            unsigned FRAME_COUNT = (unsigned) list_of_current_objects.at(0)
                    ->get_list_object_line_parameters().at(0).at(sensor_index).size();
            
            assert(FRAME_COUNT > 0);

            for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

                std::cout << "frame_count " << frame_count << " for datafilter_index " << datafilter_index<< std::endl;


                std::vector<cv::Point2f> frame_collision_points;
                std::vector<std::pair<cv::Point2i, cv::Point2f>> frame_collision_points_average;
                std::vector<cv::Point2f> frame_line_angles;

                char file_name_image[50];

                sprintf(file_name_image, "000%03d_10.png", frame_count);
                std::string temp_collision_image_path =
                        m_collision_object_path.string() + sensor_index_folder_suffix + "/" + file_name_image;


                FlowImageExtended F_png_write(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

                cv::Mat tempMatrix;
                tempMatrix.create(Dataset::getFrameSize(), CV_32FC3);
                tempMatrix = cv::Scalar_<unsigned>(255, 255, 255);
                assert(tempMatrix.channels() == 3);

                for (unsigned obj_index = 0;
                        obj_index < list_of_current_objects_combination.size(); obj_index++) {

                    if ((list_of_current_objects_combination.at(
                                    obj_index).first->get_object_mean_visibility().at(
                                    sensor_index)
                            .at(frame_count)) && (list_of_current_objects_combination.at(obj_index).second->
                                    get_object_mean_visibility()
                            .at(sensor_index)
                            .at(frame_count))) {

                        // First Freeze lineparamter1 and look for collision points
                        // Then freeze lineparameter2 and find collision point.
                        // Then push_back the two points in the vector

                        cv::Point2f lineparameters1 = list_of_current_objects_combination.at(
                                        obj_index).first->get_list_object_line_parameters().at(datafilter_index).at
                                        (sensor_index)
                                .at(frame_count);

                        cv::Point2f temp_line_parameters1 = lineparameters1;


                        cv::Point2f lineparameters2 = list_of_gt_objects_combination.at(
                                obj_index).second->get_list_object_line_parameters().at(0).at(sensor_index).at(frame_count);

                        std::cout << "object "
                                << list_of_current_objects_combination.at(obj_index).first->getObjectId()
                                << " = " <<
                                lineparameters1 << " and object " << list_of_gt_objects_combination.at(obj_index)
                                .second->getObjectId() << " = " << lineparameters2 << std::endl;

                        find_collision_points_given_two_line_parameters(lineparameters1,
                                lineparameters2, tempMatrix,
                                frame_collision_points);

                        lineparameters1 = list_of_current_objects_combination.at(
                                        obj_index).second->get_list_object_line_parameters().at(datafilter_index).at
                                        (sensor_index)
                                .at(frame_count);

                        lineparameters2 = list_of_gt_objects_combination.at(obj_index).first->get_list_object_line_parameters
                                        ().at(0).at(sensor_index)
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
                                                obj_index).first->get_object_mean_visibility().at(
                                                sensor_index)
                                        .at(frame_count) << " and object "
                                << list_of_gt_objects_combination.at(obj_index)
                                        .second->getObjectId() << " visibility = "
                                << list_of_current_objects_combination.at(
                                                obj_index).second->get_object_mean_visibility().at(
                                                sensor_index)
                                        .at(frame_count)
                                << " and hence not generating any collision points for this object combination "
                                << std::endl;
                    }
                }

                for (auto i = 0; i < frame_collision_points.size(); i = i + 2) {
                    if (frame_collision_points.at(i) != cv::Point2f(-1, -1) &&
                            frame_collision_points.at(i + 1) != cv::Point2f(-1, -1)) {
                        frame_collision_points_average.push_back(std::make_pair(cv::Point2i(frame_count, 0 ),  cv::Point2f(
                                        ((frame_collision_points.at(i).x + frame_collision_points.at(i + 1).x) / 2),
                                        ((frame_collision_points.at(i).y + frame_collision_points.at(i + 1).y) /
                                                2))));
                    }
                    else {
                        frame_collision_points_average.push_back(std::make_pair(cv::Point2i(frame_count, 0 ), cv::Point2f(std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity())));
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

            outer_sensor_count_collision_points.push_back(outer_frame_collision_points);
            outer_sensor_count_line_angles.push_back(outer_frame_line_angles);

        }
        m_list_sensor_count_collision_points.push_back(outer_sensor_count_collision_points);
        m_list_sensor_count_line_angles.push_back(outer_sensor_count_line_angles);
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


void OpticalFlow::visualiseStencil_shape(void) {

    std::cout << "visualise stencil at " << m_generatepath.string() + "stencil/" << std::endl;

    char file_name_image[50], file_name_image_output[50];

    cv::Mat image_data_and_shape;

    cv::Mat tempGroundTruthImage(Dataset::getFrameSize(), CV_8UC3);
    FlowImageExtended F_png_write;

    for ( unsigned datafilter_index = 0; datafilter_index < 4; datafilter_index++ ) {

        for (int sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

            for (ushort frame_count = 0; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_IMAGES; frame_count++) {

                std::cout << "frame_count " << frame_count << std::endl;
                std::string output_image_file_with_path;

                /*---------------------------------------------------------------------------------*/
                tempGroundTruthImage = cv::Scalar::all(0);

                sprintf(file_name_image_output, "000%03d_10_stencil_base_algo.png", frame_count);
                output_image_file_with_path =
                        m_generatepath.string() + "stencil/" + file_name_image_output;

                for (unsigned obj_index = 0; obj_index < m_ptr_list_simulated_objects_base.size(); obj_index++) {

                    if ((m_ptr_list_simulated_objects_base.at(obj_index)->get_object_visibility().at(
                            sensor_index).at(frame_count)
                    )) {

                        const unsigned CLUSTER_SIZE = (unsigned) m_ptr_list_simulated_objects_base.at(
                                obj_index)->get_object_stencil_point_displacement().at
                                (sensor_index).at(frame_count).size();
                        for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                            cv::Point2f pts = m_ptr_list_simulated_objects_base.at(
                                            obj_index)->get_object_stencil_point_displacement().at(
                                            sensor_index)
                                    .at(frame_count).at(cluster_index).first;

                            cv::circle(tempGroundTruthImage, pts, 1.5, cv::Scalar(255, 255, 255), 1, 8);

                        }
                    }
                }
                cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
                /*---------------------------------------------------------------------------------*/
                tempGroundTruthImage = cv::Scalar::all(255);

                sprintf(file_name_image_output, "000%03d_10_pixel_algo_%d.png", frame_count,
                        datafilter_index);
                output_image_file_with_path =
                        m_generatepath.string() + "stencil/" + file_name_image_output;

                for (unsigned obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++) {

                    if ((m_ptr_list_simulated_objects.at(obj_index)->get_object_visibility().at(
                            sensor_index).at(frame_count)
                    )) {

                        const unsigned CLUSTER_SIZE = (unsigned) m_ptr_list_simulated_objects.at(
                                obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(sensor_index).at(
                                frame_count).size();

                        for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                            cv::Point2f pts = m_ptr_list_simulated_objects.at(
                                    obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(
                                    sensor_index).at(frame_count).at(cluster_index).first;
                            cv::circle(tempGroundTruthImage, pts, 1.5, cv::Scalar(0, 0, 255), 1, 8);

                        }
                    }
                }
                cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
                /*---------------------------------------------------------------------------------*/

                tempGroundTruthImage = cv::Scalar::all(255);
                F_png_write = FlowImageExtended(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

                sprintf(file_name_image_output, "000%03d_10_flow_base_%d.png", frame_count,
                        datafilter_index);
                output_image_file_with_path =
                        m_generatepath.string() + "stencil/" + file_name_image_output;

                for (unsigned obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++) {

                    if ((m_ptr_list_simulated_objects.at(obj_index)->get_object_visibility().at(
                            sensor_index).at(
                            frame_count)
                    )) {

                        const unsigned CLUSTER_SIZE = (unsigned) m_ptr_list_simulated_objects.at(
                                obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(sensor_index).at(
                                frame_count).size();

                        for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                            cv::Point2f pts = m_ptr_list_simulated_objects.at(
                                    obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(
                                    sensor_index).at(frame_count).at(cluster_index).first;

                            cv::Point2f displacement = m_ptr_list_simulated_objects.at(
                                    obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(
                                    sensor_index).at(frame_count).at(cluster_index).second;

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

                sprintf(file_name_image_output, "000%03d_10_flow_algo_%d.png", frame_count,
                        datafilter_index);
                output_image_file_with_path =
                        m_generatepath.string() + "stencil/" + file_name_image_output;

                for (unsigned obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++) {

                    if ((m_ptr_list_simulated_objects.at(obj_index)->get_object_visibility().at(
                            sensor_index).at(
                            frame_count)
                    )) {

                        const unsigned CLUSTER_SIZE = (unsigned) m_ptr_list_simulated_objects.at(
                                obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(sensor_index).at(
                                frame_count).size();

                        for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                            cv::Point2f pts = m_ptr_list_simulated_objects.at(
                                    obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(
                                    sensor_index).at(frame_count).at(cluster_index).first;

                            cv::Point2f displacement = m_ptr_list_simulated_objects.at(
                                    obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(
                                    sensor_index).at(frame_count).at(cluster_index).second;

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

    for ( int sensor_index = 0; sensor_index <= SENSOR_COUNT; sensor_index++ ) {

        for (ushort frame_count = 0; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_IMAGES; frame_count++)
        {

            std::cout << "frame_count " << frame_count << std::endl;
            std::string output_image_file_with_path;

            /*---------------------------------------------------------------------------------*/
            tempGroundTruthImage = cv::Scalar::all(0);
            sprintf(file_name_input_image, "000%03d_10.png", frame_count);

            std::string input_image_file_with_path = mImageabholOrt.string() + "/" +
                    file_name_input_image;

            tempGroundTruthImage = cv::imread(input_image_file_with_path, CV_LOAD_IMAGE_COLOR);

            sprintf(file_name_image_output, "000%03d_10_bb.png", frame_count);
            output_image_file_with_path =
                    m_generatepath.string() + "/stencil/" + file_name_image_output;

            for (unsigned obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++) {

                if ((m_ptr_list_gt_objects.at(obj_index)->get_object_base_visibility().at(frame_count))) {

                    float columnBegin = m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (sensor_index).at(frame_count).m_region_of_interest_px.x;
                    float rowBegin = m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (sensor_index).at(frame_count).m_region_of_interest_px.y;

                    int width = cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(frame_count).m_object_dimensions_px.dim_width_m);
                    int height = cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(frame_count).m_object_dimensions_px.dim_height_m);

                    int width_roi = cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (sensor_index).at(frame_count).m_region_of_interest_px.width);
                    int height_roi = cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (sensor_index).at(frame_count).m_region_of_interest_px.height);

                    cv::Rect boundingbox = cv::Rect(
                            cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    sensor_index).at(frame_count).m_object_location_px.location_x_m),
                            cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    sensor_index).at(frame_count).m_object_location_px.location_y_m),
                            cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    sensor_index).at(frame_count).m_object_dimensions_px.dim_width_m),
                            cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    sensor_index).at(frame_count).m_object_dimensions_px.dim_height_m));

                    cv::Rect boundingbox2 = cv::Rect(columnBegin, rowBegin, width, height );
                    cv::Rect boundingbox3 = cv::Rect(columnBegin, rowBegin, width_roi, height_roi);

                    cv::rectangle(tempGroundTruthImage, boundingbox2, cv::Scalar(0, 255, 0), 1, 8, 0);
                    cv::rectangle(tempGroundTruthImage, boundingbox3, cv::Scalar(0, 0, 255), 1, 8, 0);
                    //cv::rectangle(tempGroundTruthImage, boundingbox3, cv::Scalar(255, 0, 0), 1, 8, 0);

                    cv::Point2f pts_mean = m_ptr_list_gt_objects.at(obj_index)->get_list_object_mean_centroid_displacement().at(sensor_index).
                            at(0).at(frame_count).first;
                    cv::Point2f pts_basic = m_ptr_list_gt_objects.at(obj_index)->get_object_pixel_position_pixel_displacement().at(
                            sensor_index).at(frame_count).first;

                    cv::Point2f displacement = m_ptr_list_gt_objects.at(obj_index)->get_list_object_mean_centroid_displacement().at(sensor_index).at(
                            0).at(frame_count).second;

                    cv::Point2f next_pts = cv::Point2f(pts_basic.x + displacement.x*10, pts_basic.y + displacement.y*10);

                    cv::arrowedLine(tempGroundTruthImage, pts_basic, next_pts, cv::Scalar(0,0,255), 2);


                }
            }
            cv::imwrite(output_image_file_with_path, tempGroundTruthImage);

            /*---------------------------------------------------------------------------------*/
            tempGroundTruthImage = cv::Scalar::all(255);

            sprintf(file_name_image_output, "000%03d_10_pixel_gt.png", frame_count);
            output_image_file_with_path =
                    m_generatepath.string() + "/stencil/" + file_name_image_output;

            for (unsigned obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++) {

                if ((m_ptr_list_gt_objects.at(obj_index)->get_object_base_visibility().at(frame_count))
                        ) {

                    const unsigned CLUSTER_SIZE = (unsigned) m_ptr_list_gt_objects.at(
                            obj_index)->get_object_stencil_point_displacement().at
                            (sensor_index).at(frame_count).size();
                    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                        cv::Point2f pts = m_ptr_list_gt_objects.at(
                                        obj_index)->get_object_stencil_point_displacement().at(
                                        sensor_index)
                                .at(frame_count).at(cluster_index).first;


                        cv::circle(tempGroundTruthImage, pts, 1.5, cv::Scalar(0, 255, 0), 1, 8);

                    }
                }
            }
            //cv::imwrite(output_image_file_with_path, tempGroundTruthImage);

            /*---------------------------------------------------------------------------------*/
            tempGroundTruthImage = cv::Scalar::all(255);
            F_png_write = FlowImageExtended(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

            sprintf(file_name_image_output, "000%03d_10_flow_gt.png", frame_count);
            output_image_file_with_path =
                    m_generatepath.string() + "/stencil/" + file_name_image_output;

            for (unsigned obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++) {

                if ((m_ptr_list_gt_objects.at(obj_index)->get_object_visibility().at(sensor_index).at(
                        frame_count)
                )) {

                    unsigned CLUSTER_SIZE = (unsigned) m_ptr_list_gt_objects.at(
                            obj_index)->get_object_stencil_point_displacement().at
                            (sensor_index).at(frame_count).size();
                    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {


                        cv::Point2f pts = m_ptr_list_gt_objects.at(
                                        obj_index)->get_object_stencil_point_displacement().at(
                                        sensor_index)
                                .at(frame_count).at(cluster_index).first;

                        cv::Point2f displacement = m_ptr_list_gt_objects.at(
                                        obj_index)->get_object_stencil_point_displacement().at(
                                        sensor_index)
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



void OpticalFlow::visualiseStencil_line(void) {

    std::cout << "visualise stencil at " << m_generatepath.string() + "stencil/" << std::endl;

    char file_name_image[50], file_name_image_output[50];

    cv::Mat image_data_and_shape;

    cv::Mat tempGroundTruthImage(Dataset::getFrameSize(), CV_8UC3);
    FlowImageExtended F_png_write;

    for ( unsigned datafilter_index = 0; datafilter_index < 4; datafilter_index++ ) {

        for (int sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

            for (ushort frame_count = 0; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_IMAGES; frame_count++) {

                std::cout << "frame_count " << frame_count << std::endl;
                std::string output_image_file_with_path;

                /*---------------------------------------------------------------------------------*/
                tempGroundTruthImage = cv::Scalar::all(0);

                sprintf(file_name_image_output, "000%03d_10_stencil_base_algo.png", frame_count);
                output_image_file_with_path =
                        m_generatepath.string() + "stencil/" + file_name_image_output;

                for (unsigned obj_index = 0; obj_index < m_ptr_list_simulated_objects_base.size(); obj_index++) {

                    if ((m_ptr_list_simulated_objects_base.at(obj_index)->get_object_visibility().at(
                            sensor_index).at(frame_count)
                    )) {

                        const unsigned CLUSTER_SIZE = (unsigned) m_ptr_list_simulated_objects_base.at(
                                obj_index)->get_object_stencil_point_displacement().at
                                (sensor_index).at(frame_count).size();
                        for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                            cv::Point2f pts = m_ptr_list_simulated_objects_base.at(
                                            obj_index)->get_object_stencil_point_displacement().at(
                                            sensor_index)
                                    .at(frame_count).at(cluster_index).first;

                            cv::circle(tempGroundTruthImage, pts, 1.5, cv::Scalar(255, 255, 255), 1, 8);

                        }
                    }
                }
                cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
                /*---------------------------------------------------------------------------------*/
                tempGroundTruthImage = cv::Scalar::all(255);

                sprintf(file_name_image_output, "000%03d_10_pixel_algo_%d.png", frame_count,
                        datafilter_index);
                output_image_file_with_path =
                        m_generatepath.string() + "stencil/" + file_name_image_output;

                for (unsigned obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++) {

                    if ((m_ptr_list_simulated_objects.at(obj_index)->get_object_visibility().at(
                            sensor_index).at(frame_count)
                    )) {

                        const unsigned CLUSTER_SIZE = (unsigned) m_ptr_list_simulated_objects.at(
                                obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(sensor_index).at(
                                frame_count).size();

                        for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                            cv::Point2f pts = m_ptr_list_simulated_objects.at(
                                    obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(
                                    sensor_index).at(frame_count).at(cluster_index).first;
                            cv::circle(tempGroundTruthImage, pts, 1.5, cv::Scalar(0, 0, 255), 1, 8);

                        }
                    }
                }
                cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
                /*---------------------------------------------------------------------------------*/

                tempGroundTruthImage = cv::Scalar::all(255);
                F_png_write = FlowImageExtended(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

                sprintf(file_name_image_output, "000%03d_10_flow_base_%d.png", frame_count,
                        datafilter_index);
                output_image_file_with_path =
                        m_generatepath.string() + "stencil/" + file_name_image_output;

                for (unsigned obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++) {

                    if ((m_ptr_list_simulated_objects.at(obj_index)->get_object_visibility().at(
                            sensor_index).at(
                            frame_count)
                    )) {

                        const unsigned CLUSTER_SIZE = (unsigned) m_ptr_list_simulated_objects.at(
                                obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(sensor_index).at(
                                frame_count).size();

                        for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                            cv::Point2f pts = m_ptr_list_simulated_objects.at(
                                    obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(
                                    sensor_index).at(frame_count).at(cluster_index).first;

                            cv::Point2f displacement = m_ptr_list_simulated_objects.at(
                                    obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(
                                    sensor_index).at(frame_count).at(cluster_index).second;

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

                sprintf(file_name_image_output, "000%03d_10_flow_algo_%d.png", frame_count,
                        datafilter_index);
                output_image_file_with_path =
                        m_generatepath.string() + "stencil/" + file_name_image_output;

                for (unsigned obj_index = 0; obj_index < m_ptr_list_simulated_objects.size(); obj_index++) {

                    if ((m_ptr_list_simulated_objects.at(obj_index)->get_object_visibility().at(
                            sensor_index).at(
                            frame_count)
                    )) {

                        const unsigned CLUSTER_SIZE = (unsigned) m_ptr_list_simulated_objects.at(
                                obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(sensor_index).at(
                                frame_count).size();

                        for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                            cv::Point2f pts = m_ptr_list_simulated_objects.at(
                                    obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(
                                    sensor_index).at(frame_count).at(cluster_index).first;

                            cv::Point2f displacement = m_ptr_list_simulated_objects.at(
                                    obj_index)->get_list_object_shapepoints_displacement().at(datafilter_index).at(
                                    sensor_index).at(frame_count).at(cluster_index).second;

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

    for ( int sensor_index = 0; sensor_index <= SENSOR_COUNT; sensor_index++ ) {

        for (ushort frame_count = 0; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_IMAGES; frame_count++)
        {

            std::cout << "frame_count " << frame_count << std::endl;
            std::string output_image_file_with_path;

            /*---------------------------------------------------------------------------------*/
            tempGroundTruthImage = cv::Scalar::all(0);
            sprintf(file_name_input_image, "000%03d_10.png", frame_count);

            std::string input_image_file_with_path = mImageabholOrt.string() + "/" +
                                                     file_name_input_image;

            tempGroundTruthImage = cv::imread(input_image_file_with_path, CV_LOAD_IMAGE_COLOR);

            sprintf(file_name_image_output, "000%03d_10_bb.png", frame_count);
            output_image_file_with_path =
                    m_generatepath.string() + "/stencil/" + file_name_image_output;

            for (unsigned obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++) {

                if ((m_ptr_list_gt_objects.at(obj_index)->get_object_base_visibility().at(frame_count))) {

                    float columnBegin = m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (sensor_index).at(frame_count).m_region_of_interest_px.x;
                    float rowBegin = m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (sensor_index).at(frame_count).m_region_of_interest_px.y;

                    int width = cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(frame_count).m_object_dimensions_px.dim_width_m);
                    int height = cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(frame_count).m_object_dimensions_px.dim_height_m);

                    int width_roi = cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (sensor_index).at(frame_count).m_region_of_interest_px.width);
                    int height_roi = cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (sensor_index).at(frame_count).m_region_of_interest_px.height);

                    cv::Rect boundingbox = cv::Rect(
                            cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    sensor_index).at(frame_count).m_object_location_px.location_x_m),
                            cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    sensor_index).at(frame_count).m_object_location_px.location_y_m),
                            cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    sensor_index).at(frame_count).m_object_dimensions_px.dim_width_m),
                            cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(
                                    sensor_index).at(frame_count).m_object_dimensions_px.dim_height_m));

                    cv::Rect boundingbox2 = cv::Rect(columnBegin, rowBegin, width, height );
                    cv::Rect boundingbox3 = cv::Rect(columnBegin, rowBegin, width_roi, height_roi);

                    cv::rectangle(tempGroundTruthImage, boundingbox2, cv::Scalar(0, 255, 0), 1, 8, 0);
                    cv::rectangle(tempGroundTruthImage, boundingbox3, cv::Scalar(0, 0, 255), 1, 8, 0);
                    //cv::rectangle(tempGroundTruthImage, boundingbox3, cv::Scalar(255, 0, 0), 1, 8, 0);

                    cv::Point2f pts_mean = m_ptr_list_gt_objects.at(obj_index)->get_list_object_mean_centroid_displacement().at(sensor_index).
                            at(0).at(frame_count).first;
                    cv::Point2f pts_basic = m_ptr_list_gt_objects.at(obj_index)->get_object_pixel_position_pixel_displacement().at(
                            sensor_index).at(frame_count).first;

                    cv::Point2f displacement = m_ptr_list_gt_objects.at(obj_index)->get_list_object_mean_centroid_displacement().at(sensor_index).at(
                            0).at(frame_count).second;

                    cv::Point2f next_pts = cv::Point2f(pts_basic.x + displacement.x*10, pts_basic.y + displacement.y*10);

                    cv::arrowedLine(tempGroundTruthImage, pts_basic, next_pts, cv::Scalar(0,0,255), 2);


                }
            }
            cv::imwrite(output_image_file_with_path, tempGroundTruthImage);

            /*---------------------------------------------------------------------------------*/
            tempGroundTruthImage = cv::Scalar::all(255);

            sprintf(file_name_image_output, "000%03d_10_pixel_gt.png", frame_count);
            output_image_file_with_path =
                    m_generatepath.string() + "/stencil/" + file_name_image_output;

            for (unsigned obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++) {

                if ((m_ptr_list_gt_objects.at(obj_index)->get_object_base_visibility().at(frame_count))
                        ) {

                    const unsigned CLUSTER_SIZE = (unsigned) m_ptr_list_gt_objects.at(
                            obj_index)->get_object_stencil_point_displacement().at
                            (sensor_index).at(frame_count).size();
                    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

                        cv::Point2f pts = m_ptr_list_gt_objects.at(
                                        obj_index)->get_object_stencil_point_displacement().at(
                                        sensor_index)
                                .at(frame_count).at(cluster_index).first;


                        cv::circle(tempGroundTruthImage, pts, 1.5, cv::Scalar(0, 255, 0), 1, 8);

                    }
                }
            }
            //cv::imwrite(output_image_file_with_path, tempGroundTruthImage);

            /*---------------------------------------------------------------------------------*/
            tempGroundTruthImage = cv::Scalar::all(255);
            F_png_write = FlowImageExtended(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

            sprintf(file_name_image_output, "000%03d_10_flow_gt.png", frame_count);
            output_image_file_with_path =
                    m_generatepath.string() + "/stencil/" + file_name_image_output;

            for (unsigned obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++) {

                if ((m_ptr_list_gt_objects.at(obj_index)->get_object_visibility().at(sensor_index).at(
                        frame_count)
                )) {

                    unsigned CLUSTER_SIZE = (unsigned) m_ptr_list_gt_objects.at(
                            obj_index)->get_object_stencil_point_displacement().at
                            (sensor_index).at(frame_count).size();
                    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {


                        cv::Point2f pts = m_ptr_list_gt_objects.at(
                                        obj_index)->get_object_stencil_point_displacement().at(
                                        sensor_index)
                                .at(frame_count).at(cluster_index).first;

                        cv::Point2f displacement = m_ptr_list_gt_objects.at(
                                        obj_index)->get_object_stencil_point_displacement().at(
                                        sensor_index)
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

