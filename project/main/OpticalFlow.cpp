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

void OpticalFlow::generate_collision_points() {

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
        std::cout << "collision between object id " << list_of_gt_objects_combination.at(i).first->getObjectId() <<
                  " and object id "
                  << list_of_gt_objects_combination.at(i).second->getObjectId()<< "\n";
    }

    std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > > list_frame_skip_collision_points;

    for ( unsigned i = 0; i < 4; i++ ) {

        std::vector<std::vector<std::vector<cv::Point2f> > > outer_frame_skip_collision_points;

        for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

            sprintf(frame_skip_folder_suffix, "%02d", frame_skip);

            std::cout << "generating collision points in OpticalFlow.cpp for " << m_resultordner << " " << frame_skip << std::endl;

            std::vector<std::vector<cv::Point2f> >  frame_collision_points;

            unsigned FRAME_COUNT = (unsigned)m_list_gt_objects.at(0)
                    ->get_line_parameters().at(frame_skip-1).at(0).size(); // we store the flow image here and hence it starts at 1. Correspondingly the size reduces.
            assert(FRAME_COUNT>0);

            for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
                char file_name_image[50];
                std::cout << "frame_count " << frame_count << std::endl;

                sprintf(file_name_image, "000%03d_10.png", frame_count*frame_skip);
                std::string temp_collision_image_path =
                        m_collision_obj_path.string() + frame_skip_folder_suffix + "/" + file_name_image;

                fs << "frame_count" << frame_count;

                FlowImageExtended F_png_write(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

                cv::Mat tempMatrix;
                tempMatrix.create(Dataset::getFrameSize(), CV_32FC3);
                tempMatrix = cv::Scalar_<unsigned>(255,255,255);
                assert(tempMatrix.channels() == 3);

                for (unsigned i = 0; i < m_list_gt_objects.size(); i++) {

                    // object image_data_and_shape
                    int width = cvRound(m_list_gt_objects.at(i)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count+1).m_object_dimensions_px.dim_width_m);
                    int height = cvRound(m_list_gt_objects.at(i)->getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count+1).m_object_dimensions_px.dim_height_m);

                    if ( m_list_gt_objects.at(i)->get_obj_extrapolated_mean_visibility().at(frame_skip - 1).at(frame_count)
                         == true ) {

                        cv::Point2f centroid = m_list_gt_objects.at(i)->
                                        get_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement().at(frame_skip-1).at(0)
                                .at(frame_count+1).first;
                        cv::Point2f mean_displacement = m_list_gt_objects.at(i)->
                                        get_list_obj_extrapolated_mean_pixel_centroid_pixel_displacement().at(frame_skip-1).at(0)
                                .at(frame_count+1).second;

                        cv::Mat roi;
                        roi = tempMatrix.
                                colRange(cvRound(centroid.x), cvRound(centroid.x + width)).
                                rowRange(cvRound(centroid.y), cvRound(centroid.y + height));
                        //bulk storage
                        roi = cv::Scalar(mean_displacement.x, mean_displacement.y,
                                         static_cast<float>(m_list_gt_objects.at(i)->getObjectId()));

                        // cv line is intelligent and it can also project to values not within the frame size including negative values.
                        //cv::line(tempMatrix, centroid, gt_line_pts, cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0);
                    }
                }

                std::vector<cv::Point2f> collision_points;

                for ( unsigned i = 0; i < list_of_gt_objects_combination.size(); i++) {

                    if ( ( list_of_gt_objects_combination.at(i).first->get_obj_extrapolated_mean_visibility().at(frame_skip
                                                                                                        - 1)
                                   .at(frame_count+1) == true ) && ( list_of_gt_objects_combination.at(i).second->
                                    get_obj_extrapolated_mean_visibility()
                                                                           .at(frame_skip - 1)
                                                                           .at(frame_count+1) == true )) {

                        // First Freeze lineparamter1 and look for collision points
                        // Then freeze lineparameter2 and find collision point.
                        // Then push_back the two points in the vector
                        cv::Point2f lineparameters1 = list_of_gt_objects_combination.at(i).first->get_line_parameters().at(0).at
                                        (frame_skip - 1)
                                .at(frame_count).first;

                        cv::Point2f lineparameters2 = list_of_gt_objects_combination.at(i).second->get_line_parameters
                                        ().at(0).at(frame_skip - 1)
                                .at(frame_count).first;

                        std::cout << "object " << list_of_gt_objects_combination.at(i).first->getObjectId() << " = " <<
                                  lineparameters1 << " and object " << list_of_gt_objects_combination.at(i)
                                          .second->getObjectId() << " = " <<lineparameters2 << std::endl ;

                        find_collision_points_given_two_line_parameters(lineparameters1, lineparameters2, tempMatrix, collision_points);

                    }
                    else {
                        std::cout << "object " << list_of_gt_objects_combination.at(i).first->getObjectId() << " visibility = " <<
                                list_of_gt_objects_combination.at(i).first->get_obj_extrapolated_mean_visibility().at(frame_skip
                                                - 1)
                                        .at(frame_count+1) << " and object " << list_of_gt_objects_combination.at(i)
                                .second->getObjectId() << " visibility = " << list_of_gt_objects_combination.at(i).second->get_obj_extrapolated_mean_visibility().at(frame_skip
                                        - 1)
                                .at(frame_count+1) << " and hence not generating any collision points for this object combination " <<  std::endl ;
                    }
                }

                frame_collision_points.push_back(collision_points);

                //Create png Matrix with 3 channels: x mean_displacement. y displacment and ObjectId
                for (int32_t row = 0; row < Dataset::getFrameSize().height; row++) { // rows
                    for (int32_t column = 0; column < Dataset::getFrameSize().width; column++) {  // cols
                        if (tempMatrix.at<cv::Vec3f>(row, column)[2] > 0.5 ) {
                            F_png_write.setFlowU(column, row, tempMatrix.at<cv::Vec3f>(row, column)[1]);
                            F_png_write.setFlowV(column, row, tempMatrix.at<cv::Vec3f>(row, column)[0]);
                            F_png_write.setObjectId(column, row, tempMatrix.at<cv::Vec3f>(row, column)[2]);
                            //position.store_in_yaml(fs, cv::Point2f(row, column), cv::Point2f(xValue, yValue) );
                        }
                    }
                }

                F_png_write.writeExtended(temp_collision_image_path);
            }
            outer_frame_skip_collision_points.push_back(frame_collision_points);
        }
        list_frame_skip_collision_points.push_back(outer_frame_skip_collision_points);
    }
    m_list_frame_skip_collision_points = list_frame_skip_collision_points;
    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
    toc_all = steady_clock::now();
    time_map["generate_flow"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << m_resultordner + " collision generation time - " << time_map["generate_flow"] << "ms" << std::endl;
}


void OpticalFlow::find_collision_points_given_two_line_parameters(const cv::Point2f lineparameters1, const cv::Point2f lineparameters2,
        cv::Mat &tempMatrix, std::vector<cv::Point2f> &collision_points) {
    // first fill rowco
    cv::Matx<float,2,2> coefficients (-lineparameters1.x,1,-lineparameters2.x,1);
    cv::Matx<float,2,1> rhs(lineparameters1.y,lineparameters2.y);


    cv::Matx<float,2,1> result_manual;
    if ( cv::determinant(coefficients ) != 0 ) {

        // solve linear equations
        result_manual = (cv::Matx<float,2,2>)coefficients.inv()*rhs;
        //result_manual = coefficients.solve(rhs);
        cv::circle(tempMatrix, cv::Point2f(result_manual(0,0), result_manual(1,0)), 5, cv::Scalar(0, 255, 0), -1,
                cv::LINE_AA);

        std::cout << "collision points x = " << result_manual(0,0) << " and y = " << result_manual(1,0) << std::endl ;
        collision_points.push_back(cv::Point2f(result_manual(0,0), result_manual(1,0)));
    }
    else {
        std::cerr << "Determinant is singular" << std::endl;
        //assert ( cv::determinant(coefficients ) != 0 );
        //result_manual(0,0) = -5;
        //result_manual(1,0) = -5;
        collision_points.push_back(cv::Point2f(-1, -1));
    }

};

