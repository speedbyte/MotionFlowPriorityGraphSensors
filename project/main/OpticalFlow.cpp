//
// Created by veikas on 06.02.18.
//

#include <map>
#include <chrono>
#include "OpticalFlow.h"
#include "FlowImageExtended.h"

using namespace std::chrono;

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

        m_trajectory_occ_path = m_generatepath.string() + "/trajectory_occ_";
        path =  m_trajectory_occ_path.string() + char_dir_append;
        boost::filesystem::create_directories(path);

        m_plots_path = m_generatepath.string() + "/plots_";
        path =  m_plots_path.string() + char_dir_append;
        boost::filesystem::create_directories(path);

    }
}

void OpticalFlow::generate_shape_points() {

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the trajectory in a png file

    std::vector<std::pair<GroundTruthObjects, GroundTruthObjects > > list_of_gt_objects_combination;
    std::vector<std::pair<SimulatedObjects, SimulatedObjects > > list_of_simulated_objects_combination;

    getCombination(list_of_gt_objects_combination, list_of_simulated_objects_combination);

    std::map<std::string, double> time_map = {{"generate",     0},
                                              {"generate_flow", 0}};

    auto tic = steady_clock::now();
    auto toc = steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();

    char frame_skip_folder_suffix[50];
    cv::FileStorage fs;

    for ( ushort i = 0; i < list_of_simulated_objects_combination.size(); i ++ ) {
        std::cout << "shape between object id " << list_of_simulated_objects_combination.at(i).first.getObjectId() <<
                " and object id "
                << list_of_simulated_objects_combination.at(i).second.getObjectId()<< "\n";
    }


    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

        sprintf(frame_skip_folder_suffix, "%02d", frame_skip);

        std::cout << "generating shape points in OpticalFlow.cpp for " << m_resultordner << " " << frame_skip << std::endl;

        std::vector<std::vector<cv::Point2f> >  m_frame_shape_points;

        unsigned FRAME_COUNT = (unsigned)m_list_simulated_objects.at(0)
                .get_obj_extrapolated_shape_pixel_point_pixel_displacement().at(frame_skip - 1).size();
        assert(FRAME_COUNT>0);

        for (ushort frame_count = 1; frame_count < FRAME_COUNT; frame_count++) {

            std::cout << "frame_count " << frame_count << std::endl;

            fs << "frame_count" << frame_count;

            std::vector<cv::Point2f> shape_points;
            std::vector<cv::Point2f> shape_points_average;

            int width = m_list_gt_objects.at(0).getWidth();
            int height = m_list_gt_objects.at(0).getHeight();


            for ( unsigned i = 0; i < m_list_simulated_objects.size(); i++) {

                if ( ( m_list_simulated_objects.at(i).get_obj_extrapolated_visibility().at(frame_skip - 1)
                               .at(frame_count) == true ) ) {

                    auto CLUSTER_COUNT_GT = m_list_gt_objects.at(i).get_obj_extrapolated_shape_pixel_point_pixel_displacement().at
                            (frame_skip - 1).at(frame_count).size();

                    auto CLUSTER_COUNT_ALGO = m_list_simulated_objects.at(i).get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                            (frame_skip - 1).at(frame_count).size();

                    float rowBegin = m_list_gt_objects.at(i).get_obj_extrapolated_pixel_point_pixel_displacement().at
                            (frame_skip-1).at(frame_count).first.y;
                    float columnBegin = m_list_gt_objects.at(i).get_obj_extrapolated_pixel_point_pixel_displacement().at
                            (frame_skip-1).at(frame_count).first.x;


                    // Instances of CLUSTER_COUNT_ALGO in CLUSTER_COUNT_GT

                    ushort vollTreffer = 0;

                    for ( auto j = 0; j < CLUSTER_COUNT_ALGO; j++ ) {
                        auto x_coordinates =  m_list_simulated_objects.at(i).get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                                (frame_skip - 1).at(frame_count).at(j).first.x;
                        auto y_coordinates = m_list_simulated_objects.at(i).get_obj_extrapolated_stencil_pixel_point_pixel_displacement().at
                                (frame_skip - 1).at(frame_count).at(j).first.y;


                        if  (( x_coordinates > columnBegin ) &&
                                ( x_coordinates < (columnBegin + width )) &&
                                        (  y_coordinates > rowBegin )  &&
                                        ( y_coordinates > ( rowBegin + height ))
                                ) {
                            vollTreffer++;
                        }
                    }

                    float keinTreffer = (CLUSTER_COUNT_ALGO - vollTreffer);
                    shape_points.push_back(cv::Point2f(vollTreffer, keinTreffer));

                    std::cout << "object vollTreffer" << m_list_simulated_objects.at(i).getObjectId() << " = " << vollTreffer << std::endl ;

                }
            }


            shape_points_average = shape_points;

            m_frame_shape_points.push_back(shape_points_average);

        }
        m_frame_skip_shape_points.push_back(m_frame_shape_points);
    }

    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
    toc_all = steady_clock::now();
    time_map["generate_flow"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << m_resultordner + " flow generation time - " << time_map["generate_flow"] << "ms" << std::endl;
}

void OpticalFlow::getCombination( std::vector<std::pair<GroundTruthObjects, GroundTruthObjects > > &list_of_gt_objects_combination,
std::vector<std::pair<SimulatedObjects, SimulatedObjects > > &list_of_simulated_objects_combination
) {
    std::vector<GroundTruthObjects>::const_iterator objectIterator = m_list_gt_objects.begin();
    std::vector<GroundTruthObjects>::const_iterator  objectIteratorNext;

    for ( ; objectIterator < m_list_gt_objects.end() ; objectIterator++ ) {
        for ( objectIteratorNext = objectIterator+1; objectIteratorNext < m_list_gt_objects.end();
                objectIteratorNext++) {

            list_of_gt_objects_combination.push_back(std::make_pair(((*objectIterator)),
                    ((*objectIteratorNext))));
        }
    }


    std::vector<SimulatedObjects>::const_iterator objectIteratorSim = m_list_simulated_objects.begin();
    std::vector<SimulatedObjects>::const_iterator  objectIteratorNextSim;

    for ( ; objectIteratorSim < m_list_simulated_objects.end() ; objectIteratorSim++ ) {
        for ( objectIteratorNextSim = objectIteratorSim+1; objectIteratorNextSim < m_list_simulated_objects.end();
                objectIteratorNextSim++) {

            list_of_simulated_objects_combination.push_back(std::make_pair(((*objectIteratorSim)),
                    ((*objectIteratorNextSim))));
        }
    }

}

void OpticalFlow::generate_collision_points() {

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the trajectory in a png file

    std::vector<std::pair<GroundTruthObjects, GroundTruthObjects > > list_of_gt_objects_combination;
    std::vector<std::pair<SimulatedObjects, SimulatedObjects > > list_of_simulated_objects_combination;

    getCombination(list_of_gt_objects_combination, list_of_simulated_objects_combination);


    std::map<std::string, double> time_map = {{"generate",     0},
                                              {"generate_flow", 0}};

    auto tic = steady_clock::now();
    auto toc = steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();

    char frame_skip_folder_suffix[50];
    cv::FileStorage fs;


    for ( ushort i = 0; i < list_of_gt_objects_combination.size(); i ++ ) {
        std::cout << "collision between object id " << list_of_gt_objects_combination.at(i).first.getObjectId() <<
                  " and object id "
                  << list_of_gt_objects_combination.at(i).second.getObjectId()<< "\n";
    }


    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

        sprintf(frame_skip_folder_suffix, "%02d", frame_skip);

        std::cout << "generating collision points in OpticalFlow.cpp for " << m_resultordner << " " << frame_skip << std::endl;

        std::vector<std::vector<cv::Point2f> >  m_frame_collision_points;

        unsigned FRAME_COUNT = (unsigned)m_list_gt_objects.at(0)
                .get_obj_extrapolated_pixel_centroid_pixel_displacement_mean().at
                        (frame_skip - 1).size() - 1; // we store the flow image here and hence it starts at 1. Correspondingly the size reduces.
        assert(FRAME_COUNT>0);

        for (ushort frame_count = 1; frame_count < FRAME_COUNT; frame_count++) {
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
                int width = m_list_gt_objects.at(i).getWidth();
                int height = m_list_gt_objects.at(i).getHeight();

                if ( m_list_gt_objects.at(i).get_obj_extrapolated_visibility().at(frame_skip - 1).at(frame_count)
                     == true ) {

                    cv::Point2f centroid = m_list_gt_objects.at(i).
                                    get_obj_extrapolated_pixel_centroid_pixel_displacement_mean().at(frame_skip - 1)
                            .at(frame_count).first;
                    cv::Point2f mean_displacement = m_list_gt_objects.at(i).
                                    get_obj_extrapolated_pixel_centroid_pixel_displacement_mean().at(frame_skip- 1)
                            .at(frame_count).second;

                    cv::Point2f gt_line_pts = m_list_gt_objects.at(i).get_line_parameters().at(frame_skip - 1)
                            .at(frame_count-1).second;  //line parameters run one less than the others.

                    cv::Mat roi;
                    roi = tempMatrix.
                            colRange(cvRound(centroid.x), cvRound(centroid.x + width)).
                            rowRange(cvRound(centroid.y), cvRound(centroid.y + height));
                    //bulk storage
                    roi = cv::Scalar(mean_displacement.x, mean_displacement.y,
                                     static_cast<float>(m_list_gt_objects.at(i).getObjectId()));

                    // cv line is intelligent and it can also project to values not within the frame size including negative values.
                    cv::line(tempMatrix, centroid, gt_line_pts, cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0);
                }
            }

            std::vector<cv::Point2f> collision_points;

            for ( unsigned i = 0; i < list_of_gt_objects_combination.size(); i++) {

                if ( ( list_of_gt_objects_combination.at(i).first.get_obj_extrapolated_visibility().at(frame_skip
                                                                                                    - 1)
                               .at(frame_count) == true ) && ( list_of_gt_objects_combination.at(i).second.
                                get_obj_extrapolated_visibility()
                                                                       .at(frame_skip - 1)
                                                                       .at(frame_count) == true )) {

                    // First Freeze lineparamter1 and look for collision points
                    // Then freeze lineparameter2 and find collision point.
                    // Then push_back the two points in the vector
                    cv::Point2f lineparameters1 = list_of_gt_objects_combination.at(i).first.get_line_parameters().at
                                    (frame_skip - 1)
                            .at(frame_count-1).first;

                    cv::Point2f lineparameters2 = list_of_gt_objects_combination.at(i).second.get_line_parameters
                                    ().at(frame_skip - 1)
                            .at(frame_count-1).first;

                    std::cout << "object " << list_of_gt_objects_combination.at(i).first.getObjectId() << " = " <<
                              lineparameters1 << " and object " << list_of_gt_objects_combination.at(i)
                                      .second.getObjectId() << " = " <<lineparameters2 << std::endl ;

                    find_collision_points_given_two_line_parameters(lineparameters1, lineparameters2, tempMatrix, collision_points);

                }
            }

            m_frame_collision_points.push_back(collision_points);

            //Create png Matrix with 3 channels: x mean_displacement. y displacment and ObjectId
            for (int32_t row = 0; row < Dataset::getFrameSize().height; row++) { // rows
                for (int32_t column = 0; column < Dataset::getFrameSize().width; column++) {  // cols
                    if (tempMatrix.at<cv::Vec3f>(row, column)[2] > 0.5 ) {
                        F_png_write.setFlowU(column, row, tempMatrix.at<cv::Vec3f>(row, column)[1]);
                        F_png_write.setFlowV(column, row, tempMatrix.at<cv::Vec3f>(row, column)[0]);
                        F_png_write.setObjectId(column, row, tempMatrix.at<cv::Vec3f>(row, column)[2]);
                        //trajectory.store_in_yaml(fs, cv::Point2f(row, column), cv::Point2f(xValue, yValue) );
                    }
                }
            }

            F_png_write.writeExtended(temp_collision_image_path);

        }
        m_frame_skip_collision_points.push_back(m_frame_collision_points);
    }

    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
    toc_all = steady_clock::now();
    time_map["generate_flow"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << m_resultordner + " flow generation time - " << time_map["generate_flow"] << "ms" << std::endl;
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

void OpticalFlow::generate_collision_points_mean() {


    std::vector<std::pair<GroundTruthObjects, GroundTruthObjects > > list_of_gt_objects_combination;
    std::vector<std::pair<SimulatedObjects, SimulatedObjects > > list_of_simulated_objects_combination;

    getCombination(list_of_gt_objects_combination, list_of_simulated_objects_combination);

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the trajectory in a png file

    std::map<std::string, double> time_map = {{"generate",     0},
            {"generate_flow", 0}};

    auto tic = steady_clock::now();
    auto toc = steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();

    char frame_skip_folder_suffix[50];
    cv::FileStorage fs;


    for ( ushort i = 0; i < list_of_simulated_objects_combination.size(); i ++ ) {
        std::cout << "collision between object id " << list_of_simulated_objects_combination.at(i).first.getObjectId() <<
                " and object id "
                << list_of_simulated_objects_combination.at(i).second.getObjectId()<< "\n";
    }

    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

        sprintf(frame_skip_folder_suffix, "%02d", frame_skip);

        std::cout << "generating collision points in OpticalFlow.cpp for " << m_resultordner << " " << frame_skip << std::endl;

        std::vector<std::vector<cv::Point2f> >  m_frame_collision_points;

        unsigned FRAME_COUNT = (unsigned)m_list_simulated_objects.at(0)
                .get_obj_extrapolated_pixel_centroid_pixel_displacement_mean().at
                (frame_skip - 1).size() - 1; // we store the flow image here and hence it starts at 1. Correspondingly the size reduces.
        assert(FRAME_COUNT>0);

        for (ushort frame_count = 1; frame_count < FRAME_COUNT; frame_count++) {
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

            for (unsigned i = 0; i < m_list_simulated_objects.size(); i++) {

                // object image_data_and_shape
                int width = m_list_simulated_objects.at(i).getWidth();
                int height = m_list_simulated_objects.at(i).getHeight();

                if ( m_list_simulated_objects.at(i).get_obj_extrapolated_visibility().at(frame_skip - 1).at(frame_count)
                        == true ) {

                    cv::Point2f centroid = m_list_simulated_objects.at(i).
                            get_obj_extrapolated_pixel_centroid_pixel_displacement_mean().at(frame_skip - 1)
                            .at(frame_count).first;
                    cv::Point2f mean_displacement = m_list_simulated_objects.at(i).
                            get_obj_extrapolated_pixel_centroid_pixel_displacement_mean().at(frame_skip- 1)
                            .at(frame_count).second;

                    cv::Point2f gt_line_pts = m_list_simulated_objects.at(i).get_line_parameters().at(frame_skip - 1)
                            .at(frame_count-1).second;  //line parameters run one less than the others.

                    cv::Mat roi;
                    roi = tempMatrix.
                            colRange(cvRound(centroid.x), cvRound(centroid.x + width)).
                            rowRange(cvRound(centroid.y), cvRound(centroid.y + height));
                    //bulk storage
                    roi = cv::Scalar(mean_displacement.x, mean_displacement.y,
                            static_cast<float>(m_list_simulated_objects.at(i).getObjectId()));

                    // cv line is intelligent and it can also project to values not within the frame size including negative values.
                    cv::line(tempMatrix, centroid, gt_line_pts, cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0);
                }
            }

            std::vector<cv::Point2f> collision_points;
            std::vector<cv::Point2f> collision_points_average;
            for ( unsigned i = 0; i < list_of_simulated_objects_combination.size(); i++) {

                if ( ( list_of_simulated_objects_combination.at(i).first.get_obj_extrapolated_visibility().at(frame_skip
                        - 1)
                        .at(frame_count) == true ) && ( list_of_simulated_objects_combination.at(i).second.
                        get_obj_extrapolated_visibility()
                        .at(frame_skip - 1)
                        .at(frame_count) == true )) {

                    // First Freeze lineparamter1 and look for collision points
                    // Then freeze lineparameter2 and find collision point.
                    // Then push_back the two points in the vector

                    for ( auto j = 0; j < 1; j++ ) {

                        cv::Point2f lineparameters1 = list_of_simulated_objects_combination.at(i).first.get_line_parameters().at
                                (frame_skip - 1)
                                .at(frame_count-1).first;

                        cv::Point2f lineparameters2 = list_of_gt_objects_combination.at(i).second.get_line_parameters
                                ().at(frame_skip - 1)
                                .at(frame_count-1).first;

                        std::cout << "object " << list_of_simulated_objects_combination.at(i).first.getObjectId() << " = " <<
                                lineparameters1 << " and object " << list_of_gt_objects_combination.at(i)
                                .second.getObjectId() << " = " <<lineparameters2 << std::endl ;

                        OpticalFlow::find_collision_points_given_two_line_parameters(lineparameters1, lineparameters2, tempMatrix, collision_points);

                        lineparameters1 = list_of_simulated_objects_combination.at(i).second.get_line_parameters().at
                                (frame_skip - 1)
                                .at(frame_count-1).first;

                        lineparameters2 = list_of_gt_objects_combination.at(i).first.get_line_parameters
                                ().at(frame_skip - 1)
                                .at(frame_count-1).first;

                        std::cout << "object " << list_of_simulated_objects_combination.at(i).second.getObjectId() << " = " <<
                                lineparameters1 << " and object " << list_of_gt_objects_combination.at(i)
                                .first.getObjectId() << " = " <<lineparameters2 << std::endl ;

                        find_collision_points_given_two_line_parameters(lineparameters1, lineparameters2, tempMatrix, collision_points);

                    }

                }
            }


            for ( auto i = 0; i < collision_points.size(); i=i+2 ) {
                if ( collision_points.at(i) != cv::Point2f(-1,-1) && collision_points.at(i+1) != cv::Point2f(-1,-1)) {
                    collision_points_average.push_back(cv::Point2f(((collision_points.at(i).x + collision_points.at(i+1).x ) / 2),
                            ((collision_points.at(i).y + collision_points.at(i+1).y ) / 2)));
                }
            }

            m_frame_collision_points.push_back(collision_points_average);

            //Create png Matrix with 3 channels: x mean_displacement. y displacment and ObjectId
            for (int32_t row = 0; row < Dataset::getFrameSize().height; row++) { // rows
                for (int32_t column = 0; column < Dataset::getFrameSize().width; column++) {  // cols
                    if (tempMatrix.at<cv::Vec3f>(row, column)[2] > 0.5 ) {
                        F_png_write.setFlowU(column, row, tempMatrix.at<cv::Vec3f>(row, column)[1]);
                        F_png_write.setFlowV(column, row, tempMatrix.at<cv::Vec3f>(row, column)[0]);
                        F_png_write.setObjectId(column, row, tempMatrix.at<cv::Vec3f>(row, column)[2]);
                        //trajectory.store_in_yaml(fs, cv::Point2f(row, column), cv::Point2f(xValue, yValue) );
                    }
                }
            }

            F_png_write.writeExtended(temp_collision_image_path);

        }
        m_frame_skip_collision_points.push_back(m_frame_collision_points);
    }

    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
    toc_all = steady_clock::now();
    time_map["generate_flow"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << m_resultordner + " flow generation time - " << time_map["generate_flow"] << "ms" << std::endl;
}


