//
// Created by veikas on 06.02.18.
//

#include <map>
#include <chrono>
#include "OpticalFlow.h"
#include "FlowImageExtended.h"

using namespace std::chrono;

void OpticalFlow::generate_collision_points(std::vector<Objects* > & m_list_objects) {

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the trajectory in a png file

    std::string m_resultordner = "";
    std::map<std::string, double> time_map = {{"generate",     0},
                                              {"ground truth", 0}};

    auto tic = steady_clock::now();
    auto toc = steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();

    std::cout << "collision graph will be srored in " << Dataset::getGroundTruthFlowPath().string() << std::endl;
    char folder_name_flow[50];
    cv::FileStorage fs;

    std::vector<Objects*>::const_iterator objectIterator = m_list_objects.begin();
    std::vector<Objects*>::const_iterator  objectIteratorNext;

    for ( ; objectIterator < m_list_objects.end() ; objectIterator++ ) {
        for ( objectIteratorNext = objectIterator+1; objectIteratorNext < m_list_objects.end();
              objectIteratorNext++) {

            m_list_objects_combination.push_back(std::make_pair(((*objectIterator)),
                                                                ((*objectIteratorNext))));
        }
    }

    for ( ushort i = 0; i < m_list_objects_combination.size(); i ++ ) {
        std::cout << "collision between object id " << m_list_objects_combination.at(i).first->getObjectId() <<
                  " and object id "
                 << m_list_objects_combination.at(i).second->getObjectId()<< "\n";
    }


    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

        sprintf(folder_name_flow, "flow_occ_%02d", frame_skip);
        fs.open(Dataset::getGroundTruthFlowPath().string() + "/" + m_resultordner + "/" + folder_name_flow + "/" + "gt_flow.yaml",
                cv::FileStorage::WRITE);

        sprintf(folder_name_flow, "flow_obj_%02d", frame_skip);
        std::cout << "generating collision points in GroundTruthFlow.cpp " << frame_skip << std::endl;

        unsigned FRAME_COUNT = (unsigned)(unsigned)m_list_objects.at(0)
                ->get_obj_extrapolated_pixel_centroid_pixel_displacement_mean().at
                (frame_skip - 1).size();
        assert(FRAME_COUNT>0);

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
            char file_name_image[50];
            std::cout << "frame_count " << frame_count << std::endl;

            sprintf(file_name_image, "000%03d_10.png", frame_count);
            std::string temp_gt_flow_image_path =
                    Dataset::getGroundTruthFlowPath().string() + "/" + m_resultordner + "/" + folder_name_flow + "/" +
                    file_name_image;

            fs << "frame_count" << frame_count;

            FlowImageExtended F_png_write(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

            cv::Mat tempMatrix;
            tempMatrix.create(Dataset::getFrameSize(), CV_32FC3);
            tempMatrix = cv::Scalar_<unsigned>(255,255,255);
            assert(tempMatrix.channels() == 3);

            for (unsigned i = 0; i < m_list_objects.size(); i++) {

                // object image_data_and_shape
                int width = m_list_objects.at(i)->getWidth();
                int height = m_list_objects.at(i)->getHeight();

                if ( m_list_objects.at(i)->get_obj_extrapolated_visibility().at(frame_skip - 1).at(frame_count)
                     == true ) {
                    // gt_displacement
                    cv::Point2f next_pts = m_list_objects.at(i)->
                            get_obj_extrapolated_pixel_centroid_pixel_displacement_mean().at(frame_skip - 1)
                            .at(frame_count).first;
                    cv::Point2f displacement = m_list_objects.at(i)->
                            get_obj_extrapolated_pixel_centroid_pixel_displacement_mean().at(frame_skip- 1)
                            .at(frame_count).second;

                    cv::Point2f gt_line_pts = m_list_objects.at(i)->get_line_parameters().at(frame_skip - 1)
                            .at(frame_count).second;


                    cv::Mat roi;
                    roi = tempMatrix.
                            colRange(cvRound(next_pts.x), cvRound(next_pts.x + width)).
                            rowRange(cvRound(next_pts.y), cvRound(next_pts.y + height));
                    //bulk storage
                    roi = cv::Scalar(displacement.x, displacement.y,
                                     static_cast<float>(m_list_objects.at(i)->getObjectId()));

                    // cv line is intelligent and it can also project to values not within the frame size including negative values.
                    cv::line(tempMatrix, next_pts, gt_line_pts, cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0);
                }
            }

            std::vector<cv::Point2f> collision_points;

            for ( unsigned i = 0; i < m_list_objects_combination.size(); i++) {

                if ( ( m_list_objects_combination.at(i).first->get_obj_extrapolated_visibility().at(frame_skip
                                                                                                          - 1)
                               .at(frame_count) == true ) && ( m_list_objects_combination.at(i).second->
                                                                       get_obj_extrapolated_visibility()
                                                                       .at(frame_skip - 1)
                                                                       .at(frame_count) == true )) {

                    cv::Point2f lineparameters1 = m_list_objects_combination.at(i).first->get_line_parameters().at
                                    (frame_skip - 1)
                            .at(frame_count).first;

                    cv::Point2f lineparameters2 = m_list_objects_combination.at(i).second->get_line_parameters
                                    ().at(frame_skip - 1)
                            .at(frame_count).first;

                    // first fill rowco
                    cv::Matx<float,2,2> coefficients (-lineparameters1.x,1,-lineparameters2.x,1);
                    cv::Matx<float,2,1> rhs(lineparameters1.y,lineparameters2.y);

                    std::cout << "object 1  = " << lineparameters1 << " and object 2 = " << lineparameters2 << std::endl ;

                    cv::Matx<float,2,1> result_manual;
                    if ( cv::determinant(coefficients ) != 0 ) {
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
                        //collision_points.push_back(cv::Point2f(result_manual(0,0), result_manual(1,0)));
                    }
                }
            }

            m_frame_collision_points.push_back(collision_points);

            //Create png Matrix with 3 channels: x displacement. y displacment and ObjectId
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

            F_png_write.writeExtended(temp_gt_flow_image_path);

        }
        m_frame_skip_collision_points.push_back(m_frame_collision_points);
    }

    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
    toc_all = steady_clock::now();
    time_map["ground truth"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth flow generation time - " << time_map["ground truth"] << "ms" << std::endl;
}
