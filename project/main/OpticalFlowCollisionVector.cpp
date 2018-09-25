#include "OpticalFlow.h"

void OpticalFlow::generate_collision_points() {

    std::vector<std::pair<Objects*, Objects* > > list_of_current_objects_combination;
    std::vector<std::pair<Objects*, Objects* > > list_of_gt_objects_combination;
    std::vector<std::pair<Objects*, Objects* > > list_of_simulated_objects_combination;

    std::vector<Objects *> ptr_list_of_copied_gt_objects;
    std::vector<Objects *> ptr_list_of_copied_simulated_objects;

    for ( auto i = 0; i < m_ptr_list_gt_objects.size(); i++) {
        ptr_list_of_copied_gt_objects.push_back(static_cast<Objects*>(m_ptr_list_gt_objects.at(i)));
    }
    getCombination(ptr_list_of_copied_gt_objects, list_of_gt_objects_combination);

    unsigned COUNT;
    if ( m_opticalFlowName == "ground_truth") {
        COUNT = 1;
        list_of_current_objects_combination = list_of_gt_objects_combination;
    }
    else {

        for ( auto i = 0; i < get_simulated_objects_ptr_list().size(); i++) {
            ptr_list_of_copied_simulated_objects.push_back(static_cast<Objects*>(get_simulated_objects_ptr_list().at(i)));
        }
        COUNT = (unsigned)get_simulated_objects_ptr_list().at(0)->get_list_object_line_parameters().size();
        getCombination(ptr_list_of_copied_simulated_objects, list_of_simulated_objects_combination);
        list_of_current_objects_combination = list_of_simulated_objects_combination;
    }

    for ( ushort obj_combination_index = 0; obj_combination_index < list_of_current_objects_combination.size(); obj_combination_index++ ) {
        std::cout << "collision between object name " << list_of_current_objects_combination.at(obj_combination_index).first->getObjectName() <<
                  " and object name "
                  << list_of_current_objects_combination.at(obj_combination_index).second->getObjectName()<< "\n";
    }

    for ( unsigned datafilter_index = 0; datafilter_index < COUNT; datafilter_index++ ) {

        std::vector<std::vector<std::vector<OPTICAL_FLOW_COLLISION_METRICS> > > sensor_collision_points;
        std::vector<std::vector<std::vector<cv::Point2f> > > sensor_line_angles;

        for (unsigned sensor_index = 0; sensor_index < Dataset::SENSOR_COUNT; sensor_index++) {

            std::vector<std::vector<OPTICAL_FLOW_COLLISION_METRICS> > sensor_frame_collision_points;
            std::vector<std::vector<cv::Point2f> > sensor_frame_line_angles;

            std::cout << "generating collision points in OpticalFlow.cpp for " << m_resultordner << " " << sensor_index
                      << " for datafilter " << datafilter_index << std::endl;

            unsigned FRAME_COUNT = (unsigned) m_ptr_list_gt_objects.at(0)
                    ->get_list_object_line_parameters().at(0).at(sensor_index).size();

            assert(FRAME_COUNT > 0);

            for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

                std::cout << "current_frame_index " << current_frame_index << " for datafilter_index " << datafilter_index<< std::endl;

                ushort image_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                        (0).at(current_frame_index).frame_no;

                std::vector<cv::Point2f> frame_collision_points;
                cv::Point2f frame_collision_points_average;
                std::vector<cv::Point2f> frame_line_angles;
                std::vector<OPTICAL_FLOW_COLLISION_METRICS> packet_frame_collision_points_analysis;


                char file_name_image[50];
                ushort vires_frame_count = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                        (0).at(current_frame_index).frame_no;
                sprintf(file_name_image, "000%03d_10.png", vires_frame_count);

                for (unsigned obj_combination_index = 0;
                     obj_combination_index < list_of_current_objects_combination.size(); obj_combination_index++) {

                    if ((list_of_current_objects_combination.at(
                            obj_combination_index).first->get_object_extrapolated_visibility().at(sensor_index).at(current_frame_index)) &&
                        (list_of_current_objects_combination.at(obj_combination_index).second->get_object_extrapolated_visibility()
                                .at(sensor_index).at(current_frame_index))) {

                        // First Freeze lineparamter1 and look for collision points
                        // Then freeze lineparameter2 and find collision point.
                        // Then push_back the two points in the vector

                        cv::Point2f lineparameters1 = list_of_current_objects_combination.at(
                                obj_combination_index).first->get_list_object_line_parameters().at(datafilter_index).at
                                (sensor_index).at(current_frame_index);

                        cv::Point2f temp_line_parameters1 = lineparameters1;

                        cv::Point2f lineparameters2 = list_of_gt_objects_combination.at(
                                obj_combination_index).second->get_list_object_line_parameters().at(0).at(sensor_index).at(current_frame_index);

                        std::cout << "object "
                                  << list_of_current_objects_combination.at(obj_combination_index).first->getObjectId()
                                  << " = " <<
                                  lineparameters1 << " and object " << list_of_gt_objects_combination.at(obj_combination_index)
                                          .second->getObjectId() << " = " << lineparameters2 << std::endl;

                        find_collision_points_given_two_line_parameters(lineparameters1, lineparameters2, frame_collision_points);

                        lineparameters1 = list_of_current_objects_combination.at(
                                obj_combination_index).second->get_list_object_line_parameters().at(datafilter_index).at
                                (sensor_index).at(current_frame_index);

                        lineparameters2 = list_of_gt_objects_combination.at(obj_combination_index).first->get_list_object_line_parameters
                                ().at(0).at(sensor_index).at(current_frame_index);

                        std::cout << "object "
                                  << list_of_current_objects_combination.at(obj_combination_index).second->getObjectId()
                                  << " = " <<
                                  lineparameters1 << " and object " << list_of_gt_objects_combination.at(obj_combination_index)
                                          .first->getObjectId() << " = " << lineparameters2 << std::endl;

                        find_collision_points_given_two_line_parameters(lineparameters1, lineparameters2,
                                                                        frame_collision_points);

                        if ( current_frame_index > 0 ) {
                            assert(temp_line_parameters1!=lineparameters1);
                        }

                        frame_line_angles.push_back(cv::Point2f(temp_line_parameters1.x,lineparameters1.x));


                    } else {

                        frame_collision_points.push_back(cv::Point2f(-1, -1));
                        frame_collision_points.push_back(cv::Point2f(-1, -1));
                        std::cout << "object "
                                  << list_of_current_objects_combination.at(obj_combination_index).first->getObjectId()
                                  << " visibility = " <<
                                  list_of_current_objects_combination.at(
                                          obj_combination_index).first->get_object_extrapolated_visibility().at(
                                          sensor_index).at(current_frame_index) << " and object "
                                  << list_of_gt_objects_combination.at(obj_combination_index)
                                          .second->getObjectId() << " visibility = "
                                  << list_of_current_objects_combination.at(
                                          obj_combination_index).second->get_object_extrapolated_visibility().at(
                                          sensor_index).at(current_frame_index)
                                  << " and hence not generating any collision points for this object combination "
                                  << std::endl;
                    }
                }

                // Average between the collision points of two objects ( Partial collision ).
                // For ground truth it is simply summing two equal points and dividing by two.
                // TODO - this is broken.. need to fix the averaging.
                for (auto i = 0; i < frame_collision_points.size(); i = i + 2) {
                    if (frame_collision_points.at(i) != cv::Point2f(-1, -1) &&
                        frame_collision_points.at(i + 1) != cv::Point2f(-1, -1)) {
                        frame_collision_points_average = cv::Point2f(
                                ((frame_collision_points.at(i).x + frame_collision_points.at(i + 1).x) / 2),
                                ((frame_collision_points.at(i).y + frame_collision_points.at(i + 1).y) /
                                 2));
                    }
                    else {
                        frame_collision_points_average = cv::Point2f(std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity());
                    }
                }

                packet_frame_collision_points_analysis.push_back({image_frame_count, frame_collision_points_average});

                sensor_frame_collision_points.push_back(packet_frame_collision_points_analysis);
                sensor_frame_line_angles.push_back(frame_line_angles);
            }

            sensor_collision_points.push_back(sensor_frame_collision_points);
            sensor_line_angles.push_back(sensor_frame_line_angles);

        }
        m_list_sensor_collision_points.push_back(sensor_collision_points);
        m_list_sensor_line_angles.push_back(sensor_line_angles);
    }

    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
    std::cout << m_resultordner + " collision generation done"  << std::endl;
}

void OpticalFlow::find_collision_points_given_two_line_parameters(const cv::Point2f lineparameters1, const cv::Point2f lineparameters2,
                                                                  std::vector<cv::Point2f> &frame_collision_points) {
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

