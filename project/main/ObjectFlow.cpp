//
// Created by veikas on 28.01.18.
//

#include "ObjectFlow.h"

#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>


#include "kitti/log_colormap.h"
#include <kitti/io_flow.h>
#include "ObjectTrajectory.h"
#include "datasets.h"

void ObjectFlow::generate_base_flow_vector(std::vector<ObjectProperties> list_objects) {

    //Initialization
    std::vector<ushort> current_index;

    for ( int i = 0; i < list_objects.size(); i++ ) {
        current_index.push_back(list_objects.at(i).getStartPoint());
        std::cout << "start point "<< current_index.at(i) << std::endl;
    }

    std::cout << "ground truth flow will be stored in " << m_dataset.getInputPath().string() << std::endl;

    for ( int i = 0; i < list_objects.size(); i++ ) {

        std::vector<std::pair<cv::Point2i, cv::Point2i> > m_object_flowvector_with_coordinate_gt;
        for (ushort frame_count=0; frame_count < MAX_ITERATION_GT; frame_count++) {
        // The first frame is the reference frame, hence it is skipped
            std::vector<cv::Point2i> trajectory_points = list_objects.at(i).getTrajectoryPoints().get();
            cv::Mat shape = list_objects.at(i).getShape().get();

            if ( frame_count > 0 ) {

                cv::Point2i l_pixel_position= {0,0}, l_pixel_movement = {0,0};

                //If we are at the end of the path vector, we need to reset our iterators
                if ((current_index.at(i)) >= trajectory_points.size()) {
                    current_index.at(i) = 0;
                    l_pixel_movement.x = trajectory_points.at(current_index.at(i)).x - trajectory_points.at
                            (trajectory_points.size() - 1).x;
                    l_pixel_movement.y = trajectory_points.at(current_index.at(i)).y - trajectory_points.at
                            (trajectory_points.size() - 1).y;
                    l_pixel_position = trajectory_points.at(current_index.at(i));
                } else {
                    l_pixel_movement.x = trajectory_points.at(current_index.at(i)).x - trajectory_points.at
                            (current_index.at(i) - (ushort) 1).x;
                    l_pixel_movement.y = trajectory_points.at(current_index.at(i)).y - trajectory_points.at
                            (current_index.at(i) - (ushort) 1).y;
                    l_pixel_position = trajectory_points.at(current_index.at(i));
                }

                printf("%u, %u , %u, %u, %d, %d\n", frame_count, current_index.at(i), l_pixel_position.x,
                       l_pixel_position.y,
                       l_pixel_movement.x, l_pixel_movement.y);

                // make m_flowvector_with_coordinate_gt with smallest resolution.
                m_object_flowvector_with_coordinate_gt.push_back(std::make_pair(l_pixel_position, l_pixel_movement));
            }
            else {
                m_object_flowvector_with_coordinate_gt.push_back(std::make_pair(cv::Point2i(0,0), cv::Point2i(0,0)));
            }
            current_index.at(i)++;
        }
        m_scene_flow_vector_with_coordinate_gt.push_back(m_object_flowvector_with_coordinate_gt);
    }
}

/**
 *
 * @param pt - start from pt
 * @param width - extrapolate pt.y + width
 * @param height - extrapolate pt.x + height
 * @param xValue - what x value?
 * @param yValue - what y value?
 * @param image_path - where should the extrapolated image be stored?
 */

void ObjectFlow::extrapolate_flowpoints( FlowImage &F_gt_write, cv::FileStorage fs, cv::Point2i pt, int width, int
height, int xValue, int yValue) {

    cv::Mat tempMatrix;
    tempMatrix.create(m_dataset.getFrameSize(),CV_32FC3);
    assert(tempMatrix.channels() == 3);

    tempMatrix = cv::Scalar::all(0);
    cv::Mat roi;
    roi = tempMatrix.
            colRange(pt.x, (pt.x + width)).
            rowRange(pt.y, (pt.y + height));
    //bulk storage
    roi = cv::Scalar(xValue, yValue, 1.0f);

    // TODO take all the non 0 data in a float matrix and then call FlowImage Constructor with additional data
    // parameter
    //Create png Matrix with 3 channels: x displacement. y displacment and Validation bit
    for (int32_t row=0; row<m_dataset.getFrameSize().height; row++) { // rows
        for (int32_t column=0; column<m_dataset.getFrameSize().width; column++) {  // cols
            if (tempMatrix.at<cv::Vec3f>(row,column)[2] > 0.5 ) {
                F_gt_write.setFlowU(column,row,yValue);
                F_gt_write.setFlowV(column,row,xValue);
                F_gt_write.setValid(column,row,1.0f);
                //trajectory.store_in_yaml(fs, cv::Point2i(row, column), cv::Point2i(xValue, yValue) );
            }
        }
    }
}

void ObjectFlow::generate_extended_flow_vector(const int &max_skips, std::vector<ObjectProperties> list_objects) {

    char folder_name_flow[50];

    std::vector<int> temp_flow_x(list_objects.size(),0);
    std::vector<int> temp_flow_y(list_objects.size(),0);

    for ( int frame_skip = 1; frame_skip < max_skips ; frame_skip++ ) {

        sprintf(folder_name_flow, "flow_occ_%02d", frame_skip);
        cv::FileStorage fs;
        fs.open(m_dataset.getInputPath().string() + "/" + folder_name_flow + "/" + "gt_flow.yaml",
                cv::FileStorage::WRITE);
        std::cout << "creating flow files for frame_skip " << frame_skip << std::endl;

        for (ushort frame_count = 1; frame_count < MAX_ITERATION_GT; frame_count++) {

            /* What is the shape of the object ? */
            FlowImage F_gt_write(m_dataset.getFrameSize().width, m_dataset.getFrameSize().height);

            char file_name_image[50];
            sprintf(file_name_image, "000%03d_10.png", frame_count);
            std::string temp_gt_flow_image_path = m_dataset.getInputPath().string() + "/" + folder_name_flow + "/"
                                                  + file_name_image;

            for ( int i = 0; i < m_scene_flow_vector_with_coordinate_gt.size(); i++ ) {

                // The first frame is the reference frame.
                // The below code has to go through consecutive frames
                if (frame_count % frame_skip != 0) {
                    temp_flow_x[i] += m_scene_flow_vector_with_coordinate_gt.at(i).at(frame_count).second.x;
                    temp_flow_y[i] += m_scene_flow_vector_with_coordinate_gt.at(i).at(frame_count).second.y;
                }
                else {
                    temp_flow_x[i] += m_scene_flow_vector_with_coordinate_gt.at(i).at(frame_count).second.x;
                    temp_flow_y[i] += m_scene_flow_vector_with_coordinate_gt.at(i).at(frame_count).second.y;
                    fs << "frame_count" << frame_count;

                    extrapolate_flowpoints(F_gt_write, fs,
                                           cv::Point2i(m_scene_flow_vector_with_coordinate_gt.at(i).at(frame_count).first.x,
                                                       m_scene_flow_vector_with_coordinate_gt.at(i).at(frame_count).first.y),
                                           list_objects.at(i).getShape().get().cols, list_objects.at(i).getShape().get().rows,
                                           temp_flow_x[i], temp_flow_y[i]);

                    F_gt_write.write(temp_gt_flow_image_path);
                    temp_flow_x[i] = 0, temp_flow_y[i] = 0;
                }
            }
        }
        fs.release();
    }

}
