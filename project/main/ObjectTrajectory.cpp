//
// Created by veikas on 25.01.18.
//

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

void ObjectFlow::generate_base_flow_vector(std::vector<cv::Point2i> trajectory_points, const ushort start) {

    ushort current_index = 0;
    current_index = start;
    cv::Point2i l_pixel_position, l_pixel_movement;



    for (ushort frame_count=0; frame_count < MAX_ITERATION_GT; frame_count++) {
        // The first frame is the reference frame, hence it is skipped
        if ( frame_count > 0 ) {
            //If we are at the end of the path vector, we need to reset our iterators
            if ((current_index) >= trajectory_points.size()) {
                current_index = 0;
                l_pixel_movement.x = trajectory_points.at(current_index).x - trajectory_points.at(trajectory_points.size() - 1).x;
                l_pixel_movement.y = trajectory_points.at(current_index).y - trajectory_points.at(trajectory_points.size() - 1).y;
                l_pixel_position = trajectory_points.at(current_index);
            } else {
                l_pixel_movement.x = trajectory_points.at(current_index).x - trajectory_points.at(current_index - (ushort) 1).x;
                l_pixel_movement.y = trajectory_points.at(current_index).y - trajectory_points.at(current_index - (ushort) 1).y;
                l_pixel_position = trajectory_points.at(current_index);
            }

            printf("%u, %u , %u, %u, %u, %d, %d\n", frame_count, start, current_index, l_pixel_position.x, l_pixel_position.y,
                   l_pixel_movement.x, l_pixel_movement.y);

            // make m_flowvector_with_coordinate_gt with smallest resolution.
            m_flowvector_with_coordinate_gt.push_back(std::make_pair(l_pixel_position, l_pixel_movement));
        }
        else {
            m_flowvector_with_coordinate_gt.push_back(std::make_pair(cv::Point2i(0,0), cv::Point2i(0,0)));
        }
        current_index++;
    }
}

void ObjectTrajectory::storeData(const std::vector<cv::Point2f> &prev_pts, std::vector<cv::Point2f> &next_pts,
                                 const std::vector<uchar> status) {

    unsigned count = 0;

    for (unsigned i = 0; i < next_pts.size(); i++) {

        int minDist = 1;

        cv::Point2i l_pixel_position, l_pixel_movement;
        cv::Point2f algorithmMovement ((next_pts[i].x - prev_pts[i].x), (next_pts[i].y - prev_pts[i]
                .y));

        // Check if the status vector is good
        if (!status[i])
            continue;


        printf("flow_frame.at<cv::Point2f>(%f, %f).x =  %f\n", next_pts[i].x, next_pts[i].y,
               algorithmMovement.x);
        printf("flow_frame.at<cv::Point2f>(%f, %f).y =  %f\n", next_pts[i].x, next_pts[i].y,
               algorithmMovement.y);

        l_pixel_movement.x = cvRound( algorithmMovement.x + 0.5);
        l_pixel_movement.y = cvRound( algorithmMovement.y + 0.5);

        /* If the new point is within 'minDist' distance from an existing point, it will not be tracked */
        // auto dist = cv::norm(prev_pts[i] - next_pts[i]);
        double dist;
        dist = pow(l_pixel_movement.x,2)+pow(l_pixel_movement.y,2);
        //calculating distance by euclidean formula
        dist = sqrt(dist);

        if ( dist <= minDist ) {
            printf("minimum distance for %i is %f\n", i, dist);
            continue;
        }

        if ( l_pixel_movement.x == 0 && l_pixel_movement.y == 0) {
            continue;
        }

        next_pts[count++] = next_pts[i];

        // l_pixel_position is the new pixel position !
        l_pixel_position.x = std::abs(cvRound(next_pts[i].x));
        l_pixel_position.y = std::abs(cvRound(next_pts[i].y));

        printf("(iteration %u, coordinates x y (%i,%i) ->  Vx, Vy (%d,%d) \n", i,
               l_pixel_position.x, l_pixel_position.y, l_pixel_movement.x, l_pixel_movement.y);
        // Lines to indicate the motion vectors
        m_flow_matrix_result.push_back(std::make_pair(l_pixel_position, l_pixel_movement));
    }
    next_pts.resize(count);
}

void ObjectTrajectory::store_in_png(cv::FileStorage &fs, ushort &frame_count, cv::Size frame_size, std::string
temp_result_flow_path) {
    //Create png Matrix with 3 channels: x displacement. y displacment and Validation bit
    //kitti uses col, row specification
    FlowImage F_result_write(frame_size.width, frame_size.height);
    std::vector<std::pair<cv::Point2i, cv::Point2i> >::iterator it ;

    fs << "frame_count" << frame_count;

    for ( it = m_flow_matrix_result.begin(); it != m_flow_matrix_result.end(); it++ )
    {
        F_result_write.setFlowU((*it).first.x,(*it).first.y,(*it).second.x);
        F_result_write.setFlowV((*it).first.x,(*it).first.y,(*it).second.y);
        F_result_write.setValid((*it).first.x,(*it).first.y,(bool)1.0f);
        store_in_yaml(fs, (*it).first, (*it).second ); // coordinate - > movement y(row),x(col) ; x,y
    }
    F_result_write.write(temp_result_flow_path);
}

void ObjectTrajectory::store_in_yaml(cv::FileStorage &fs, const cv::Point2i &l_pixelposition, const cv::Point2i
    &l_pixelmovement )  {

    fs << "gt flow png file read" << "[";
    fs << "{:" << "row" <<  l_pixelposition.y << "col" << l_pixelposition.x << "displacement" << "[:";
    fs << l_pixelmovement.x;
    fs << l_pixelmovement.y;
    fs << 1;
    fs << "]" << "}";
    fs << "]";
}

void Achterbahn::process(cv::Size frame_size) {
    std::vector<ushort> theta;
    for ( ushort frame_count = 0; frame_count < MAX_ITERATION_THETA; frame_count++) {
        theta.push_back(frame_count);
    }
    // Prepare points
    cv::Point2i l_pixel_position;
    for ( int i = 0; i< MAX_ITERATION_THETA; i++) {

        l_pixel_position.x = (static_cast<ushort>((frame_size.width/2) + (500 * cos(theta[i] *CV_PI / 180.0) /
                                                                            (1.0 + std::pow(sin(theta[i] * CV_PI / 180.0), 2)))));

        l_pixel_position.y = (static_cast<ushort>((frame_size.height/2) + (55 * (cos(theta[i] * CV_PI / 180.0) *
                                                                                   sin(theta[i] * CV_PI / 180.0)) / (0.2 +std::pow(sin(theta[i] * CV_PI / 180.0),2)))));

        m_trajectory.push_back(l_pixel_position);
    }
}

void Rectangle::process() {


    m_shape.create(object_height, object_width, CV_8UC3);

    uchar r = 0;
    uchar b = 0;

    r = 0;
    b = 0;
    for (int k = 0; k < (object_height - 1); k++) {
        for (int j = 0; j < (object_width -1 ); j++) {
            m_shape.at<cv::Vec3b>(k, j)[0] = b;
            m_shape.at<cv::Vec3b>(k, j)[1] = 0;
            m_shape.at<cv::Vec3b>(k, j)[2] = r;
            r = r + (uchar)2;
            b = b + (uchar)2;
            if (r > 254)
                r = 130;
        }
        if (b > 254)
            b = 46;
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

void ObjectFlow::extrapolate_flowpoints( cv::FileStorage fs, cv::Point2i pt, ushort width, ushort height, int xValue,
                                         int yValue, std::string image_path) {

    cv::Mat tempMatrix;
    //ObjectTrajectory trajectory;
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
    FlowImage F_gt_write(m_dataset.getFrameSize().width, m_dataset.getFrameSize().height);
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
    F_gt_write.write(image_path);

}

void ObjectFlow::generate_extended_flow_vector(const int &max_skips) {

    char folder_name_flow[50];
    char file_name_image[50];

    int temp_flow_x = 0, temp_flow_y = 0 ;

    for ( int frame_skip = 1; frame_skip < max_skips ; frame_skip++ ){
        sprintf(folder_name_flow, "flow_occ_%02d", frame_skip);
        cv::FileStorage fs;
        fs.open(m_dataset.getInputPath().string() + "/" + folder_name_flow + "/" + "gt_flow.yaml",
                cv::FileStorage::WRITE);
        std::cout << "creating flow files for frame_skip " << frame_skip << std::endl;
        for (ushort frame_count=1; frame_count < MAX_ITERATION_GT; frame_count++) {
            // The first frame is the reference frame.
            //the below code has to go through consecutive frames
            if ( frame_count%frame_skip != 0 ) {
                temp_flow_x += m_flowvector_with_coordinate_gt.at(frame_count).second.x;
                temp_flow_y += m_flowvector_with_coordinate_gt.at(frame_count).second.y;
                continue;
            }
            temp_flow_x += m_flowvector_with_coordinate_gt.at(frame_count).second.x;
            temp_flow_y += m_flowvector_with_coordinate_gt.at(frame_count).second.y;
            fs << "frame_count" << frame_count;
            sprintf(file_name_image, "000%03d_10.png", frame_count);
            std::string temp_gt_flow_image_path = m_dataset.getInputPath().string() + "/" + folder_name_flow + "/"
                                                  + file_name_image;
            extrapolate_flowpoints( fs, cv::Point2i(m_flowvector_with_coordinate_gt.at(frame_count).first.x,
                                                    m_flowvector_with_coordinate_gt.at
                                                            (frame_count).first.y),
                                    object_width, object_height, temp_flow_x, temp_flow_y, temp_gt_flow_image_path );
            temp_flow_x = 0, temp_flow_y = 0 ;
        }
        fs.release();
    }

}
