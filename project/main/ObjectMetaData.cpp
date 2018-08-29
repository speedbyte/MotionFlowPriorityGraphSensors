//
// Created by veikas on 21.02.18.
//

#include "ObjectMetaData.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>
#include "datasets.h"


void Achterbahn::process(cv::Size frame_size) {

    // Prepare points
    cv::Point2f l_pixel_position;
    ushort current_frame_index = 0;
    for ( ushort i = m_objectMetaData_startPoint; i< (Dataset::MAX_ITERATION_DATASET + m_objectMetaData_startPoint); i++) {


        l_pixel_position.x = static_cast<float>((frame_size.width/2) + (100 * cos((i) *CV_PI / 180.0) /
                                                                        (1.0 + std::pow(sin((i) * CV_PI / 180.0), 2))));

        l_pixel_position.y = static_cast<float>((frame_size.height/2) + (55 * (cos((i) * CV_PI / 180.0) *
                                                                               sin((i) * CV_PI / 180.0)) /
                                                                         (0.2 +std::pow(sin((i) * CV_PI / 180.0),2))));

        m_object_gt_all.at(current_frame_index).occluded = 0;

        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.location_x_px = (l_pixel_position.x);
        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.location_y_px = (l_pixel_position.y);

        m_object_gt_all.at(current_frame_index).m_object_location_inertial_m.location_x_m = 1;
        m_object_gt_all.at(current_frame_index).visMask = 7;

        m_object_gt_all.at(current_frame_index).frame_no = current_frame_index;

        current_frame_index++;

    }
}

void CircleTrajectory::process(cv::Size frame_size) {

    // Prepare points
    cv::Point2f l_pixel_position;
    ushort current_frame_index = 0;
    for ( ushort i = m_objectMetaData_startPoint; i< (Dataset::ITERATION_END_POINT + m_objectMetaData_startPoint); i++) {

        l_pixel_position.x = static_cast<float>( frame_size.width/2 + 100 * cos((i)));

        l_pixel_position.y = static_cast<float>( frame_size.height/2 + 100 * sin((i)));

        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.location_x_px  = (l_pixel_position.x);
        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.location_y_px  = (l_pixel_position.y);

        m_object_gt_all.at(current_frame_index).occluded;
        current_frame_index++;
    }
}

void Ramp::process(cv::Size frame_size) {

    // Prepare points
    cv::Point2f l_pixel_position;
    ushort current_frame_index = 0;
    for ( ushort i = m_objectMetaData_startPoint; i< (Dataset::ITERATION_END_POINT + m_objectMetaData_startPoint); i++) {

        //l_pixel_position.x = static_cast<float>((frame_size.width/2) + 10 * cos(theta.at(i)));

        //l_pixel_position.y = static_cast<float>((frame_size.height/2) + 10 * sin(theta.at(i)));

        l_pixel_position.x = static_cast<float>(0 + ((i)));

        l_pixel_position.y = static_cast<float>(0 + ((i)));

        m_object_gt_all.at(current_frame_index).occluded = 0;

        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.location_x_px = (l_pixel_position.x);
        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.location_y_px = (l_pixel_position.y);

        m_object_gt_all.at(current_frame_index).m_object_location_inertial_m.location_x_m = 1;
        m_object_gt_all.at(current_frame_index).visMask = 7;

        m_object_gt_all.at(current_frame_index).frame_no = current_frame_index;

        current_frame_index++;

    }
}

void NegativeRamp::process(cv::Size frame_size) {

    // Prepare points
    cv::Point2f l_pixel_position;
    ushort current_frame_index = 0;
    for ( ushort i = m_objectMetaData_startPoint; i< (Dataset::ITERATION_END_POINT + m_objectMetaData_startPoint); i++) {

        //l_pixel_position.x = static_cast<float>((frame_size.width/2) + 10 * cos(theta.at(i)));

        //l_pixel_position.y = static_cast<float>((frame_size.height/2) + 10 * sin(theta.at(i)));

        l_pixel_position.x = static_cast<float>(frame_size.width - ((i)));

        l_pixel_position.y = static_cast<float>(0  + ((i)));

        m_object_gt_all.at(current_frame_index).occluded = 0;

        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.location_x_px = (l_pixel_position.x);
        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.location_y_px = (l_pixel_position.y);

        m_object_gt_all.at(current_frame_index).m_object_location_inertial_m.location_x_m = 1;
        m_object_gt_all.at(current_frame_index).visMask = 7;

        m_object_gt_all.at(current_frame_index).frame_no = current_frame_index;

        current_frame_index++;
    }
}

void NoPosition::process(cv::Size frame_size) {

};

void ObjectMetaData::setBoundingBoxPoints(ushort frameNumber, std::vector<cv::Point2f> bbox_points) {

    m_object_gt_all.at(frameNumber).m_bounding_box.bb_lower_bottom_px = bbox_points.at(7);
    m_object_gt_all.at(frameNumber).m_bounding_box.bb_lower_right_px = bbox_points.at(6);
    m_object_gt_all.at(frameNumber).m_bounding_box.bb_lower_top_px = bbox_points.at(4);
    m_object_gt_all.at(frameNumber).m_bounding_box.bb_lower_left_px = bbox_points.at(5);
    m_object_gt_all.at(frameNumber).m_bounding_box.bb_higher_bottom_px = bbox_points.at(3);
    m_object_gt_all.at(frameNumber).m_bounding_box.bb_higher_right_px = bbox_points.at(2);
    m_object_gt_all.at(frameNumber).m_bounding_box.bb_higher_top_px = bbox_points.at(0);
    m_object_gt_all.at(frameNumber).m_bounding_box.bb_higher_left_px = bbox_points.at(1);

    cv::Rect roi_2d = cv::boundingRect(bbox_points);
    std::cout << roi_2d << std::endl;

    if ( "Pedesterian" == m_object_gt_all.at(frameNumber).object_name ) {
        // hack
        m_object_gt_all.at(frameNumber).m_region_of_interest_px.x = roi_2d.x - 3;
        m_object_gt_all.at(frameNumber).m_region_of_interest_px.y = roi_2d.y - 13;
        m_object_gt_all.at(frameNumber).m_region_of_interest_px.width_px = roi_2d.width + 4;
        m_object_gt_all.at(frameNumber).m_region_of_interest_px.height_px = roi_2d.height + 13;
    } else {
        m_object_gt_all.at(frameNumber).m_region_of_interest_px.x = roi_2d.x;
        m_object_gt_all.at(frameNumber).m_region_of_interest_px.y = roi_2d.y;
        m_object_gt_all.at(frameNumber).m_region_of_interest_px.width_px = roi_2d.width;
        m_object_gt_all.at(frameNumber).m_region_of_interest_px.height_px = roi_2d.height;
    }

    m_object_gt_all.at(frameNumber).m_object_location_camera_px.cog_px = bbox_points.at(8);
}


void ObjectMetaData::setCppData() {

    unsigned long FRAME_COUNT = Dataset::MAX_ITERATION_DATASET;
    assert(FRAME_COUNT > 0);

    for ( ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

        m_object_gt_all.at(current_frame_index).m_object_dimension_camera_px.width_px = m_objectMetaData_shape.getObjectWidth();
        m_object_gt_all.at(current_frame_index).m_object_dimension_camera_px.height_px = m_objectMetaData_shape.getObjectHeight();
        m_object_gt_all.at(current_frame_index).m_object_distances.sensor_to_obj_usk = m_objectMetaData_shape.getObjectDepth();

        m_object_gt_all.at(current_frame_index).m_region_of_interest_px.x = m_object_gt_all.at(
                current_frame_index).m_object_location_camera_px.location_x_px;
        m_object_gt_all.at(current_frame_index).m_region_of_interest_px.y = m_object_gt_all.at(
                current_frame_index).m_object_location_camera_px.location_y_px;

        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.cog_px.x = (
                m_object_gt_all.at(current_frame_index).m_region_of_interest_px.x +
                m_object_gt_all.at(current_frame_index).m_object_dimension_camera_px.width_px / 2);
        m_object_gt_all.at(current_frame_index).m_object_location_camera_px.cog_px.y = (
                m_object_gt_all.at(current_frame_index).m_region_of_interest_px.y +
                m_object_gt_all.at(current_frame_index).m_object_dimension_camera_px.height_px / 2);

        m_object_gt_all.at(current_frame_index).m_region_of_interest_px.width_px = m_object_gt_all.at(
                current_frame_index).m_object_dimension_camera_px.width_px;
        m_object_gt_all.at(current_frame_index).m_region_of_interest_px.height_px = m_object_gt_all.at(
                current_frame_index).m_object_dimension_camera_px.height_px;
    }
}


void Rectangle::process() {

    //m_data_image.create(m_objectHeight, m_objectWidth, CV_8UC3);
    //m_data_depth.create(m_objectHeight, m_objectWidth, CV_8UC1);

}

void Canvas::process() {

    //m_data_image.create(m_objectHeight, m_objectWidth, CV_8UC3);

}

void Circle::construct(ushort radius, std::unique_ptr<Noise> &noise, ushort depth)  {

    m_data_image.create(radius, radius, CV_8UC3);
    m_data_image.setTo(cv::Scalar(255,255,255));
    m_data_depth.create(radius, radius, CV_8UC1);
    cv::circle(m_data_image, cv::Point(m_objectRadius, m_objectRadius), (m_objectRadius-5), cv::Scalar(255,0,0), CV_FILLED);
    applyNoise(noise);
    applyDepth(depth);

}

void Circle::process() {

    //m_data_image.create(m_objectHeight, m_objectWidth, CV_8UC3);
    //m_data_depth.create(m_objectHeight, m_objectWidth, CV_8UC1);
}




