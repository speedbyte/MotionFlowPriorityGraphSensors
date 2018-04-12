//
// Created by veikas on 21.02.18.
//

#include "ObjectMetaData.h"
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>
#include <iostream>


#include "kitti/log_colormap.h"
#include "datasets.h"


void Achterbahn::process(cv::Size frame_size) {
    std::vector<ushort> theta;
    for ( ushort frame_count = 0; frame_count < MAX_ITERATION_THETA; frame_count+=1) {
        theta.push_back(frame_count);
    }
    // Prepare points
    cv::Point2f l_pixel_position;
    for ( int i = 0; i< MAX_ITERATION_THETA; i++) {

        l_pixel_position.x = static_cast<float>((frame_size.width/2) + (100 * cos(theta[i] *CV_PI / 180.0) /
                                                                        (1.0 + std::pow(sin(theta[i] * CV_PI / 180.0), 2))));

        l_pixel_position.y = static_cast<float>((frame_size.height/2) + (55 * (cos(theta[i] * CV_PI / 180.0) *
                                                                               sin(theta[i] * CV_PI / 180.0)) /
                                                                         (0.2 +std::pow(sin(theta[i] * CV_PI / 180.0),2))));

        m_gt_all.at(i).m_object_location_px.location_x_m  = (l_pixel_position.x);
        m_gt_all.at(i).m_object_location_px.location_y_m  = (l_pixel_position.y);
        m_visibility.at(i) = true;
        m_gt_all.at(i).m_object_dimensions_px.dim_width_m = 30;
        m_gt_all.at(i).m_object_dimensions_px.dim_height_m = 70;

    }
}

void Circle::process(cv::Size frame_size) {

    std::vector<ushort> theta;
    for ( ushort frame_count = 0; frame_count < MAX_ITERATION_THETA; frame_count++) {
        theta.push_back(frame_count);
    }
    // Prepare points
    cv::Point2f l_pixel_position;
    for ( int i = 0; i< MAX_ITERATION_THETA; i++) {

        l_pixel_position.x = static_cast<float>( frame_size.width/2 + 100 * cos(theta[i]));

        l_pixel_position.y = static_cast<float>( frame_size.height/2 + 100 * sin(theta[i]));

        m_gt_all.at(i).m_object_location_px.location_x_m  = (l_pixel_position.x);
        m_gt_all.at(i).m_object_location_px.location_y_m  = (l_pixel_position.y);
        m_visibility.at(i) = true;
    }
}

void Ramp::process(cv::Size frame_size) {

    std::vector<ushort> theta;
    for ( ushort frame_count = 0; frame_count < MAX_ITERATION_THETA; frame_count++) {
        theta.push_back(frame_count);
    }
    // Prepare points
    cv::Point2f l_pixel_position;
    for ( int i = 0; i< MAX_ITERATION_THETA; i++) {

        //l_pixel_position.x = static_cast<float>((frame_size.width/2) + 10 * cos(theta[i]));

        //l_pixel_position.y = static_cast<float>((frame_size.height/2) + 10 * sin(theta[i]));

        l_pixel_position.x = static_cast<float>(0 + (theta[i]));

        l_pixel_position.y = static_cast<float>(0 + (theta[i]));

        m_gt_all.at(i).m_object_location_px.location_x_m  = (l_pixel_position.x);
        m_gt_all.at(i).m_object_location_px.location_y_m  = (l_pixel_position.y);
        m_visibility.at(i) = true;
    }
}

void NegativeRamp::process(cv::Size frame_size) {

    std::vector<ushort> theta;
    for ( ushort frame_count = 0; frame_count < MAX_ITERATION_THETA; frame_count++) {
        theta.push_back(frame_count);
    }
    // Prepare points
    cv::Point2f l_pixel_position;
    for ( int i = 0; i< MAX_ITERATION_THETA; i++) {

        //l_pixel_position.x = static_cast<float>((frame_size.width/2) + 10 * cos(theta[i]));

        //l_pixel_position.y = static_cast<float>((frame_size.height/2) + 10 * sin(theta[i]));

        l_pixel_position.x = static_cast<float>(120 - (theta[i]));

        l_pixel_position.y = static_cast<float>(0  + (theta[i]));

        m_gt_all.at(i).m_object_location_px.location_x_m  = (l_pixel_position.x);
        m_gt_all.at(i).m_object_location_px.location_y_m  = (l_pixel_position.y);
        m_visibility.at(i) = true;
    }
}

void NoPosition::process(cv::Size frame_size) {

};

#if 0
void ObjectMetaData::calcBBFrom3DPosition(int screen_width, int screen_height, cv::Point3d cam_pos, float fov_v, float pixSize = 2.2e-6){

    std::vector<cv::Point3f> bounding_points_3d;
    //all 8 3d bounding box points of an object
    //VTD center of object with z = 0!
    //dimension of real world
    
    bounding_points_3d.push_back(cv::Point3d( m_gt_all.at(0).m_object_realworld_dim_m.dim_length_m/2,  m_gt_all.at(0).m_object_realworld_dim_m.dim_width_m/2, 0));
    bounding_points_3d.push_back(cv::Point3d( -m_gt_all.at(0).m_object_realworld_dim_m.dim_length_m/2,  m_gt_all.at(0).m_object_realworld_dim_m.dim_width_m/2, 0));
    bounding_points_3d.push_back(cv::Point3d(  m_gt_all.at(0).m_object_realworld_dim_m.dim_length_m/2, -m_gt_all.at(0).m_object_realworld_dim_m.dim_width_m/2, 0));
    bounding_points_3d.push_back(cv::Point3d( -m_gt_all.at(0).m_object_realworld_dim_m.dim_length_m/2, -m_gt_all.at(0).m_object_realworld_dim_m.dim_width_m/2, 0));
    bounding_points_3d.push_back(cv::Point3d(  m_gt_all.at(0).m_object_realworld_dim_m.dim_length_m/2,  m_gt_all.at(0).m_object_realworld_dim_m.dim_width_m/2, m_gt_all.at(0).m_object_realworld_dim_m.dim_height_m));
    bounding_points_3d.push_back(cv::Point3d( -m_gt_all.at(0).m_object_realworld_dim_m.dim_length_m/2,  m_gt_all.at(0).m_object_realworld_dim_m.dim_width_m/2, m_gt_all.at(0).m_object_realworld_dim_m.dim_height_m));
    bounding_points_3d.push_back(cv::Point3d(  m_gt_all.at(0).m_object_realworld_dim_m.dim_length_m/2, -m_gt_all.at(0).m_object_realworld_dim_m.dim_width_m/2, m_gt_all.at(0).m_object_realworld_dim_m.dim_height_m));
    bounding_points_3d.push_back(cv::Point3d( -m_gt_all.at(0).m_object_realworld_dim_m.dim_length_m/2, -m_gt_all.at(0).m_object_realworld_dim_m.dim_width_m/2, m_gt_all.at(0).m_object_realworld_dim_m.dim_height_m));

    std::vector<cv::Point2d> bounding_points_2d;

    float width = screen_width, height = screen_height;
    float fovv = fov_v / 180. * M_PI; // [rad]
    float distToImagePlane = 0.5 * height / tan(fovv/2); // [px]
    float pxSize = pixSize; // [m/px]
    cv::Point3d toMeter = cv::Point3d(pxSize, pxSize, 1);

    cv::Point2d min(2000,2000), max(0,0);
    //transformation matrix to transform to camera location
    QMatrix4x4 toCamPos;
    toCamPos.translate(-cam_pos); //translate camera pos


    //iterate over bounding points and add transformed points to path to print
    for(cv::Point3d p: bounding_points_3d){


        //calculate correct bounding box point by adding offset to reference point (for VTD  = read middle axle of the car)
        p+=getDimensionOffset();  // get dimension offset

        //transformation matrix for offset to center of roi and the 3d bounding box point p
        QMatrix4x4 toPosition;
        QQuaternion rot = getRealWorldOrientation().getRotation();  // hpr
        toPosition.translate(rot * (p));

        cv::Point3d objPositionRelativeToPerfectOrigin = cv::Point3d(m_gt_all.at(0).m_object_location_inertial_m.location_x_m, m_gt_all.at(0).m_object_location_inertial_m.location_y_m, m_gt_all.at(0).m_object_location_inertial_m.location_z_m);  // get real world position from perfect sensor

        //first translate to camera pos
        objPositionRelativetoCamera = toCamPos * objPositionRelativeToPerfectOrigin;

        //then translate and rotate to 3d bounding box point
        boundingBoxPositionRelativetoCamera = toPosition * objPositionRelativetoCamera;

        //transform from sensor coordinates to camera coordinates
        pos = cv::Point3d(boundingBoxPositionRelativetoCamera.y(), boundingBoxPositionRelativetoCamera.z(), boundingBoxPositionRelativetoCamera.x());

        //scale 3D point back onto image
        pos = pos * ((distToImagePlane * pxSize) /*m*/ / pos.z());
        //convert meter to pixel
        pos = pos / toMeter;
        bounding_points_2d.push_back(cv::Point2d(width/2 - pos.x(), height/2 - pos.y()));
    }
    //get min and max for x, y to get the correct 2d bounding box
    for(QPoint p : bounding_points_2d){
        if(p.x() < min.x()){
            min.setX(p.x());
        }
        if(p.y() < min.y()){
            min.setY(p.y());
        }
        if(p.x() > max.x()){
            max.setX(p.x());
        }
        if(p.y() > max.y()){
            max.setY(p.y());
        }
    }
    m_bb.setTopLeft(min);
    m_bb.setBottomRight(max);
}

#endif


