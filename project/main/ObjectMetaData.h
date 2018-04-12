//
// Created by veikas on 21.02.18.
//

#ifndef MAIN_OBJECTMETADATA_H
#define MAIN_OBJECTMETADATA_H

#include <opencv2/core/utility.hpp>
#include "Dataset.h"
#include <iostream>
#include "datasets.h"
#include "ObjectImageShapeData.h"

/*
%YAML:1.0
iterationNr: 100
strings:
   - "image1.jpg"
   - Awesomeness
   - "baboon.jpg"
Mapping:
   One: 1
   Two: 2
R: !!opencv-matrix
   rows: 3
   cols: 3
   dt: u
   data: [ 1, 0, 0, 0, 1, 0, 0, 0, 1 ]
T: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [ 0., 0., 0. ]
MyData:
   A: 97
   X: 3.1415926535897931e+000
   id: mydata1234
*/



class STRUCT_GT_ALL {

public:

    //Each line contains one object annotation with the following columns:
    //frame: frame index in the video (starts from 0)
    ushort frame_no;

    //tid: track identification number (unique for each object instance)
    ushort tid;

    //label: KITTI-like name of the 'type' of the object (Car, Van, DontCare)
    std::string label;

    //truncated: (changed name in v1.3) KITTI-like truncation flag (0: not truncated, 1: truncated, 2: heavily truncated, marked as “DontCare”)
    bool truncated;

    //occluded: (changed name in v1.3) KITTI-like occlusion flag  (0: not occluded, 1; occluded, 2: heavily occluded, marked as “DontCare”)
    bool occluded;

    //alpha: KITTI-like observation angle of the object in [-pi..pi]
    float alpha_rad;

    //l, t, r, b: KITTI-like 2D 'bbox', respectively left, top, right, bottom bounding box in pixel coordinates (inclusive, (0,0) origin is on the upper left corner of the image)
    struct bounding_box_m { float bb_left_px; float bb_top_px; float bb_right_px; float bb_bottom_px;} m_bounding_box;

    //w3d, h3d, l3d: KITTI-like 3D object 'dimensions', respectively width, height, length in meters
    struct object_dimensions_px { float dim_width_m; float dim_height_m; float dim_length_m; } m_object_dimensions_px;

    //w3d, h3d, l3d: KITTI-like 3D object 'dimensions', respectively width, height, length in meters
    struct object_realworld_dim_m { float dim_width_m; float dim_height_m; float dim_length_m; } m_object_realworld_dim_m;

    //x3d, y3d, z3d: KITTI-like 3D object 'location', respectively x, y, z in camera coordinates in meters
    struct object_location_px { float location_x_m; float location_y_m; float location_z_m;} m_object_location_px;

    //x3d, y3d, z3d: KITTI-like 3D object 'location', respectively x, y, z in camera coordinates in meters
    struct object_location_m { float location_x_m; float location_y_m; float location_z_m;} m_object_location_m;

    struct object_location_inertial_m { float location_x_m; float location_y_m; float location_z_m;} m_object_location_inertial_m;

    //(center of bottom face of 3D bounding box)
    //ry: KITTI-like 3D object 'rotation_y', rotation around Y-axis (yaw) in camera coordinates [-pi..pi]
    //(KITTI convention is ry == 0 iff object is aligned with x-axis and pointing right)
    //rx: rotation around X-axis (pitch) in camera coordinates [-pi..pi]
    //rz: rotation around Z-axis (roll) in camera coordinates [-pi..pi]
    struct object_rotation_rad { float rotation_ry_yaw_rad; float rotation_rx_pitch_rad; float rotation_rz_roll_rad; } m_object_rotation_rad;
    struct object_rotation_inertial_rad { float rotation_ry_yaw_rad; float rotation_rx_pitch_rad; float rotation_rz_roll_rad; } m_object_rotation_inertial_rad;

    //truncr: (changed in v1.3) object 2D truncation ratio in [0..1] (0: no truncation, 1: entirely truncated)
    bool truncr;

    //occupr: object 2D occupancy ratio (fraction of non-occluded pixels) in [0..1] (0: fully occluded, 1: fully visible, independent of truncation)
    bool occupr;

    //orig_label: original KITTI-like name of the 'type' of the object ignoring the 'DontCare' rules (allows to know original type of DontCare objects)
    std::string orig_label;

    //moving: 0/1 flag to indicate whether the object is really moving between this frame and the next one
    bool moving;

    //model: the name of the 3D model used to render the object (can be used for fine-grained recognition)
    std::string cad_3d_model;

    //color: the name of the color of the object
    std::string color;

    //offset. where is the position determined on the object ( center of gravity )
    struct object_offset_m { float offset_x; float offset_y; float offset_z; } m_object_offset;

    //speed of the object
    struct object_speed_m { float x; float y; float z; } m_object_speed;
    struct object_speed_inertial_m { float x; float y; float z; } m_object_speed_inertial;

    /*
     Remarks about 3D information

    Internally, the 3D world is projected on the screen by using the Unity Engine rendering pipeline and various shaders. You can reproduce this by projecting points from the camera space (e.g., coordinates x3d,y3d,z3d) to the image pixels by using our camera intrinsic matrix (in pixels, constant, computed from our 1242x375 resolution and 29° fov):

          [[725,    0, 620.5],
    K =  [   0, 725, 187.0],
            [   0,     0,       1]]

    In our system of 3D camera coordinates x is going to the right, y is going down, and z is going forward (the origin is the optical center of the camera).*
     */

    /*
     * Camera pose (extrinsic parameters): link (1.1MB)

    The 3D camera pose (rotation and translation) for each frame of a video consists of one CSV-like text files named:

    vkitti_<version>_extrinsicsgt/<world>_<variation>.txt

    Each file can be loaded with the following one-liner in Python using the popular pandas library (assuming ‘import pandas as pd’):

    extgt = pd.read_csv("<filename>", sep=" ", index_col=False)

    Each line consists of the frame index in the video (starts from 0) followed by the row-wise flattened 4x4 extrinsic matrix at that frame:

    r1,1 r1,2 r1,3 t1
    M = r2,1 r2,2 r2,3 t2
    r3,1 r3,2 r3,3 t3
    0     0     0    1

    where ri,j are the coefficients of the camera rotation matrix R and ti are the coefficients of the camera translation coefficients t.

    This matrix can be used to convert points from world space to the camera space. For a point p = (x,y,z) in the world space, P = (x,y,z,1) in the homogeneous coordinates, you can get the coordinates in the camera space by doing the dot product MP.

    See section above for the camera intrinsic parameters and description of our camera coordinate system.
     */


    /*
    The MOT ground truth for each video consists of a CSV-like text file named:

    vkitti_<version>_motgt/<world>_<variation>.txt

    These files are in a KITTI-like format that can be loaded with the following one-liner in python using the popular pandas library (assuming ‘import pandas as pd’):

     motgt = pd.read_csv("<filename>", sep=" ", index_col=False)

    */

    // class for extracting meta data and supplying to Objects at runtime
    // Each dataset describes the way it extracts the metadata. I will try to keep it very generalized ie. follow the
    // kitti concept.

    // this also includes calibration data

    // for example[KITTI_RAW_CALIBRATION]
    // PATH = "../../../datasets/kitti_raw_dataset/data/"
    // EXECUTE = 0

};

class ObjectMetaData {

protected:
    std::vector<STRUCT_GT_ALL> m_gt_all;
    std::vector<bool> m_visibility;

private:
    ObjectImageShapeData m_objectMetaData_shape;
    std::string m_objectMetaData_name;
    ushort m_objectMetaData_startPoint;


public:

    ObjectMetaData() {
        for (ushort i = 0; i < MAX_ITERATION_THETA; i++) {
            m_visibility.push_back(false);
            STRUCT_GT_ALL s = {};
            m_gt_all.push_back(s);
        }
    };

    ObjectMetaData(ObjectImageShapeData shape, std::string name, ushort startPoint) :
            m_objectMetaData_shape(shape), m_objectMetaData_name(name), m_objectMetaData_startPoint(startPoint) {
        for (ushort i = 0; i < MAX_ITERATION_THETA; i++) {
            m_visibility.push_back(false);
            STRUCT_GT_ALL s = {};
            m_gt_all.push_back(s);
        }
    } ;

    void fillData() {

    }

    ObjectImageShapeData& getObjectShape() {
        return m_objectMetaData_shape;
    }

    std::string& getObjectName() {
        return m_objectMetaData_name;
    }

    ushort& getObjectStartPoint() {
        return m_objectMetaData_startPoint;
    }

    void setObjectName(std::string objectName) {
        m_objectMetaData_name = objectName;
    }

    void setObjectShape(ObjectImageShapeData objectShape) {
        m_objectMetaData_shape = objectShape;
    }

    virtual void process(cv::Size frame_size) {};

    virtual void pushPositionPoints(cv::Point2f points) {}

    virtual void pushVisibility(bool visibility) {}

    void atFrameNumberCameraSensor(ushort frameNumber, cv::Point3f position, cv::Point2f offset, cv::Point2f dimensions) {
        //m_pixel_position.at(frameNumber) = position;
        m_gt_all.at(frameNumber).m_object_location_px.location_x_m = position.x;
        m_gt_all.at(frameNumber).m_object_location_px.location_y_m = position.y;
        m_gt_all.at(frameNumber).m_object_location_px.location_z_m = position.z;

        m_gt_all.at(frameNumber).m_object_offset.offset_y = offset.y;
        m_gt_all.at(frameNumber).m_object_offset.offset_x = offset.x;

        m_gt_all.at(frameNumber).m_object_dimensions_px.dim_width_m = dimensions.x;
        m_gt_all.at(frameNumber).m_object_dimensions_px.dim_height_m = dimensions.y;
    }

    void atFrameNumberPerfectSensor(ushort frameNumber, cv::Point3f position, cv::Point3f orientation, cv::Point3f dimensions, cv::Point2f speed) {
        m_gt_all.at(frameNumber).m_object_location_m.location_x_m = position.x;
        m_gt_all.at(frameNumber).m_object_location_m.location_y_m = position.y;
        m_gt_all.at(frameNumber).m_object_location_m.location_z_m = position.z;

        m_gt_all.at(frameNumber).m_object_rotation_rad.rotation_ry_yaw_rad = orientation.x;
        m_gt_all.at(frameNumber).m_object_rotation_rad.rotation_rx_pitch_rad = orientation.y;
        m_gt_all.at(frameNumber).m_object_rotation_rad.rotation_rz_roll_rad = orientation.y;

        m_gt_all.at(frameNumber).m_object_realworld_dim_m.dim_width_m = dimensions.x;
        m_gt_all.at(frameNumber).m_object_realworld_dim_m.dim_height_m = dimensions.y;
        m_gt_all.at(frameNumber).m_object_realworld_dim_m.dim_length_m = dimensions.z;

        m_gt_all.at(frameNumber).m_object_speed.x = speed.x;
        m_gt_all.at(frameNumber).m_object_speed.y = speed.y;

    }

    void atFrameNumberPerfectSensorInertial(ushort frameNumber, cv::Point3f position, cv::Point3f orientation, cv::Point3f dimensions, cv::Point2f speed) {
        m_gt_all.at(frameNumber).m_object_location_inertial_m.location_x_m = position.x;
        m_gt_all.at(frameNumber).m_object_location_inertial_m.location_y_m = position.y;
        m_gt_all.at(frameNumber).m_object_location_inertial_m.location_z_m = position.z;

        m_gt_all.at(frameNumber).m_object_rotation_inertial_rad.rotation_ry_yaw_rad = orientation.x;
        m_gt_all.at(frameNumber).m_object_rotation_inertial_rad.rotation_rx_pitch_rad = orientation.y;
        m_gt_all.at(frameNumber).m_object_rotation_inertial_rad.rotation_rz_roll_rad = orientation.z;

        m_gt_all.at(frameNumber).m_object_realworld_dim_m.dim_width_m = dimensions.x;
        m_gt_all.at(frameNumber).m_object_realworld_dim_m.dim_height_m = dimensions.y;
        m_gt_all.at(frameNumber).m_object_realworld_dim_m.dim_length_m = dimensions.z;

        m_gt_all.at(frameNumber).m_object_speed_inertial.x = speed.x;
        m_gt_all.at(frameNumber).m_object_speed_inertial.y = speed.y;

    }

    void atFrameNumberVisibility(ushort frameNumber, bool visibility) {
        m_visibility.at(frameNumber) = visibility;
    }

    std::vector<STRUCT_GT_ALL> getAll() const {
        return m_gt_all;
    }

    std::vector<bool> getVisibility() const {
        return m_visibility;
    }

    void calcBBFrom3DPosition(int screen_width, int screen_height, cv::Point3d cam_pos, float fov_v, float pixSize = 2.2e-6);

    void setDynamic() {

        cv::RNG rng(cv::getTickCount());
        for (ushort i = 0; i < MAX_ITERATION_THETA / 20; i++) {
            ushort index = (ushort) rng.uniform(0, 360);
            m_visibility.at(index) = false;
            //TODO - set pixel position
        }
        for ( auto t : m_visibility ) {
            std::cout << t;
        }
        std::cout << std::endl;

        for (ushort i = 0; i < MAX_ITERATION_THETA; i++) {

            if (i < (m_visibility.size() - 1) && m_visibility.at(i + (ushort) 1) == false) {
                m_visibility.at(i) = false;
            }
        }

        for ( auto t : m_visibility ) {
            std::cout << t;
        }

        std::cout << std::endl;
        for (ushort i = MAX_ITERATION_THETA; i > 0; i--) {

            if (i < (m_visibility.size() - 1) && m_visibility.at(i - (ushort) 1) == false) {
                m_visibility.at(i) = false;
            }
        }

        for ( auto t : m_visibility ) {
            std::cout << t;
        }
        std::cout << std::endl;
    }

};


class Achterbahn : public ObjectMetaData {

public:

    Achterbahn() {};
    Achterbahn(ObjectImageShapeData shape, std::string name, ushort startPoint) : ObjectMetaData(shape, name, startPoint) {} ;

    void process(cv::Size frame_size) override ;

};

class Circle : public ObjectMetaData {

public:

    Circle(ObjectImageShapeData shape, std::string name, ushort startPoint) : ObjectMetaData(shape, name, startPoint) {} ;

    void process(cv::Size frame_size) override ;

};

class Ramp : public ObjectMetaData {

public:

    Ramp(ObjectImageShapeData shape, std::string name, ushort startPoint) : ObjectMetaData(shape, name, startPoint) {} ;

    void process(cv::Size frame_size) override ;

};

class NegativeRamp : public ObjectMetaData {

public:

    NegativeRamp(ObjectImageShapeData shape, std::string name, ushort startPoint) : ObjectMetaData(shape, name, startPoint) {} ;

    void process(cv::Size frame_size) override ;

};

class NoPosition: public ObjectMetaData {

public:

    NoPosition(ObjectImageShapeData shape, std::string name, ushort startPoint) : ObjectMetaData(shape, name, startPoint) {} ;

    void process(cv::Size frame_size) override;

};




#endif //MAIN_OBJECTMETADATA_H
