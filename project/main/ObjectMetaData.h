//
// Created by veikas on 21.02.18.
//

#ifndef MAIN_OBJECTMETADATA_H
#define MAIN_OBJECTMETADATA_H

#include <opencv2/core/utility.hpp>
#include "Dataset.h"
#include <iostream>
#include <vires-interface/Common/viRDBIcd.h>
#include "datasets.h"
#include <opencv2/core/mat.hpp>
#include <bits/unique_ptr.h>
#include "Noise.h"


class ObjectImageShapeData {

protected:

    cv::Mat m_data_image;
    cv::Mat m_data_depth;

    ushort m_objectWidth;
    ushort m_objectHeight;
    float m_depth;

public:

    ObjectImageShapeData() {};

    ObjectImageShapeData(ushort width, ushort height, std::unique_ptr<Noise> &noise, ushort depth ) : m_objectWidth(width), m_objectHeight
            (height), m_depth(depth) {
    }

    virtual void process() {};

    void applyNoise(std::unique_ptr<Noise> &noise) {
        noise->apply(m_data_image);
    }

    void applyDepth(ushort depth) {
        m_data_depth.setTo(depth);
    }

    cv::Mat getImage() {
        return m_data_image;
    }

    cv::Mat getDepthImage() {
        return m_data_depth;
    }

    float getObjectDepth() {
        return m_depth;
    }

    ushort getObjectWidth() {
        return m_objectWidth;
    }

    ushort getObjectHeight() {
        return m_objectHeight;
    }
};

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

typedef struct object_location_inertial_m { float location_x_m; float location_y_m; float location_z_m;} object_location_inertial_m_str;

//w3d, h3d, l3d: KITTI-like 3D object 'dimensions', respectively width, height, length in meters
typedef struct object_dimensions_px { float width_px; float height_px; float dim_length_m; } object_dimensions_px_str;

//w3d, h3d, l3d: KITTI-like 3D object 'dimensions', respectively width, height, length in meters
typedef struct object_realworld_dim_m { float dim_width_m; float dim_height_m; float dim_length_m; } object_realworld_dim_m_str;

typedef struct object_rotation_inertial_rad { float rotation_rx_roll_rad; float rotation_ry_pitch_rad; float rotation_rz_yaw_rad; } object_rotation_inertial_rad_str;

//offset. Shift of point from the center of mass
typedef struct object_offset_m { float offset_x; float offset_y; float offset_z; } object_offset_m_str;

//x3d, y3d, z3d: KITTI-like 3D object 'location', respectively x, y, z in camera coordinates in meters
typedef struct object_location_m { float location_x_m; float location_y_m; float location_z_m;} object_location_m_str;

//x3d, y3d, z3d: KITTI-like 3D object 'location', respectively x, y, z in camera coordinates in meters
typedef struct region_of_interest_px { float x; float y; float width_px; float height_px;} region_of_interest_px_str;

//x3d, y3d, z3d: KITTI-like 3D object 'location', respectively x, y, z in camera coordinates in meters
typedef struct object_location_px { float location_x_px; float location_y_px; float location_z_px; cv::Point2f cog_px;} object_location_px_str;

//x3d, y3d, z3d: KITTI-like 3D object 'location', respectively x, y, z in camera coordinates in meters
typedef struct object_occlusion { signed char occlusion_px; signed char occlusion_usk; signed char occlusion_inertial; } object_occlusion_str;

class STRUCT_GT_OBJECTS_ALL {

public:

    //Each line contains one object annotation with the following columns:
    //frame: frame index in the video (starts from 0)
    ushort frame_no;

    std::string object_name;

    //tid: track identification number (unique for each object instance)
    ushort tid;

    //label: KITTI-like name of the 'type' of the object (Car, Van, DontCare)
    std::string label;

    //truncated: (changed name in v1.3) KITTI-like truncation flag (0: not truncated, 1: truncated, 2: heavily truncated, marked as “DontCare”)
    bool truncated;

    //occluded: (changed name in v1.3) KITTI-like occlusion flag  (0: not occluded, 1; occluded, 2: heavily occluded, marked as “DontCare”)
    ushort occluded;

    //occluded: (changed name in v1.3) KITTI-like occlusion flag  (0: not occluded, 1; occluded, 2: heavily occluded, marked as “DontCare”)
    ushort visMask;


    //alpha: KITTI-like observation angle of the object in [-pi..pi]
    float alpha_rad;

    //l, t, r, b: KITTI-like 2D 'bbox', respectively left, top, right, bottom bounding box in pixel coordinates (inclusive, (0,0) origin is on the upper left corner of the image)
    struct bounding_box_m { 
        cv::Point2f bb_lower_bottom_px; 
        cv::Point2f bb_lower_right_px; 
        cv::Point2f bb_lower_top_px; 
        cv::Point2f bb_lower_left_px;
        cv::Point2f bb_higher_bottom_px;
        cv::Point2f bb_higher_right_px;
        cv::Point2f bb_higher_top_px;
        cv::Point2f bb_higher_left_px;
    } m_bounding_box;

    object_dimensions_px_str m_object_dimension_camera_px;

    object_realworld_dim_m_str m_object_dimension_realworld_m;

    object_location_inertial_m_str m_object_location_inertial_m;

    object_location_m_str m_object_location_usk_m;

    object_location_px_str m_object_location_camera_px;

    object_occlusion_str m_object_occlusion;

    //(center of bottom face of 3D bounding box)
    //ry: KITTI-like 3D object 'rotation_y', rotation around Y-axis (yaw) in camera coordinates [-pi..pi]
    //(KITTI convention is ry == 0 iff object is aligned with x-axis and pointing right)
    //rx: rotation around X-axis (pitch) in camera coordinates [-pi..pi]
    //rz: rotation around Z-axis (roll) in camera coordinates [-pi..pi]
    struct object_rotation_rad { float rotation_rx_roll_rad; float rotation_ry_pitch_rad; float rotation_rz_yaw_rad;} m_object_rotation_rad;

    object_rotation_inertial_rad_str m_object_rotation_inertial_rad;

    region_of_interest_px_str m_region_of_interest_px;

    struct object_distances { float sensor_to_obj_px; float sensor_to_obj_usk; float total_distance_covered; } m_object_distances;

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

    object_offset_m_str m_object_offset_m;

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
    // meta data
    std::vector<STRUCT_GT_OBJECTS_ALL> m_object_gt_all;
    std::vector<RDB_OBJECT_STATE_t> m_object_state_all;
    ushort m_objectMetaData_startPoint;

private:
    // shape data
    ObjectImageShapeData m_objectMetaData_shape;
    //ObjectTrajectory m_objectTrajectory;

public:

    ObjectMetaData() {
        for (ushort i = 0; i < MAX_ITERATION_THETA; i++) {
            STRUCT_GT_OBJECTS_ALL s = {};
            m_object_gt_all.push_back(s);
        }
    };

    ObjectMetaData(ObjectImageShapeData shape, std::string name, ushort startPoint) :
            m_objectMetaData_shape(shape), m_objectMetaData_startPoint(startPoint) {
        for (ushort i = 0; i < MAX_ITERATION_THETA; i++) {
            STRUCT_GT_OBJECTS_ALL s = {};
            m_object_gt_all.push_back(s);
            m_object_gt_all.at(i).object_name = name;
        }
    } ;

    ObjectMetaData(std::string name, ushort startPoint) : m_objectMetaData_startPoint(startPoint) {
        for (ushort i = 0; i < MAX_ITERATION_THETA; i++) {
            STRUCT_GT_OBJECTS_ALL s = {};
            m_object_gt_all.push_back(s);
            m_object_gt_all.at(i).object_name = name;
        }
    } ;

    void fillData() {

    }

    ObjectImageShapeData& getObjectShape() {
        return m_objectMetaData_shape;
    }

    std::string& getObjectName() {
        return m_object_gt_all.at(0).object_name;
    }

    ushort& getObjectStartPoint() {
        return m_objectMetaData_startPoint;
    }

    void setObjectName(std::string objectName) {
        for (ushort i = 0; i < MAX_ITERATION_THETA; i++) {
            m_object_gt_all.at(i).object_name = objectName;
        }
    }

    void setObjectShape(ObjectImageShapeData objectShape) {
        m_objectMetaData_shape = objectShape;
    }

    void setStartPoint(ushort startPoint) {
        m_objectMetaData_startPoint = startPoint;
    }

    virtual void process(cv::Size frame_size)  {};

    virtual void pushPositionPoints(cv::Point2f points) {}

    virtual void pushVisibility(bool visibility) {}

    void atFrameNumberFrameCount(ushort frameNumber) {
        m_object_gt_all.at(frameNumber).frame_no = frameNumber;
    }

    void atAllObjectStateData(ushort frame_number, RDB_OBJECT_STATE_t* data) {
        // dereference and store data
        cv::Point3f offset, position_inertial, position_usk, position_pixel, dimension_realworld;
        cv::Point2f dimension_pixel;
        cv::Point2f speed_inertial, speed_usk;
        cv::Point3f orientation_usk, orientation_inertial;
        float dist_cam_to_obj;
        float total_distance_travelled;

        m_object_state_all.push_back(*data);

        if (data->base.pos.type == RDB_COORD_TYPE_WINDOW) {

            position_pixel = cv::Point3f((float) data->base.pos.x, (float) data->base.pos.y,
                                         float(data->base.pos.z));
            dimension_pixel = cv::Point2f((float) data->base.geo.dimX, (float) data->base.geo.dimY);
            offset = cv::Point3f((float) data->base.geo.offX, (float) data->base.geo.offY,
                                 (float) data->base.geo.offZ);
            total_distance_travelled = data->ext.traveledDist;


            atFrameNumberCameraSensor(
                    (ushort) frame_number, position_pixel, offset,
                    dimension_pixel, total_distance_travelled);
            atFrameNumberVisibility(
                    (ushort) frame_number, data->base.visMask);

        } else if (data->base.pos.type == RDB_COORD_TYPE_USK) {

            position_usk = cv::Point3f((float) data->base.pos.x, (float) data->base.pos.y,
                                       (float) data->base.pos.z);
            orientation_usk = cv::Point3f((float) data->base.pos.h, (float) data->base.pos.p,
                                          (float) data->base.pos.r);
            dimension_realworld = cv::Point3f((float) data->base.geo.dimX, (float) data->base.geo.dimY,
                                              (float) data->base.geo.dimZ);
            speed_usk = cv::Point2f((float) data->ext.speed.x, (float) data->ext.speed.y);
            total_distance_travelled = data->ext.traveledDist;
            atFrameNumberPerfectSensor(
                    (ushort) frame_number, position_usk,
                    orientation_usk, dimension_realworld, speed_usk, total_distance_travelled);
            atFrameNumberVisibility(
                    (ushort) frame_number, data->base.visMask);

        } else if (data->base.pos.type == RDB_COORD_TYPE_INERTIAL) {

            position_inertial = cv::Point3f((float) data->base.pos.x, (float) data->base.pos.y,
                                            (float) data->base.pos.z);
            orientation_inertial = cv::Point3f((float) data->base.pos.h, (float) data->base.pos.p,
                                               (float) data->base.pos.r);
            dimension_realworld = cv::Point3f((float) data->base.geo.dimX, (float) data->base.geo.dimY,
                                              (float) data->base.geo.dimZ);
            speed_inertial = cv::Point2f((float) data->ext.speed.x, (float) data->ext.speed.y);
            total_distance_travelled = data->ext.traveledDist;
            atFrameNumberPerfectSensorInertial(
                    (ushort) frame_number, position_inertial,
                    orientation_inertial, dimension_realworld, speed_inertial, total_distance_travelled);
            atFrameNumberVisibility(
                    (ushort) frame_number, data->base.visMask);
        }

    }

    void atFrameNumberCameraSensor(ushort frameNumber, cv::Point3f position, cv::Point3f offset, cv::Point2f dimensions, float total_distance_travelled) {
        //m_pixel_position.at(frameNumber) = position;
        m_object_gt_all.at(frameNumber).m_object_location_camera_px.location_x_px = position.x;
        m_object_gt_all.at(frameNumber).m_object_location_camera_px.location_y_px = position.y;
        m_object_gt_all.at(frameNumber).m_object_location_camera_px.location_z_px = position.z;

        m_object_gt_all.at(frameNumber).m_object_offset_m.offset_x = offset.x;
        m_object_gt_all.at(frameNumber).m_object_offset_m.offset_y = offset.y;
        m_object_gt_all.at(frameNumber).m_object_offset_m.offset_z = offset.z;

        m_object_gt_all.at(frameNumber).m_object_dimension_camera_px.width_px = dimensions.x;
        m_object_gt_all.at(frameNumber).m_object_dimension_camera_px.height_px = dimensions.y;

        m_object_gt_all.at(frameNumber).m_object_distances.total_distance_covered = total_distance_travelled;

    }

    void atFrameNumberPerfectSensor(ushort frameNumber, cv::Point3f position, cv::Point3f orientation, cv::Point3f dimensions, cv::Point2f speed, float total_distance_travelled) {
        m_object_gt_all.at(frameNumber).m_object_location_usk_m.location_x_m = position.x;
        m_object_gt_all.at(frameNumber).m_object_location_usk_m.location_y_m = position.y;
        m_object_gt_all.at(frameNumber).m_object_location_usk_m.location_z_m = position.z;

        m_object_gt_all.at(frameNumber).m_object_rotation_rad.rotation_rz_yaw_rad = orientation.x; //h
        m_object_gt_all.at(frameNumber).m_object_rotation_rad.rotation_ry_pitch_rad = orientation.y; //p
        m_object_gt_all.at(frameNumber).m_object_rotation_rad.rotation_rx_roll_rad = orientation.x; //r

        m_object_gt_all.at(frameNumber).m_object_dimension_realworld_m.dim_length_m = dimensions.x;
        m_object_gt_all.at(frameNumber).m_object_dimension_realworld_m.dim_width_m = dimensions.y;
        m_object_gt_all.at(frameNumber).m_object_dimension_realworld_m.dim_height_m = dimensions.z;

        m_object_gt_all.at(frameNumber).m_object_speed.x = speed.x;
        m_object_gt_all.at(frameNumber).m_object_speed.y = speed.y;

        m_object_gt_all.at(frameNumber).m_object_distances.total_distance_covered = total_distance_travelled;

    }

    void atFrameNumberPerfectSensorInertial(ushort frameNumber, cv::Point3f position, cv::Point3f orientation, cv::Point3f dimensions, cv::Point2f speed, float total_distance_travelled) {
        m_object_gt_all.at(frameNumber).m_object_location_inertial_m.location_x_m = position.x;
        m_object_gt_all.at(frameNumber).m_object_location_inertial_m.location_y_m = position.y;
        m_object_gt_all.at(frameNumber).m_object_location_inertial_m.location_z_m = position.z;

        m_object_gt_all.at(frameNumber).m_object_rotation_inertial_rad.rotation_rz_yaw_rad = orientation.x; //h
        m_object_gt_all.at(frameNumber).m_object_rotation_inertial_rad.rotation_ry_pitch_rad = orientation.y; //p
        m_object_gt_all.at(frameNumber).m_object_rotation_inertial_rad.rotation_rx_roll_rad = orientation.z; //r

        m_object_gt_all.at(frameNumber).m_object_dimension_realworld_m.dim_length_m = dimensions.x;
        m_object_gt_all.at(frameNumber).m_object_dimension_realworld_m.dim_width_m = dimensions.y;
        m_object_gt_all.at(frameNumber).m_object_dimension_realworld_m.dim_height_m = dimensions.z;

        m_object_gt_all.at(frameNumber).m_object_speed_inertial.x = speed.x;
        m_object_gt_all.at(frameNumber).m_object_speed_inertial.y = speed.y;

        m_object_gt_all.at(frameNumber).m_object_distances.total_distance_covered = total_distance_travelled;

    }

    void setCppData();

    void setBoundingBoxPoints(ushort frameNumber, std::vector<cv::Point2f> bbox_points);

    void atFrameNumberOcclusionWindow(ushort frameNumber, signed char occlusion, float distance_cam_to_object) {
        m_object_gt_all.at(frameNumber).m_object_occlusion.occlusion_px = occlusion;
        m_object_gt_all.at(frameNumber).m_object_distances.sensor_to_obj_px = distance_cam_to_object;
    }

    void atFrameNumberOcclusionUsk(ushort frameNumber, signed char occlusion, float distance_cam_to_object) {
        m_object_gt_all.at(frameNumber).m_object_occlusion.occlusion_usk= occlusion;
        m_object_gt_all.at(frameNumber).m_object_distances.sensor_to_obj_usk = distance_cam_to_object;
    }

    void atFrameNumberOcclusionInertial(ushort frameNumber, signed char occlusion) {
        m_object_gt_all.at(frameNumber).m_object_occlusion.occlusion_inertial = occlusion;
    }

    void atFrameNumberVisibility(ushort frameNumber, ushort visibility) {
        m_object_gt_all.at(frameNumber).visMask = visibility;
    }

    std::vector<STRUCT_GT_OBJECTS_ALL> getAll() const {
        return m_object_gt_all;
    }

    void setDynamic() {

        cv::RNG rng(cv::getTickCount());
        for (ushort i = 0; i < MAX_ITERATION_THETA / 20; i++) {
            ushort index = (ushort) rng.uniform(0, 360);
            m_object_gt_all.at(index).visMask = 0;
            //TODO - set pixel position
        }
        for ( auto t : m_object_gt_all) {
            std::cout << t.visMask;
        }
        std::cout << std::endl;

        for (ushort i = 0; i < MAX_ITERATION_THETA; i++) {

            if (i < (m_object_gt_all.size() - 1) && m_object_gt_all.at(i + (ushort) 1).visMask == 0) {
                m_object_gt_all.at(i).visMask = 0;
            }
        }

        for ( auto t : m_object_gt_all ) {
            std::cout << t.visMask;
        }

        std::cout << std::endl;
        for (ushort i = MAX_ITERATION_THETA; i > 0; i--) {

            if (i < (m_object_gt_all.size() - 1) && m_object_gt_all.at(i - (ushort) 1).visMask == 0) {
                m_object_gt_all.at(i).visMask = 0;
            }
        }

        for ( auto t : m_object_gt_all ) {
            std::cout << t.visMask;
        }
        std::cout << std::endl;
    }

};


class ObjectTrajectory : public ObjectMetaData {

public:

    ObjectTrajectory() {}

    ObjectTrajectory(ObjectImageShapeData shape, std::string name, ushort startPoint) :
            ObjectMetaData(name, startPoint) {}

    ObjectTrajectory(std::string name, ushort startPoint) :
            ObjectMetaData(name, startPoint) {}
};

class Achterbahn : public ObjectTrajectory {

public:

    Achterbahn() {};

    Achterbahn(std::string name, ushort startPoint) : ObjectTrajectory(name, startPoint) {} ;

    void process(cv::Size frame_size) override;

};

class CircleTrajectory : public ObjectTrajectory {

public:

    CircleTrajectory(ObjectImageShapeData shape, std::string name, ushort startPoint) : ObjectTrajectory(shape, name, startPoint) {} ;

    void process(cv::Size frame_size) override;

};

class Ramp : public ObjectTrajectory {

public:

    Ramp(std::string name, ushort startPoint) : ObjectTrajectory(name, startPoint) {} ;

    void process(cv::Size frame_size) override;

};

class NegativeRamp : public ObjectTrajectory {

public:

    NegativeRamp(std::string name, ushort startPoint) : ObjectTrajectory(name, startPoint) {} ;

    void process(cv::Size frame_size) override;

};

class NoPosition: public ObjectTrajectory {

public:

    NoPosition(std::string name, ushort startPoint) : ObjectTrajectory(name, startPoint) {} ;

    void process(cv::Size frame_size) override;

};


// Canvas is a kind of Object ( its just a big object, and hence has the same property )
class Canvas  : public ObjectImageShapeData {

private:


public:


    // The canvas can move to simulate a moving car. Hence we need position etc.
    Canvas( ushort width, ushort height, std::unique_ptr<Noise> &noise ) : ObjectImageShapeData(width, height, noise, 0) {
        m_data_image.create(height, width, CV_8UC3);
        applyNoise(noise);
    };

    void process() override ;


};


class Rectangle :  public ObjectImageShapeData {

private:

public:

    Rectangle(ushort width, ushort height, std::unique_ptr<Noise> &noise, ushort depth) : ObjectImageShapeData(width, height, noise, depth) {
        m_data_image.create(height, width, CV_8UC3);
        m_data_depth.create(height, width, CV_8UC1);
        applyNoise(noise);
        applyDepth(depth);
    };

    void process() override ;

};

class Circle :  public ObjectImageShapeData {

private:
    ushort m_objectRadius;

public:

    Circle(ushort radius, std::unique_ptr<Noise> &noise, ushort depth) : m_objectRadius(radius/2), ObjectImageShapeData(radius, radius, noise, depth) {
        construct(radius, noise, depth);
    };

    void construct(ushort radius, std::unique_ptr<Noise> &noise, ushort depth);

    void process() override ;

};




#endif //MAIN_OBJECTMETADATA_H
