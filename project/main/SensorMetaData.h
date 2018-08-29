//
// Created by veikas on 21.02.18.
//

#ifndef MAIN_SENSORMETADATA_H
#define MAIN_SENSORMETADATA_H

#include <opencv2/core/utility.hpp>
#include "Dataset.h"
#include <iostream>
#include "datasets.h"



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

typedef struct sensor_location_carrier_m { float location_x_m; float location_y_m; float location_z_m;} sensor_location_carrier_m_str;

typedef struct sensor_rotation_carrier_rad { float rotation_rx_roll_rad; float rotation_ry_pitch_rad; float rotation_rz_yaw_rad; } sensor_rotation_carrier_rad_str;

//offset. Shift of point from the center of mass
typedef struct sensor_offset_m { float offset_x; float offset_y; float offset_z; } sensor_offset_m_str;

typedef struct sensor_cam_info { float fov_horizontal_rad; float fov_vertical_rad; float near_m; float far_m; } sensor_cam_info_str;


class STRUCT_GT_SENSORS_ALL {

public:

    //Each line contains one sensor annotation with the following columns:
    //frame: frame index in the video (starts from 0)
    ushort frame_no;

    //tid: track identification number (unique for each sensor instance)
    ushort tid;

    //label: KITTI-like name of the 'type' of the sensor (Car, Van, DontCare)
    std::string label;

    //truncated: (changed name in v1.3) KITTI-like truncation flag (0: not truncated, 1: truncated, 2: heavily truncated, marked as “DontCare”)
    bool truncated;

    //occluded: (changed name in v1.3) KITTI-like occlusion flag  (0: not occluded, 1; occluded, 2: heavily occluded, marked as “DontCare”)
    bool occluded;

    //occluded: (changed name in v1.3) KITTI-like occlusion flag  (0: not occluded, 1; occluded, 2: heavily occluded, marked as “DontCare”)
    ushort visMask;

    //alpha: KITTI-like observation angle of the sensor in [-pi..pi]
    float alpha_rad;

    //l, t, r, b: KITTI-like 2D 'bbox', respectively left, top, right, bottom bounding box in pixel coordinates (inclusive, (0,0) origin is on the upper left corner of the image)
    struct bounding_box_m { float bb_left_px; float bb_top_px; float bb_right_px; float bb_bottom_px;} m_bounding_box;

    sensor_location_carrier_m_str m_sensor_location_carrier_m;

    //x3d, y3d, z3d: KITTI-like 3D sensor 'location', respectively x, y, z in camera coordinates in meters
    struct sensor_location_px { float location_x_m; float location_y_m; float location_z_m;} m_sensor_location_px;

    //x3d, y3d, z3d: KITTI-like 3D sensor 'location', respectively x, y, z in camera coordinates in meters
    struct sensor_location_m { float location_x_m; float location_y_m; float location_z_m;} m_sensor_location_m_str;

    //(center of bottom face of 3D bounding box)
    //ry: KITTI-like 3D sensor 'rotation_y', rotation around Y-axis (yaw) in camera coordinates [-pi..pi]
    //(KITTI convention is ry == 0 iff sensor is aligned with x-axis and pointing right)
    //rx: rotation around X-axis (pitch) in camera coordinates [-pi..pi]
    //rz: rotation around Z-axis (roll) in camera coordinates [-pi..pi]
    struct sensor_rotation_rad { float rotation_rx_roll_rad; float rotation_ry_pitch_rad; float rotation_rz_yaw_rad;} m_sensor_rotation_rad;

    sensor_rotation_carrier_rad_str m_sensor_rotation_carrier_rad;

    struct sensor_distances { float sensor_to_obj; float total_distance_covered; } m_sensor_distances;

    //truncr: (changed in v1.3) sensor 2D truncation ratio in [0..1] (0: no truncation, 1: entirely truncated)
    bool truncr;

    //occupr: sensor 2D occupancy ratio (fraction of non-occluded pixels) in [0..1] (0: fully occluded, 1: fully visible, independent of truncation)
    bool occupr;

    //orig_label: original KITTI-like name of the 'type' of the sensor ignoring the 'DontCare' rules (allows to know original type of DontCare sensors)
    std::string orig_label;

    //moving: 0/1 flag to indicate whether the sensor is really moving between this frame and the next one
    bool moving;

    //model: the name of the 3D model used to render the sensor (can be used for fine-grained recognition)
    std::string cad_3d_model;

    //color: the name of the color of the sensor
    std::string color;

    sensor_offset_m_str m_sensor_offset_m;

    sensor_cam_info_str m_sensor_cam_info;

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

    // class for extracting meta data and supplying to Sensors at runtime
    // Each dataset describes the way it extracts the metadata. I will try to keep it very generalized ie. follow the
    // kitti concept.

    // this also includes calibration data

    // for example[KITTI_RAW_CALIBRATION]
    // PATH = "../../../datasets/kitti_raw_dataset/data/"
    // EXECUTE = 0

};

class SensorMetaData {

protected:
    std::vector<STRUCT_GT_SENSORS_ALL> m_sensor_gt_all;

private:

    //ObjectImageShapeData m_sensorMetaData_shape;
    std::string m_sensorMetaData_name;
    ushort m_sensorMetaData_startPoint;


public:

    SensorMetaData() {
        for (ushort i = 0; i < Dataset::MAX_ITERATION_DATASET; i++) {
            STRUCT_GT_SENSORS_ALL s = {};
            m_sensor_gt_all.push_back(s);
        }
    };

    SensorMetaData(std::string name, ushort startPoint) :
            m_sensorMetaData_name(name), m_sensorMetaData_startPoint(startPoint) {
        for (ushort i = 0; i < Dataset::MAX_ITERATION_DATASET; i++) {
            STRUCT_GT_SENSORS_ALL s = {};
            m_sensor_gt_all.push_back(s);
        }
    } ;

    void setStartPoint(ushort startPoint) {
        m_sensorMetaData_startPoint = startPoint;
    }

    void fillData() {

    }

    virtual void process(cv::Size frame_size, ushort startTrajectoryPoint) {};

    virtual void pushPositionPoints(cv::Point2f points) {}

    virtual void pushVisibility(bool visibility) {}

    void atFrameNumberSensorState(ushort frameNumber, cv::Point3f position_carrier, cv::Point3f orientation_carrier,  cv::Point3f orientation_sensor, cv::Point3f offset_sensor, cv::Point2f fov,
            cv::Point2f clip) {

        m_sensor_gt_all.at(frameNumber).m_sensor_location_carrier_m.location_x_m = position_carrier.x;
        m_sensor_gt_all.at(frameNumber).m_sensor_location_carrier_m.location_y_m = position_carrier.y;
        m_sensor_gt_all.at(frameNumber).m_sensor_location_carrier_m.location_z_m = position_carrier.z;

        m_sensor_gt_all.at(frameNumber).m_sensor_rotation_carrier_rad.rotation_rz_yaw_rad = orientation_carrier.x; //h
        m_sensor_gt_all.at(frameNumber).m_sensor_rotation_carrier_rad.rotation_ry_pitch_rad = orientation_carrier.y; //p
        m_sensor_gt_all.at(frameNumber).m_sensor_rotation_carrier_rad.rotation_rx_roll_rad = orientation_carrier.z; //r

        m_sensor_gt_all.at(frameNumber).m_sensor_rotation_rad.rotation_rz_yaw_rad = orientation_sensor.x;
        m_sensor_gt_all.at(frameNumber).m_sensor_rotation_rad.rotation_ry_pitch_rad = orientation_sensor.y;
        m_sensor_gt_all.at(frameNumber).m_sensor_rotation_rad.rotation_rx_roll_rad = orientation_sensor.z;

        m_sensor_gt_all.at(frameNumber).m_sensor_offset_m.offset_x = offset_sensor.x;
        m_sensor_gt_all.at(frameNumber).m_sensor_offset_m.offset_y = offset_sensor.y;
        m_sensor_gt_all.at(frameNumber).m_sensor_offset_m.offset_z = offset_sensor.z;

        m_sensor_gt_all.at(frameNumber).m_sensor_cam_info.fov_horizontal_rad = fov.x;
        m_sensor_gt_all.at(frameNumber).m_sensor_cam_info.fov_vertical_rad = fov.y;
        m_sensor_gt_all.at(frameNumber).m_sensor_cam_info.near_m = clip.x;
        m_sensor_gt_all.at(frameNumber).m_sensor_cam_info.far_m = clip.y;


    }

    void atFrameNumberVisibility(ushort frameNumber, bool visibility) {
        m_sensor_gt_all.at(frameNumber).occluded = visibility;
    }

    std::vector<STRUCT_GT_SENSORS_ALL> getAll() const {
        return m_sensor_gt_all;
    }

    ushort& getSensorStartPoint() {
        return m_sensorMetaData_startPoint;
    }

    const std::string &getSensorName() const {
        return m_sensorMetaData_name;
    }

    void setSensorName(std::string sensorName) {
        m_sensorMetaData_name = sensorName;
    }

    void setDynamic() {

        cv::RNG rng(cv::getTickCount());
        for (ushort i = 0; i < Dataset::MAX_ITERATION_DATASET / 20; i++) {
            ushort index = (ushort) rng.uniform(0, 360);
            m_sensor_gt_all.at(index).visMask = 0;
            //TODO - set pixel position
        }
        for ( auto t : m_sensor_gt_all) {
            std::cout << t.visMask;
        }
        std::cout << std::endl;

        for (ushort i = 0; i < Dataset::MAX_ITERATION_DATASET; i++) {

            if (i < (m_sensor_gt_all.size() - 1) && m_sensor_gt_all.at(i + 1).visMask == 0) {
                m_sensor_gt_all.at(i).visMask = 0;
            }
        }

        for ( auto t : m_sensor_gt_all ) {
            std::cout << t.visMask;
        }

        std::cout << std::endl;
        for (ushort i = Dataset::MAX_ITERATION_DATASET; i > 0; i--) {

            if (i < (m_sensor_gt_all.size() - 1) && m_sensor_gt_all.at(i - 1).visMask == 0) {
                m_sensor_gt_all.at(i).visMask = 0;
            }
        }

        for ( auto t : m_sensor_gt_all ) {
            std::cout << t.visMask;
        }
        std::cout << std::endl;
    }

};




#endif //MAIN_SENSORMETADATA_H
