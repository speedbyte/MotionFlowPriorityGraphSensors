//
// Created by veikas on 21.02.18.
//

#ifndef MAIN_OBJECTMETADATA_H
#define MAIN_OBJECTMETADATA_H

#include <iostream>
#include <opencv2/core/mat.hpp>
#include <vires-interface/Common/viRDBIcd.h>
#include "ObjectImageShapeData.h"
#include "Dataset.h"


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
        for (ushort i = 0; i < Dataset::MAX_GENERATION_DATASET; i++) {
            STRUCT_GT_OBJECTS_ALL s = {};
            m_object_gt_all.push_back(s);
        }
    };

    ObjectMetaData(ObjectImageShapeData shape, std::string name, ushort startPoint) :
            m_objectMetaData_shape(shape), m_objectMetaData_startPoint(startPoint) {
        for (ushort i = 0; i < Dataset::MAX_GENERATION_DATASET; i++) {
            STRUCT_GT_OBJECTS_ALL s = {};
            m_object_gt_all.push_back(s);
            m_object_gt_all.at(i).object_name = name;
        }
    } ;

    ObjectMetaData(std::string name, ushort startPoint) : m_objectMetaData_startPoint(startPoint) {
        for (ushort i = 0; i < Dataset::MAX_GENERATION_DATASET; i++) {
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
        for (ushort i = 0; i < Dataset::MAX_GENERATION_DATASET; i++) {
            m_object_gt_all.at(i).object_name = objectName;
        }
    }

    void setObjectShape(ObjectImageShapeData &objectShape) {
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

    void setCppData(ushort width, ushort height, ushort depth);

    void setCppDataGenerate();

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
        for (ushort i = 0; i < Dataset::MAX_GENERATION_DATASET/ 20; i++) {
            ushort index = (ushort) rng.uniform(0, 360);
            m_object_gt_all.at(index).visMask = 0;
            //TODO - set pixel position
        }
        for ( auto t : m_object_gt_all) {
            std::cout << t.visMask;
        }
        std::cout << std::endl;

        for (ushort i = 0; i < Dataset::MAX_GENERATION_DATASET; i++) {

            if (i < (m_object_gt_all.size() - 1) && m_object_gt_all.at(i + (ushort) 1).visMask == 0) {
                m_object_gt_all.at(i).visMask = 0;
            }
        }

        for ( auto t : m_object_gt_all ) {
            std::cout << t.visMask;
        }

        std::cout << std::endl;
        for (ushort i = Dataset::MAX_GENERATION_DATASET; i > 0; i--) {

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



#endif //MAIN_OBJECTMETADATA_H
