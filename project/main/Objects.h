//
// Created by veikas on 28.01.18.
//

#ifndef MAIN_OBJECTPROPERTIES_H
#define MAIN_OBJECTPROPERTIES_H


// Objects are either CameraSensorImage or RadarSensorImage

#include "SensorImage.h"
#include "Dataset.h"
#include "ObjectTrajectory.h"
#include "datasets.h"

class Objects : public CameraSensorImage {

private:

    
    const ushort m_startPoint;

    ObjectTrajectory &m_obj_trajectory;

    static unsigned objectCurrentCount;

    const unsigned m_objectId;
    const std::string m_objectName;

    std::vector<std::pair<cv::Point2i, cv::Point2i> > m_obj_flow_point_base_movement;
    std::vector<std::vector<std::pair<cv::Point2i, cv::Point2i> > > m_obj_flow_point_fast_movement;

public:

    Objects( ObjectShapeImageData &shape, ObjectTrajectory &trajectory, ushort startPoint, Noise
    &noise, std::string objectName ) : CameraSensorImage(shape, noise), m_obj_trajectory
            (trajectory), m_startPoint(startPoint) , m_objectName ( objectName ), m_objectId
                                               (objectCurrentCount) {

        shape.process();
        trajectory.process(Dataset::getFrameSize());
        objectCurrentCount += 1;

        printf("generating ground truth basic displacement for name %s with object id %u\n", getObjectName().c_str
                (), getObjectId());

        generate_baseframe_flow_vector( m_startPoint, m_obj_trajectory.get());

        printf("generating ground truth frame displacement for name %s with object id %u\n", getObjectName().c_str
                (), getObjectId());

        generate_multiframe_flow_vector( MAX_SKIPS );
    }

    void generate_baseframe_flow_vector(const ushort &start_point, const std::vector<cv::Point2i>
    &trajectory_points);

    void generate_multiframe_flow_vector(const int &max_skips);


    ObjectTrajectory getTrajectoryPoints() {
        return m_obj_trajectory;
    }

    std::vector<std::pair<cv::Point2i, cv::Point2i> >  getBasePoints() {
        return m_obj_flow_point_base_movement;
    }

    std::vector<std::vector<std::pair<cv::Point2i, cv::Point2i> > >  getFlowPoints() {
        return m_obj_flow_point_fast_movement;
    }

    ushort getStartPoint() {
        return m_startPoint;
    }

    unsigned getObjectId() {
        return m_objectId;
    }

    std::string getObjectName() {
        return m_objectName;
    }
};



#endif //MAIN_OBJECTPROPERTIES_H
