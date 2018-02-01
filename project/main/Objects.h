//
// Created by veikas on 28.01.18.
//

#ifndef MAIN_OBJECTPROPERTIES_H
#define MAIN_OBJECTPROPERTIES_H


// Objects are either CameraSensorImage or RadarSensorImage

#include "SensorImage.h"
#include "Dataset.h"
#include "ObjectTrajectory.h"
#include "ObjectFlow.h"
#include "datasets.h"

class Objects : public CameraSensorImage {

private:

    
    const ushort m_startPoint;

    ObjectTrajectory &m_obj_trajectory;
    ObjectFlow m_obj_flow;

    static unsigned objectCurrentCount;

    const unsigned m_objectId;
    const std::string m_objectName;

public:

    Objects( ObjectShapeImageData &shape, ObjectTrajectory &trajectory, ushort startPoint, Noise
    &noise, std::string objectName ) : CameraSensorImage(shape, noise), m_obj_trajectory
            (trajectory), m_startPoint(startPoint) , m_objectName ( objectName ), m_objectId
                                               (objectCurrentCount) {

        shape.process();
        trajectory.process(Dataset::getFrameSize());
        objectCurrentCount += 1;

        m_obj_flow = ObjectFlow();

        printf("generating ground truth basic displacement for name %s with object id %u\n", getObjectName().c_str
                (), getObjectId());

        m_obj_flow.generate_baseframe_flow_vector( m_startPoint, m_obj_trajectory.get());

        printf("generating ground truth frame displacement for name %s with object id %u\n", getObjectName().c_str
                (), getObjectId());

        m_obj_flow.generate_multiframe_flow_vector( MAX_SKIPS );
    }


    ObjectTrajectory getTrajectoryPoints() {
        return m_obj_trajectory;
    }

    ObjectFlow getFlowPoints() {
        return m_obj_flow;
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
