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

    Dataset &m_dataset;
    const ushort m_startPoint;

    ObjectTrajectory &m_obj_trajectory;
    ObjectFlow m_obj_flow;

public:

    Objects( Dataset &dataset, ObjectShapeImageData &shape, ObjectTrajectory &trajectory, ushort startPoint, Noise
    &noise, ObjectFlow objectFlow) : CameraSensorImage(shape, noise), m_dataset(dataset), m_obj_trajectory
            (trajectory), m_startPoint(startPoint), m_obj_flow(objectFlow) {

        shape.process();
        trajectory.process(dataset.getFrameSize());
        m_obj_flow = ObjectFlow();
    }

    Objects( Dataset &dataset, ObjectShapeImageData &shape, ObjectTrajectory &trajectory, ushort startPoint, Noise
    &noise) : CameraSensorImage(shape, noise), m_dataset(dataset), m_obj_trajectory
            (trajectory), m_startPoint(startPoint) {

        shape.process();
        trajectory.process(dataset.getFrameSize());
    }

    ObjectTrajectory getTrajectoryPoints() {
        return m_obj_trajectory;
    }

    ObjectFlow getFlowPoints() {
        return m_obj_flow;
    }

    void generate_base_flow_vector()  {
        m_obj_flow.generate_base_flow_vector(m_dataset, m_startPoint, m_obj_trajectory.get());
    }

    void generate_extended_flow_vector()  {
        m_obj_flow.generate_extended_flow_vector(m_dataset, MAX_SKIPS );
    }

    ushort getStartPoint() {
        return m_startPoint;
    }

};



#endif //MAIN_OBJECTPROPERTIES_H
