//
// Created by veikas on 28.01.18.
//

#ifndef MAIN_OBJECTPROPERTIES_H
#define MAIN_OBJECTPROPERTIES_H


#include "ObjectTrajectory.h"

class ObjectProperties {

private:
    ObjectShape &obj_shape;
    ObjectTrajectory &obj_trajectory;
    ushort m_startPoint;

public:

    ObjectProperties( Dataset dataset, ObjectShape &shape, ObjectTrajectory &trajectory, ushort startPoint) :
            obj_shape (shape ),obj_trajectory (trajectory), m_startPoint(startPoint) {
        shape.process();
        trajectory.process(dataset.getFrameSize());
    }

    ObjectShape getShape() {
        return obj_shape;
    }

    ObjectTrajectory getTrajectoryPoints() {
        return obj_trajectory;
    }

    ushort getStartPoint() {
        return m_startPoint;
    }

};



#endif //MAIN_OBJECTPROPERTIES_H
