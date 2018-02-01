//
// Created by veikas on 30.01.18.
//

#ifndef MAIN_CANVAS_H
#define MAIN_CANVAS_H


#include <opencv2/core/mat.hpp>
#include "SensorImage.h"
#include "Dataset.h"
#include "Objects.h"


// Canvas is a kind of Object ( its just a big object, and hence has the same property )
class Canvas : public Objects {


public:

    // The canvas can move to simulate a moving car. Hence we need trajectory etc.
    Canvas ( Dataset &dataset, ObjectShapeImageData &shape, ObjectTrajectory &trajectory, ushort startPoint, Noise
    &noise) : Objects(dataset, shape, trajectory, startPoint, noise, "BackgroundCanvas") {
        shape.process();
    };

};


#endif //MAIN_CANVAS_H
