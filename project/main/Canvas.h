//
// Created by veikas on 30.01.18.
//

#ifndef MAIN_CANVAS_H
#define MAIN_CANVAS_H


#include <opencv2/core/mat.hpp>
#include "SensorImage.h"
#include "Dataset.h"
#include "GroundTruthObjects.h"


// Canvas is a kind of Object ( its just a big object, and hence has the same property )
class Canvas : public GroundTruthObjects {


public:

    // The canvas can move to simulate a moving car. Hence we need trajectory etc.
    Canvas ( ObjectImageShapeData &image_data_and_shape, ObjectTrajectory &trajectory, ushort startPoint, Noise
    &noise) : GroundTruthObjects(image_data_and_shape, trajectory, startPoint, noise, "BackgroundCanvas") {
        image_data_and_shape.process();
    };

};


#endif //MAIN_CANVAS_H
