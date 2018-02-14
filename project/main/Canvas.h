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
class Canvas  {

private:

    WhiteNoise whiteNoise;
    Rectangle background;

public:


    // The canvas can move to simulate a moving car. Hence we need trajectory etc.
    Canvas( ) : whiteNoise(), background((ushort)Dataset::getFrameSize().width, (ushort)Dataset::getFrameSize().height){
        CameraSensorImage(background, whiteNoise);
        background.process();
    };

    ObjectImageShapeData getImageShapeAndData() {
        return background;
    }
};


#endif //MAIN_CANVAS_H
