//
// Created by veikas on 30.01.18.
//

#ifndef MAIN_SENSORIMAGE_H
#define MAIN_SENSORIMAGE_H

#include <opencv2/core/mat.hpp>
#include "Noise.h"
#include "ObjectImageShapeData.h"

class Noise;

class SensorImage {

public:
    virtual void setNoise( Noise &noise ) {}

    virtual void appendNoise( Noise &noise )  {}

    virtual void clearNoise() {}

    virtual ObjectImageShapeData getImageShapeAndData() const  {
    }
};


#endif //MAIN_SENSORIMAGE_H
