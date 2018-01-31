//
// Created by veikas on 30.01.18.
//

#ifndef MAIN_SENSORIMAGE_H
#define MAIN_SENSORIMAGE_H

#include <opencv2/core/mat.hpp>
#include "Noise.h"
#include "ObjectShapeImageData.h"

class Noise;

class SensorImage {

public:
    virtual void setNoise( Noise &noise ) {}

    virtual void appendNoise( Noise &noise )  {}

    virtual void clearNoise() {}

};

class CameraSensorImage : public SensorImage {
// How does object look like from the perspective of a camera?
protected:
    Noise &m_noise;
    ObjectShapeImageData &m_shape;

public:

    CameraSensorImage(ObjectShapeImageData &shape, Noise &noise):m_shape(shape), m_noise(noise) {}

    virtual void setNoise( Noise &noise ) override {
        SensorImage image;
        //noise.set(image);
    }

    virtual void appendNoise( Noise &noise ) override {
        SensorImage image;
        //noise.append();
    }

    virtual void clearNoise() override {
    }

    ObjectShapeImageData getShapeImageData() const {
        return m_shape;
    }
};

class RadarSensorImage : public SensorImage {
// How does object look like from the perspective of a radar?
};


#endif //MAIN_SENSORIMAGE_H
