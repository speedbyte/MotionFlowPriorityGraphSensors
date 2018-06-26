//
// Created by veikas on 30.01.18.
//

#ifndef MAIN_SENSORIMAGE_H
#define MAIN_SENSORIMAGE_H

#include <opencv2/core/mat.hpp>
#include "Noise.h"

class Noise;

class SensorImage {

public:
    virtual void setNoise( Noise &noise ) {}

    virtual void appendNoise( Noise &noise )  {}

    virtual void clearNoise() {}

};


class CameraSensor : public SensorImage {
// How does object look like from the perspective of a camera?
protected:

    Noise m_noise;
    cv::Matx33f camaera_intrinsic_parameters;
    cv::Matx33f camaera_pose_parameters;


public:

    CameraSensor(Noise &noise) : m_noise(noise) {

    }

    virtual void setNoise( Noise &noise ) override {
        //SensorImage image;
        m_noise = noise;
    }

    virtual void appendNoise( Noise &noise ) override {
        //SensorImage image;
        //noise.append();
    }

    virtual void clearNoise() override {
    }

};

class RadarSensor : public SensorImage {
// How does object look like from the perspective of a radar?
};



#endif //MAIN_SENSORIMAGE_H
