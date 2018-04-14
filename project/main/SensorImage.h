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


class CameraSensor : public SensorImage {
// How does object look like from the perspective of a camera?
protected:

    Noise m_noise;
    ObjectImageShapeData m_image_data_and_shape;
    cv::Matx33f camaera_intrinsic_parameters;
    cv::Matx33f camaera_pose_parameters;


public:

    CameraSensor(ObjectImageShapeData &image_data_and_shape, Noise &noise) : m_image_data_and_shape(image_data_and_shape),
            m_noise(noise) {

        m_image_data_and_shape.applyNoise(&m_noise);
    }

    virtual void setNoise( Noise &noise ) override {
        //SensorImage image;
        m_noise = noise;
        m_image_data_and_shape.applyNoise(&m_noise);
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
