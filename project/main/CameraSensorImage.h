//
// Created by veikas on 02.02.18.
//

#ifndef MAIN_CAMERASENSORIMAGE_H
#define MAIN_CAMERASENSORIMAGE_H


#include "Noise.h"
#include "SensorImage.h"

class CameraSensorImage : public SensorImage {
// How does object look like from the perspective of a camera?
protected:

    Noise &m_noise; // TODO use a own copy instead of reference.
    ObjectImageShapeData m_image_data_and_shape;
    cv::Matx33f camaera_intrinsic_parameters;
    cv::Matx33f camaera_pose_parameters;


public:

    CameraSensorImage(ObjectImageShapeData &image_data_and_shape, Noise &noise):m_image_data_and_shape
                                                                                        (image_data_and_shape), m_noise(noise) {
        m_noise.apply(m_image_data_and_shape);
    }

    virtual void setNoise( Noise &noise ) override {
        //SensorImage image;
        m_noise = noise;
        m_noise.apply(m_image_data_and_shape);
    }

    virtual void appendNoise( Noise &noise ) override {
        //SensorImage image;
        //noise.append();
    }

    virtual void clearNoise() override {
    }

    ObjectImageShapeData getImageShapeAndData() const override {
        return m_image_data_and_shape;
    }
};

class RadarSensorImage : public SensorImage {
// How does object look like from the perspective of a radar?
};



#endif //MAIN_CAMERASENSORIMAGE_H
