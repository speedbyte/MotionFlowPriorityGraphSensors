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

    Noise &m_noise;
    ObjectImageShapeData &m_image_data_and_shape; // TODO use a own copy instead of reference.
    cv::Matx33f camaera_intrinsic_parameters;
    cv::Matx33f camaera_pose_parameters;

    /*
     Remarks about 3D information

Internally, the 3D world is projected on the screen by using the Unity Engine rendering pipeline and various shaders. You can reproduce this by projecting points from the camera space (e.g., coordinates x3d,y3d,z3d) to the image pixels by using our camera intrinsic matrix (in pixels, constant, computed from our 1242x375 resolution and 29° fov):

          [[725,    0, 620.5],
    K =  [   0, 725, 187.0],
            [   0,     0,       1]]

In our system of 3D camera coordinates x is going to the right, y is going down, and z is going forward (the origin is the optical center of the camera).*
     */

    /*
     * Camera pose (extrinsic parameters): link (1.1MB)

The 3D camera pose (rotation and translation) for each frame of a video consists of one CSV-like text files named:

    vkitti_<version>_extrinsicsgt/<world>_<variation>.txt

Each file can be loaded with the following one-liner in Python using the popular pandas library (assuming ‘import pandas as pd’):

    extgt = pd.read_csv("<filename>", sep=" ", index_col=False)

Each line consists of the frame index in the video (starts from 0) followed by the row-wise flattened 4x4 extrinsic matrix at that frame:

           r1,1 r1,2 r1,3 t1
    M = r2,1 r2,2 r2,3 t2
           r3,1 r3,2 r3,3 t3
             0     0     0    1

where ri,j are the coefficients of the camera rotation matrix R and ti are the coefficients of the camera translation coefficients t.

This matrix can be used to convert points from world space to the camera space. For a point p = (x,y,z) in the world space, P = (x,y,z,1) in the homogeneous coordinates, you can get the coordinates in the camera space by doing the dot product MP.

See section above for the camera intrinsic parameters and description of our camera coordinate system.
     */




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
