//
// Created by veikas on 30.01.18.
//

#ifndef MAIN_NOISE_H
#define MAIN_NOISE_H


#include <opencv2/imgproc.hpp>
#include "SensorImage.h"

class SensorImage;

class Noise {

public:

    virtual void apply(SensorImage &image) {

        clear(); // restore original image without noise and then apply
        append();
    }

    virtual void append() {
       // append the noise on already noised element
    }

    virtual void clear() {

    }
};

class GuassianNoise: public Noise {

public:
    void apply(SensorImage &image) override {
        //cv::GaussianBlur();// manipulate image with noise on this and return
    }

};

class Reflection: public Noise {
public:
    void apply() {
        // manipulate image with noise and return
    }
};

class NoNoise: public Noise {

public:
    void apply() {
        // manipulate image with noise and return
    }
};




#endif //MAIN_NOISE_H
