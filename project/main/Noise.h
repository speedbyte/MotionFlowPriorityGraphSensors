//
// Created by veikas on 30.01.18.
//

#ifndef MAIN_NOISE_H
#define MAIN_NOISE_H


#include "SensorImage.h"

class SensorImage;

class Noise {

public:

    virtual void set(SensorImage &image) {
        // clear the element and repeats the noise
    }

    virtual void append() {
       // append the noise on already noised element
    }

    // we cannot clear a noise, because we dont have the original. To solve this
    // the original image needs to be stored somewhere.
};

class GuassianNoise: public Noise {

public:
    void set(SensorImage &image) override {
        // manipulate image with noise on this and return
    }
};

class Reflection: public Noise {
public:
    void set() {
        // manipulate image with noise and return
    }
};

class NoNoise: public Noise {

public:
    void set() {
        // manipulate image with noise and return
    }
};

#endif //MAIN_NOISE_H
