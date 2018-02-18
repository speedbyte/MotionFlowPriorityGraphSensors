//
// Created by veikas on 30.01.18.
//

#ifndef MAIN_NOISE_H
#define MAIN_NOISE_H


#include <opencv2/imgproc.hpp>
#include <iostream>
#include "ObjectImageShapeData.h"

//class SensorImage;

class Noise {

public:

    virtual void apply(ObjectImageShapeData &image) {

        clear(); // restore original image without noise and then apply
        append();
    }

    virtual void append() {
       // append the noise on already noised element
    }

    virtual void clear() {

    }
};

class ColorfulNoise : public Noise {

public:

    void apply(ObjectImageShapeData &image) override {

        std::cout << "rows"  << image.get().rows << std::endl;

        uchar r = 0;
        uchar b = 0;

        r = 0;
        b = 0;

        for (int k = 0; k < ( image.get().rows - 1); k++) {
            for (int j = 0; j < (image.get().cols -1 ); j++) {
                image.get().at<cv::Vec3b>(k, j)[0] = b;
                image.get().at<cv::Vec3b>(k, j)[1] = 0;
                image.get().at<cv::Vec3b>(k, j)[2] = r;
                r = r + (uchar)2;
                b = b + (uchar)2;
                if (r > 254)
                    r = 130;
            }
            if (b > 254)
                b = 46;
        }
    }
};

class WhiteNoise : public Noise {

public:

    void apply(ObjectImageShapeData &image) override {

        std::cout << "applying white noise" << std::endl;
        image.get() = cv::Scalar(255,255,255);
    }
};


class GuassianNoise: public Noise {

public:
    void apply(ObjectImageShapeData &image) override {
        //cv::GaussianBlur();// manipulate image with noise on this and return
    }

};

class Reflection: public Noise {
public:
    void apply(ObjectImageShapeData &image) override{
        // manipulate image with noise and return
    }
};

class NoNoise: public Noise {

public:
    void apply(ObjectImageShapeData &image) override {

        std::cout << "plain blue item" << std::endl;
        image.get() = cv::Scalar(255,0,255);
        // manipulate image with noise and return
    }
};




#endif //MAIN_NOISE_H
