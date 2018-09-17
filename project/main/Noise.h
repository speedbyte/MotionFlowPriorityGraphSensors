//
// Created by veikas on 30.01.18.
//

#ifndef MAIN_NOISE_H
#define MAIN_NOISE_H


#include <opencv2/imgproc.hpp>
#include <iostream>

//class SensorImage;

class Noise {

public:

    virtual void apply(cv::Mat &image) {

        clear(); // restore original image without noise and then apply
        append();
    }

    virtual void append() {
       // append the noise on already noised element
    }

    virtual void clear() {

    }
};

class GlobalScaleChange: public Noise {

};

class GlobalGammaChange: public Noise {

};

class Vignetting : public Noise {

};

class GuassianNoise: public Noise {

public:
    void apply(cv::Mat &image) override {
        //cv::GaussianBlur();// manipulate image with noise on this and return
    }

};



template <ushort data>
class StaticNoise : public Noise {

    // Brute force noise. Blindly assign data to the Canvas

public:

    void apply(cv::Mat &image) override {

        srand(1000);

        if ( data == 0 ) {
            std::cout << "applying black static noise" << std::endl;
            image = cv::Scalar(data, data, data);
        } else if ( data == 255 ) {
            image = cv::Scalar(data, data, data);
            std::cout << "applying white static noise" << std::endl;
        } else {
            std::cout << "applying colored static noise" << std::endl;
            image = cv::Scalar(rand()%255, rand()%255, rand()%255);
        }
    }
};


template <ushort data>
class DynamicNoise : public Noise {

public:

    void apply(cv::Mat &image) override {

        srand(time(NULL));

        if (data == 0) {
            std::cout << "applying black dynamic noise" << std::endl;
            image = cv::Scalar(data, data, data);
        } else if (data == 255) {
            image = cv::Scalar(data, data, data);
            std::cout << "applying white dynamic noise" << std::endl;
        } else {
            std::cout << "applying colored dynamic noise" << std::endl;
            image = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
        }
    }

};

class ColorfulNoise : public Noise {

public:

    void apply(cv::Mat &image) override {

        // get the existing image data and apply noise on the top of it.

        uchar r = 0;
        uchar b = 0;

        r = 0;
        b = 0;

        for (int k = 0; k < ( image.rows - 1); k++) {
            for (int j = 0; j < (image.cols -1 ); j++) {
                image.at<cv::Vec3b>(k, j)[0] = b;
                image.at<cv::Vec3b>(k, j)[1] = 0;
                image.at<cv::Vec3b>(k, j)[2] = r;
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


class RandomNoise : public Noise {

public:

    void apply(cv::Mat &image) override {

        std::cout << "applying random noise" << std::endl;
        cv::randu(image, 0, 255);
    }
};

class Reflection: public Noise {
public:
    void apply(cv::Mat &image) override{
        // manipulate image with noise and return
    }
};

class NoNoise: public Noise {

public:
    void apply(cv::Mat &image) override {

        // dont do anything with the object or canvas
    }
};



#endif //MAIN_NOISE_H
