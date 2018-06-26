//
// Created by veikas on 28.01.18.
//

#ifndef MAIN_OBJECTSHAPE_H
#define MAIN_OBJECTSHAPE_H

#include <opencv2/core/mat.hpp>
#include <bits/unique_ptr.h>
#include "Noise.h"

class ObjectImageShapeData {

protected:

    cv::Mat m_data;
    ushort m_objectWidth;
    ushort m_objectHeight;

public:

    ObjectImageShapeData() {};

    ObjectImageShapeData(ushort width, ushort height, std::unique_ptr<Noise> &noise ) : m_objectWidth(width), m_objectHeight
            (height){
        m_data.create(height, width, CV_8UC3);
        applyNoise(noise);
    }

    virtual void process() {};

    void applyNoise(std::unique_ptr<Noise> &noise) {
        noise->apply(m_data);
    }

    cv::Mat get() {
        return m_data;
    }

};


// Canvas is a kind of Object ( its just a big object, and hence has the same property )
class Canvas  : public ObjectImageShapeData {

private:


public:


    // The canvas can move to simulate a moving car. Hence we need position etc.
    Canvas( ushort width, ushort height, std::unique_ptr<Noise> &noise ) : ObjectImageShapeData(width, height, noise) {
    };

    void process() override ;


};


class Rectangle :  public ObjectImageShapeData {

private:

public:

    Rectangle(ushort width, ushort height, std::unique_ptr<Noise> &noise) : ObjectImageShapeData(width, height, noise) {};

    void process() override ;

};

#endif //MAIN_OBJECTSHAPE_H
