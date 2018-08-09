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

    cv::Mat m_data_image;
    cv::Mat m_data_depth;
    ushort m_objectWidth;
    ushort m_objectHeight;
    ushort m_depth;

public:

    ObjectImageShapeData() {};

    ObjectImageShapeData(ushort width, ushort height, std::unique_ptr<Noise> &noise, ushort depth ) : m_objectWidth(width), m_objectHeight
            (height), m_depth(depth) {
    }

    virtual void process() {};

    void applyNoise(std::unique_ptr<Noise> &noise) {
        noise->apply(m_data_image);
    }

    void applyDepth(ushort depth) {
        m_data_depth.setTo(depth);
    }

    cv::Mat getImage() {
        return m_data_image;
    }

    cv::Mat getDepth() {
        return m_data_depth;
    }

};


// Canvas is a kind of Object ( its just a big object, and hence has the same property )
class Canvas  : public ObjectImageShapeData {

private:


public:


    // The canvas can move to simulate a moving car. Hence we need position etc.
    Canvas( ushort width, ushort height, std::unique_ptr<Noise> &noise ) : ObjectImageShapeData(width, height, noise, 0) {
        m_data_image.create(height, width, CV_8UC3);
        applyNoise(noise);
    };

    void process() override ;


};


class Rectangle :  public ObjectImageShapeData {

private:

public:

    Rectangle(ushort width, ushort height, std::unique_ptr<Noise> &noise, ushort depth) : ObjectImageShapeData(width, height, noise, depth) {
        m_data_image.create(height, width, CV_8UC3);
        m_data_depth.create(height, width, CV_8UC1);
        applyNoise(noise);
        applyDepth(depth);
    };

    void process() override ;

};

class Circle :  public ObjectImageShapeData {

private:
    ushort m_objectRadius;

public:

    Circle(ushort radius, std::unique_ptr<Noise> &noise, ushort depth) : m_objectRadius(radius/2), ObjectImageShapeData(radius, radius, noise, depth) {
        m_data_image.create(radius, radius, CV_8UC3);
        m_data_image.setTo(cv::Scalar(255,255,255));
        m_data_depth.create(radius, radius, CV_8UC1);
        cv::circle(m_data_image, cv::Point(m_objectRadius, m_objectRadius), m_objectRadius, cv::Scalar(255,0,0), CV_FILLED);
        applyNoise(noise);
        applyDepth(depth);
    };

    void process() override ;

};

#endif //MAIN_OBJECTSHAPE_H
