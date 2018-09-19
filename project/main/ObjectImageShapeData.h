//
// Created by veikas on 17.09.18.
//

#ifndef MAIN_OBJECTIMAGESHAPEDATA_H
#define MAIN_OBJECTIMAGESHAPEDATA_H


#include <opencv2/core/mat.hpp>
#include <memory>
#include "Noise.h"

class ObjectImageShapeData {

protected:

    cv::Mat m_data_image;
    cv::Mat m_data_depth;

    ushort m_objectWidth;
    ushort m_objectHeight;
    ushort m_objectRadius;

    float m_depth;

public:

    ObjectImageShapeData() {};

    ObjectImageShapeData(ushort width, ushort height, std::unique_ptr<Noise> &noise, ushort depth ) : m_objectRadius(std::max(width, height)/2), m_objectWidth(width), m_objectHeight
            (height), m_depth(depth) {
        m_data_depth.setTo(m_depth);
    }

    virtual void process() {};

    void applyNoise(std::unique_ptr<Noise> &noise) {
        noise->apply(m_data_image);
    }

    void applyDepth(ushort depth) {
        m_depth = depth;
        m_data_depth.setTo(depth);
    }

    cv::Mat getImage() {
        return m_data_image;
    }

    cv::Mat getDepthImage() {
        return m_data_depth;
    }

    float getObjectDepth() {
        return m_depth;
    }

    ushort getObjectWidth() {
        return m_objectWidth;
    }

    ushort getObjectHeight() {
        return m_objectHeight;
    }

    ushort getObjectRadius() {
        return m_objectRadius;
    }

    void addObjectShape(cv::Point center, ushort radius)
    {
        cv::Scalar color = cv::Scalar(1,1,1);
        //cv::fillConvexPoly(tempGroundTruthDepthImage, contours.at(0), cv::Scalar(255,0,0));
        cv::circle(m_data_image, center, radius, color, CV_FILLED);
    }

};

// ObjectImageBackgroundShapeData is a kind of Object ( its just a big object, and hence has the same property )
class ObjectImageBackgroundShapeData  : public ObjectImageShapeData {

private:


public:

    // The canvas can move to simulate a moving car. Hence we need position etc.
    ObjectImageBackgroundShapeData( ushort width, ushort height, std::unique_ptr<Noise> &noise ) : ObjectImageShapeData(width, height, noise, 0) {
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

public:

    Circle(ushort diameter, std::unique_ptr<Noise> &noise, ushort depth) : ObjectImageShapeData(diameter, diameter, noise, depth) {

    };

    void process() override ;

};



#endif //MAIN_OBJECTIMAGESHAPEDATA_H
