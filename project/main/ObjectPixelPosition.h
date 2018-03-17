//
// Created by veikas on 25.01.18.
//

#ifndef MAIN_OBJECTTRAJECTORY_H
#define MAIN_OBJECTTRAJECTORY_H


#include "Dataset.h"
#include "ObjectImageShapeData.h"
#include <iostream>
#include "datasets.h"
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>

class ObjectDimensions {

private:
    std::vector<cv::Point2f> m_dimensions;

public:

    ObjectDimensions() {
        for (ushort i = 0; i < MAX_ITERATION_THETA; i++) {
            m_dimensions.push_back(cv::Point2f(0,0));
        }
    }

    void atFrameNumber(ushort frameNumber, cv::Point2f points) {
        m_dimensions.at(frameNumber) = points;
    }

    std::vector<cv::Point2f> getObjectPixelDimensions() const {
        return m_dimensions;
    }

    void process(cv::Size frame_size) {
        std::vector<ushort> theta;
        for ( ushort frame_count = 0; frame_count < MAX_ITERATION_THETA; frame_count++) {
            theta.push_back(frame_count);
        }
        // Prepare points
        cv::Point2f l_pixel_dimension;
        for ( int i = 0; i< MAX_ITERATION_THETA; i++) {

            l_pixel_dimension.x = static_cast<float>((frame_size.width/2) + (100 * cos(theta[i] *CV_PI / 180.0) /
                                                                            (1.0 + std::pow(sin(theta[i] * CV_PI / 180.0), 2))));

            l_pixel_dimension.y = static_cast<float>((frame_size.height/2) + (55 * (cos(theta[i] * CV_PI / 180.0) *
                                                                                   sin(theta[i] * CV_PI / 180.0)) /
                                                                             (0.2 +std::pow(sin(theta[i] * CV_PI / 180.0),2))));

            l_pixel_dimension.x /= 10;
            l_pixel_dimension.y /= 10;

            m_dimensions.at(i) = (l_pixel_dimension);
        }
    }

};


class ObjectPixelPosition {

protected:
    std::vector<cv::Point2f> m_position;
    std::vector<cv::Point2f> m_offset;
    std::vector<bool> m_visibility;

public:

    ObjectPixelPosition() {
        for (ushort i = 0; i < MAX_ITERATION_THETA; i++) {
            m_visibility.push_back(false);
            m_position.push_back(cv::Point2f(-1,-1));
            m_offset.push_back(cv::Point2f(0,0));
        }
    };

    virtual void process(cv::Size frame_size) {};

    virtual void pushPositionPoints(cv::Point2f points) {}

    virtual void pushVisibility(bool visibility) {}

    void atFrameNumber(ushort frameNumber, cv::Point2f position, cv::Point2f offset, bool visibility) {
        m_position.at(frameNumber) = position;
        m_offset.at(frameNumber) = offset;
        m_visibility.at(frameNumber) = visibility;
    }

    std::vector<cv::Point2f> getPosition() const {
        return m_position;
    }

    std::vector<bool> getVisibility() const {
        return m_visibility;
    }

    void setDynamic() {

        cv::RNG rng(cv::getTickCount());
        for (ushort i = 0; i < MAX_ITERATION_THETA / 20; i++) {
            ushort index = (ushort) rng.uniform(0, 360);
            m_visibility.at(index) = false;
            m_position.at(index) = cv::Point2f(0.0f, 0.0f);
        }
        for ( auto t : m_visibility ) {
            std::cout << t;
        }
        std::cout << std::endl;

        for (ushort i = 0; i < MAX_ITERATION_THETA; i++) {

            if (i < (m_visibility.size() - 1) && m_visibility.at(i + (ushort) 1) == false) {
                m_visibility.at(i) = false;
            }
        }

        for ( auto t : m_visibility ) {
            std::cout << t;
        }

        std::cout << std::endl;
        for (ushort i = MAX_ITERATION_THETA; i > 0; i--) {

            if (i < (m_visibility.size() - 1) && m_visibility.at(i - (ushort) 1) == false) {
                m_visibility.at(i) = false;
            }
        }

        for ( auto t : m_visibility ) {
            std::cout << t;
        }
        std::cout << std::endl;
    }
};

class Achterbahn : public ObjectPixelPosition {

public:

    Achterbahn() {};

    void process(cv::Size frame_size) override ;

};

class Circle : public ObjectPixelPosition {

public:

    Circle() {};

    void process(cv::Size frame_size) override ;

};

class Ramp : public ObjectPixelPosition {

public:

    Ramp() {};

    void process(cv::Size frame_size) override ;

};

class NegativeRamp : public ObjectPixelPosition {

public:

    NegativeRamp() {};

    void process(cv::Size frame_size) override ;

};

class NoPosition: public ObjectPixelPosition {

public:

    NoPosition() {};

    void process(cv::Size frame_size) override;

};

class MyPosition: public ObjectPixelPosition {

public:

    MyPosition() {};

    void process(cv::Size frame_size) override;

    void pushPositionPoints(cv::Point2f points) override {
        m_position.push_back(points);
    }

    void pushVisibility(bool visibility) override{
        m_visibility.push_back(visibility);
    }

};


#endif //MAIN_OBJECTTRAJECTORY_H

