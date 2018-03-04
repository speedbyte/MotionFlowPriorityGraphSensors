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


class ObjectTrajectory {

protected:
    std::vector<cv::Point2f> m_trajectory;
    std::vector<bool> m_visibility;

public:

    ObjectTrajectory() {
        for (ushort i = 0; i < MAX_ITERATION_THETA; i++) {
            m_visibility.push_back(false);
            m_trajectory.push_back(cv::Point2f(-1,-1));
        }
    };

    virtual void process(cv::Size frame_size) {};

    virtual void pushTrajectoryPoints(cv::Point2f points) {}

    virtual void pushVisibility(bool visibility) {}

    void atFrameNumber(ushort frameNumber, cv::Point2f points, bool visibility) {
        m_trajectory.at(frameNumber) = points;
        m_visibility.at(frameNumber) = visibility;
    }

    std::vector<cv::Point2f> getTrajectory() const {
        return m_trajectory;
    }

    std::vector<bool> getVisibility() const {
        return m_visibility;
    }

    void setDynamic() {

        cv::RNG rng(cv::getTickCount());
        for (ushort i = 0; i < MAX_ITERATION_THETA / 20; i++) {
            ushort index = (ushort) rng.uniform(0, 360);
            m_visibility.at(index) = false;
            m_trajectory.at(index) = cv::Point2f(0.0f, 0.0f);
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

class Achterbahn : public ObjectTrajectory {

public:

    Achterbahn() {};

    void process(cv::Size frame_size) override ;

};

class Circle : public ObjectTrajectory {

public:

    Circle() {};

    void process(cv::Size frame_size) override ;

};

class Ramp : public ObjectTrajectory {

public:

    Ramp() {};

    void process(cv::Size frame_size) override ;

};

class NegativeRamp : public ObjectTrajectory {

public:

    NegativeRamp() {};

    void process(cv::Size frame_size) override ;

};

class NoTrajectory: public ObjectTrajectory {

public:

    NoTrajectory() {};

    void process(cv::Size frame_size) override;

};

class MyTrajectory: public ObjectTrajectory {

public:

    MyTrajectory() {};

    void process(cv::Size frame_size) override;

    void pushTrajectoryPoints(cv::Point2f points) override {
        m_trajectory.push_back(points);
    }

    void pushVisibility(bool visibility) override{
        m_visibility.push_back(visibility);
    }

};


#endif //MAIN_OBJECTTRAJECTORY_H

