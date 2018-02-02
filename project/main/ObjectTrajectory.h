//
// Created by veikas on 25.01.18.
//

#ifndef MAIN_OBJECTTRAJECTORY_H
#define MAIN_OBJECTTRAJECTORY_H


#include "Dataset.h"
#include "ObjectImageShapeData.h"
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>


class ObjectTrajectory {

protected:
    std::vector<cv::Point2i> m_trajectory;

public:

    ObjectTrajectory() {};

    virtual void process(cv::Size framesize)  {};

    std::vector<cv::Point2i> get() {
        return m_trajectory;
    }

};

class Achterbahn : public ObjectTrajectory {

public:

    Achterbahn() {};

    void process(cv::Size framesize) override ;

};

class NoTrajectory: public ObjectTrajectory {

public:

    NoTrajectory() {};

    void process(cv::Size framesize) override {
        m_trajectory.push_back(cv::Point2i(0,0));
    };

};


class ObjectCollision {

};

class PlotCollision {

};

class PlotTrajectory {

};

#endif //MAIN_OBJECTTRAJECTORY_H

