//
// Created by veikas on 25.01.18.
//

#ifndef MAIN_OBJECTTRAJECTORY_H
#define MAIN_OBJECTTRAJECTORY_H


#include "Dataset.h"
#include "ObjectShape.h"
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>


class ObjectTrajectory {

private:
    std::vector<std::pair<cv::Point2i, cv::Point2i> > m_flow_matrix_result;

protected:
    std::vector<cv::Point2i> m_trajectory;

public:

    ObjectTrajectory() {};

    void storeData(const std::vector<cv::Point2f> &prev_pts, std::vector<cv::Point2f> &next_pts, const
    std::vector<uchar> status);

    void store_in_png(cv::FileStorage &fs, ushort &frame_count, cv::Size frame_size, std::string
    temp_result_flow_path);

    void store_in_yaml(cv::FileStorage &fs, const cv::Point2i &l_pixelposition, const cv::Point2i
    &l_pixelmovement );

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


class ObjectCollision {

};

class PlotCollision {

};

class PlotTrajectory {

};

#endif //MAIN_OBJECTTRAJECTORY_H

