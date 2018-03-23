//
// Created by veikas on 02.02.18.
//

#ifndef MAIN_ROBUSTNESSINDEX_H
#define MAIN_ROBUSTNESSINDEX_H


#include "OpticalFlow.h"

class RobustnessIndex {

protected:

    cv::FileStorage &m_fs;

public:

    RobustnessIndex(cv::FileStorage &fs): m_fs(fs) {}

    void make_video_from_png(const std::string &videoOrdner);

};


class MagnitudeRobusntess : public RobustnessIndex {

public:
    MagnitudeRobusntess(cv::FileStorage &fs) : RobustnessIndex(fs) {};

};

class PixelRobustness : public RobustnessIndex {

public:

    PixelRobustness(cv::FileStorage &fs) : RobustnessIndex(fs) {};
    void generatePixelRobustness(const OpticalFlow &opticalFlow_gt, const OpticalFlow &opticalFlow_base_algo);

private:
    void generateFrameJaccardIndex(const OpticalFlow &opticalFlow);

};

class VectorRobustness : public RobustnessIndex {

public:

    VectorRobustness(cv::FileStorage &fs) : RobustnessIndex(fs) {};
    void generateVectorRobustness(const OpticalFlow &opticalFlow_gt, const OpticalFlow &opticalFlow_base_algo);


private:

    void generateFrameVectorSignature(const OpticalFlow &opticalFlow);
    void fitLineForCollisionPoints(const cv::Mat_<float> &samples_xy, std::string &list_gp_lines );


};

#endif //MAIN_ROBUSTNESSINDEX_H
