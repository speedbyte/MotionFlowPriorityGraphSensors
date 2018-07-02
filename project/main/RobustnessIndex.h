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

};


class MagnitudeRobusntess : public RobustnessIndex {

public:
    MagnitudeRobusntess(cv::FileStorage &fs) : RobustnessIndex(fs) {};

};

class PixelRobustness : public RobustnessIndex {

private:
    std::vector<std::vector<std::vector<std::vector<OPTICAL_FLOW_EVALUATION_METRICS> > > > m_list_evaluation_data_multiframe;

public:

    PixelRobustness(cv::FileStorage &fs) : RobustnessIndex(fs) {};
    void generatePixelRobustness(ushort SENSOR_COUNT, const OpticalFlow &opticalFlow_gt, const OpticalFlow &opticalFlow_base_algo);
    void writeToYaml(ushort SENSOR_COUNT, const OpticalFlow &opticalFlow);

};

class VectorRobustness : public RobustnessIndex {

private:

    std::vector<std::vector<std::vector<std::vector<OPTICAL_FLOW_COLLISION_METRICS> > > > m_list_collision_data_multiframe;

public:

    VectorRobustness(cv::FileStorage &fs) : RobustnessIndex(fs) {};
    void generateVectorRobustness(ushort SENSOR_COUNT, const OpticalFlow &opticalFlow_gt, const OpticalFlow &opticalFlow_base_algo);
    void writeToYaml(ushort SENSOR_COUNT, const OpticalFlow &opticalFlow);


};


class SensorFusionRobustness : public RobustnessIndex {

public:

    SensorFusionRobustness(cv::FileStorage &fs) : RobustnessIndex(fs) {};

};


#endif //MAIN_ROBUSTNESSINDEX_H
