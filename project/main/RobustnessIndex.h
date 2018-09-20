//
// Created by veikas on 02.02.18.
//

#ifndef MAIN_ROBUSTNESSINDEX_H
#define MAIN_ROBUSTNESSINDEX_H


#include "OpticalFlow.h"

class RobustnessIndex {

protected:

public:

    RobustnessIndex() {}

};


class MagnitudeRobusntess : public RobustnessIndex {

public:
    MagnitudeRobusntess() : RobustnessIndex() {};

};

class PixelRobustness : public RobustnessIndex {

private:
    std::vector<std::vector<std::vector<std::vector<OPTICAL_FLOW_EVALUATION_METRICS> > > > m_list_evaluation_data_multiframe;

public:

    PixelRobustness() : RobustnessIndex() {};
    void generatePixelRobustness(const OpticalFlow &opticalFlow_gt, const OpticalFlow &opticalFlow_base_algo, cv::FileStorage &fs);
    void writeToYaml(const OpticalFlow &opticalFlow, cv::FileStorage &fs);

};

class VectorRobustness : public RobustnessIndex {

private:

    std::vector<std::vector<std::vector<std::vector<OPTICAL_FLOW_COLLISION_METRICS> > > > m_list_collision_data_multiframe;

public:

    VectorRobustness() : RobustnessIndex() {};
    void generateVectorRobustness(const OpticalFlow &opticalFlow_gt, const OpticalFlow &opticalFlow_base_algo, cv::FileStorage &fs);
    void writeToYaml(const OpticalFlow &opticalFlow, cv::FileStorage &fs);


};


class SensorFusionRobustness : public RobustnessIndex {

public:

    SensorFusionRobustness() : RobustnessIndex() {};

};


#endif //MAIN_ROBUSTNESSINDEX_H
