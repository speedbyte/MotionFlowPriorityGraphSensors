//
// Created by veikas on 02.02.18.
//

#ifndef MAIN_ROBUSTNESSINDEX_H
#define MAIN_ROBUSTNESSINDEX_H


#include "OpticalFlow.h"

class RobustnessIndex {

public:

};


class PixelRobustness : public RobustnessIndex {

public:
    void generatePixelRobustness(const std::string &resultOrdner);

};

class VectorRobustness : public RobustnessIndex {

public:

    void generateVectorRobustness(const OpticalFlow &opticalFlow);

private:

    void calcCovarMatrix(const OpticalFlow &opticalFlow);
    void fitLineForCollisionPoints(cv::Mat_<float> &samples_xy, std::vector<std::string> &list_gp_lines );


};

#endif //MAIN_ROBUSTNESSINDEX_H
