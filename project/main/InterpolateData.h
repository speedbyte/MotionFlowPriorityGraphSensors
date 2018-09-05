//
// Created by veikas on 05.09.18.
//

#ifndef MAIN_INTERPOLATEDATA_H
#define MAIN_INTERPOLATEDATA_H


#include <opencv2/core/types.hpp>

class InterpolateData {

public:
    void interpolateBackground(std::vector<std::pair<cv::Point2f, cv::Point2f> > &object_stencil_displacement, std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_stencil_disjoint_displacement);

};


#endif //MAIN_INTERPOLATEDATA_H
