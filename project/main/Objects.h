//
// Created by veikas on 08.02.18.
//

#ifndef MAIN_OBJECTS_H
#define MAIN_OBJECTS_H


#include <opencv2/core/types.hpp>

class Objects {

public:

    virtual unsigned getObjectId() const = 0;

    virtual std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
    get_obj_extrapolated_pixel_centroid_pixel_displacement_mean() const = 0;

    virtual int getWidth() const = 0;

    virtual int getHeight() const = 0;

    virtual std::vector<std::vector<bool> >  get_obj_extrapolated_visibility() const = 0;

    virtual std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >  get_line_parameters() const = 0;

};


#endif //MAIN_OBJECTS_H
