//
// Created by veikas on 28.01.18.
//

#ifndef MAIN_OBJECTSHAPE_H
#define MAIN_OBJECTSHAPE_H

#include <opencv2/core/mat.hpp>

class ObjectShape {

protected:

    cv::Mat m_shape;

public:

    virtual void process() {};

    cv::Mat get() {
        return m_shape;
    }

};

class Rectangle :  public ObjectShape {

public:

    void process() override ;

};


#endif //MAIN_OBJECTSHAPE_H
