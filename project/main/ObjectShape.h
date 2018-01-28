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

    virtual ushort getWidth() {};

    virtual ushort getHeight() {};

    cv::Mat get() {
        return m_shape;
    }

};

class Rectangle :  public ObjectShape {

private:
    ushort m_objectWidth;
    ushort m_objectHeight;

public:

    Rectangle(ushort width, ushort height) : m_objectWidth(width), m_objectHeight(height) {};

    void process() override ;

    ushort getWidth() {
        return m_objectWidth;
    }

    ushort getHeight() {
        return m_objectHeight;
    }
};


#endif //MAIN_OBJECTSHAPE_H
