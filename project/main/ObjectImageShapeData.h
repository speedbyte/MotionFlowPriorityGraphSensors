//
// Created by veikas on 28.01.18.
//

#ifndef MAIN_OBJECTSHAPE_H
#define MAIN_OBJECTSHAPE_H

#include <opencv2/core/mat.hpp>

class ObjectImageShapeData {

protected:

    cv::Mat m_data;

public:

    ObjectImageShapeData(int width, int height ) {
        m_data.create(height, width, CV_8UC3);
    }
    virtual void process() {};

    cv::Mat get() {
        return m_data;
    }

    cv::Mat set() {
        m_data = cv::Scalar::all(0);
    }
};

class Rectangle :  public ObjectImageShapeData {

private:
    ushort m_objectWidth;
    ushort m_objectHeight;

public:

    Rectangle(ushort width, ushort height) : ObjectImageShapeData(width, height), m_objectWidth(width), m_objectHeight
            (height) {};

    void process() override ;

    ushort getWidth() {
        return m_objectWidth;
    }

    ushort getHeight() {
        return m_objectHeight;
    }
};

#endif //MAIN_OBJECTSHAPE_H
