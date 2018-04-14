//
// Created by veikas on 28.01.18.
//

#ifndef MAIN_SENSORIMAGESHAPEDATA_H
#define MAIN_SENSORIMAGESHAPEDATA_H

#include <opencv2/core/mat.hpp>

class SensorImageShapeData {

protected:

    cv::Mat m_data;
    ushort m_objectWidth;
    ushort m_objectHeight;

public:

    SensorImageShapeData() {};

    SensorImageShapeData(ushort width, ushort height ) : m_objectWidth(width), m_objectHeight
            (height) {
        m_data.create(height, width, CV_8UC3);
    }
    virtual void process() {};

    ushort getWidth() {
        return m_objectWidth;
    }

    ushort getHeight() {
        return m_objectHeight;
    }

    cv::Mat get() {
        return m_data;
    }

    cv::Mat set() {
        m_data = cv::Scalar::all(0);
    }
};

class Rectangle :  public SensorImageShapeData {

private:

public:

    Rectangle(ushort width, ushort height) : SensorImageShapeData(width, height) {};

    void process() override ;

};

#endif //MAIN_SENSORIMAGESHAPEDATA_H
