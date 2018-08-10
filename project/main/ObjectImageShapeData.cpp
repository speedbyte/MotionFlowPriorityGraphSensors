//
// Created by veikas on 28.01.18.
//

#include "ObjectImageShapeData.h"


void Rectangle::process() {

    //m_data_image.create(m_objectHeight, m_objectWidth, CV_8UC3);
    //m_data_depth.create(m_objectHeight, m_objectWidth, CV_8UC1);

}

void Canvas::process() {

    //m_data_image.create(m_objectHeight, m_objectWidth, CV_8UC3);

}

void Circle::construct(ushort radius, std::unique_ptr<Noise> &noise, ushort depth)  {
    m_data_image.create(radius, radius, CV_8UC3);
    m_data_image.setTo(cv::Scalar(255,255,255));
    m_data_depth.create(radius, radius, CV_8UC1);
    cv::circle(m_data_image, cv::Point(m_objectRadius, m_objectRadius), (m_objectRadius-5), cv::Scalar(255,0,0), CV_FILLED);
    applyNoise(noise);
    applyDepth(depth);

}
void Circle::process() {

    //m_data_image.create(m_objectHeight, m_objectWidth, CV_8UC3);
    //m_data_depth.create(m_objectHeight, m_objectWidth, CV_8UC1);

}
