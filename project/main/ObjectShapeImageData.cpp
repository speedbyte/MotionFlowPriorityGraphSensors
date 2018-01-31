//
// Created by veikas on 28.01.18.
//

#include "ObjectShapeImageData.h"
#include "ObjectTrajectory.h"


void Rectangle::process() {

    m_data.create(m_objectHeight, m_objectWidth, CV_8UC3);

    uchar r = 0;
    uchar b = 0;

    r = 0;
    b = 0;
    for (int k = 0; k < (m_objectHeight - 1); k++) {
        for (int j = 0; j < (m_objectWidth -1 ); j++) {
            m_data.at<cv::Vec3b>(k, j)[0] = b;
            m_data.at<cv::Vec3b>(k, j)[1] = 0;
            m_data.at<cv::Vec3b>(k, j)[2] = r;
            r = r + (uchar)2;
            b = b + (uchar)2;
            if (r > 254)
                r = 130;
        }
        if (b > 254)
            b = 46;
    }

}
