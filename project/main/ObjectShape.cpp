//
// Created by veikas on 28.01.18.
//

#include "ObjectShape.h"
#include "ObjectTrajectory.h"
#include "datasets.h"
#include <opencv2/core.hpp>


void Rectangle::process() {

    m_shape.create(object_height, object_width, CV_8UC3);

    uchar r = 0;
    uchar b = 0;

    r = 0;
    b = 0;
    for (int k = 0; k < (object_height - 1); k++) {
        for (int j = 0; j < (object_width -1 ); j++) {
            m_shape.at<cv::Vec3b>(k, j)[0] = b;
            m_shape.at<cv::Vec3b>(k, j)[1] = 0;
            m_shape.at<cv::Vec3b>(k, j)[2] = r;
            r = r + (uchar)2;
            b = b + (uchar)2;
            if (r > 254)
                r = 130;
        }
        if (b > 254)
            b = 46;
    }

}
