//
// Created by veikas on 13.04.18.
//

#ifndef MAIN_UTILS_H
#define MAIN_UTILS_H


class Utils {

public:
    static cv::Point3f translate_points(const cv::Point3f initial, const cv::Point3f delta) {

        cv::Matx34f translate_matrix_bbox = {
                1,0,0,delta.x,
                0,1,0,delta.y,
                0,0,1,delta.z
        };

        cv::Matx41f vecPoints = {initial.x, initial.y, initial.z, 1};

        cv::Matx31f final = translate_matrix_bbox*vecPoints;

        return cv::Point3f(final(0), final(1), final(2));
    }

};


#endif //MAIN_UTILS_H
