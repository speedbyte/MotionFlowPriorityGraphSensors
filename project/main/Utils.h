//
// Created by veikas on 13.04.18.
//

#ifndef MAIN_UTILS_H
#define MAIN_UTILS_H


class Utils {

public:
    static cv::Point3f translate_and_rotate_points(const cv::Point3f initial, const cv::Point3f delta, const cv::Point3f orientation) {

        cv::Matx34f translate_matrix = {
                1,0,0,delta.x,
                0,1,0,delta.y,
                0,0,1,delta.z
        };

        float h_mat = orientation.x;
        float p_mat = orientation.y;
        float r_mat = orientation.z;

        cv::Matx33f rot_matrix = {
                cos(p_mat)*cos(h_mat) ,  -cos(r_mat)*sin(h_mat) + sin(r_mat)*sin(p_mat)*cos(h_mat),   sin(r_mat)*sin(h_mat) + cos(r_mat)*sin(p_mat)*cos(h_mat),
                cos(p_mat)*sin(h_mat) ,  cos(r_mat)*cos(h_mat) + sin(r_mat)*sin(p_mat)*sin(h_mat) ,  -sin(r_mat)*cos(h_mat) + cos(r_mat)*sin(p_mat)*sin(h_mat),
                -sin(p_mat)           ,  sin(r_mat)*cos(p_mat)                                    ,   cos(r_mat)*cos(p_mat)};

        cv::Matx41f vecPoints_1 = {initial.x, initial.y, initial.z, 1};
        cv::Matx31f translate_1 = translate_matrix * vecPoints_1;
        cv::Matx31f rotated_1 = rot_matrix * translate_1;

        cv::Matx31f vecPoints_2 = {initial.x, initial.y, initial.z};
        cv::Matx31f rotated_2 = rot_matrix * vecPoints_2;
        cv::Matx41f rotated = {rotated_2(0), rotated_2(1), rotated_2(2), 1};
        cv::Matx31f translate_2 = translate_matrix * rotated;

        //assert(rotated_1==translate_2);

        cv::Matx31f final = translate_2;

        return cv::Point3f(final(0), final(1), final(2));
    }


};


#endif //MAIN_UTILS_H
