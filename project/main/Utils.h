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

        cv::Matx31f vecPoints = {initial.x, initial.y, initial.z};
        cv::Matx31f rotated = rot_matrix * vecPoints;
        cv::Matx41f rotated_extend = {rotated(0), rotated(1), rotated(2), 1};
        cv::Matx31f translate = translate_matrix * rotated_extend;

        cv::Matx31f final = translate;

        return cv::Point3f(final(0), final(1), final(2));
    }


    static cv::Point2f worldToCamera(cv::Point3f final, float fov_rad, float fx, float fy) {

        //transform to VTD coordinates, x = depth, y = width, z = height. Hence the camera co-ordinates needs to be changed similarly.
        cv::Point3f pos = cv::Point3f(-final.y, -final.z, final.x);

        float fx_x =  pos.x * fx /pos.z;
        float fy_y =  pos.y * fy /pos.z;

        float distToImagePlane = 0.5 * Dataset::getFrameSize().height / tan(fov_rad/ 2); // [px] from camera position.
        float pxSize = 2.2e-6; // [m/px]
        //scale 3D point back onto image. row and column are calculated from the optical center.


        float x = pos.x * ((distToImagePlane) / pos.z);
        float y = pos.y * ((distToImagePlane) / pos.z);

        // Change from optical axis to origin ( top, left )
        float x_image =  Dataset::getFrameSize().width/2 + x;
        float y_image =  Dataset::getFrameSize().height/2 + y;

        return cv::Point2f(x_image, y_image);

    }


};



#endif //MAIN_UTILS_H
