//
// Created by veikas on 25.01.18.
//

#ifndef MAIN_OBJECTTRAJECTORY_H
#define MAIN_OBJECTTRAJECTORY_H


class ObjectShape {
public:
    cv::Mat createRectangle();
};


class ObjectFlow {

};

class ObjectTrajectory {

private:
    std::vector<std::pair<cv::Point2i, cv::Point2i> > m_flow_matrix_result;

    std::vector<cv::Point2i> m_trajectory_1;


public:

    ObjectTrajectory();

    void storeData(const std::vector<cv::Point2f> &prev_pts, std::vector<cv::Point2f> &next_pts, const
    std::vector<uchar> status);

    void store_in_png(cv::FileStorage &fs, ushort &frame_count, cv::Size frame_size, std::string
    temp_result_flow_path);

    void store_in_yaml(cv::FileStorage &fs, const cv::Point2i &l_pixelposition, const cv::Point2i
    &l_pixelmovement );

    std::vector<cv::Point2i> create_trajectory(cv::Size m_frame_size);

    std::vector<cv::Point2i> getTrajectoryPoints(void);

};

class ObjectCollision {

};

class PlotCollision {

};

class PlotTrajectory {

};

#endif //MAIN_OBJECTTRAJECTORY_H

