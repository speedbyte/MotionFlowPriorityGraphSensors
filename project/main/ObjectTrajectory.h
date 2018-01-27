//
// Created by veikas on 25.01.18.
//

#ifndef MAIN_OBJECTTRAJECTORY_H
#define MAIN_OBJECTTRAJECTORY_H


#include "Dataset.h"

class ObjectShape {

protected:

    cv::Mat m_shape;

public:

    virtual void process() {};

    cv::Mat get() {
        return m_shape;
    }

};

class Rectangle :  public ObjectShape {

public:

    void process() override ;

};


class ObjectTrajectory {

private:
    std::vector<std::pair<cv::Point2i, cv::Point2i> > m_flow_matrix_result;

protected:
    std::vector<cv::Point2i> m_trajectory;

public:

    ObjectTrajectory() {};

    void storeData(const std::vector<cv::Point2f> &prev_pts, std::vector<cv::Point2f> &next_pts, const
    std::vector<uchar> status);

    void store_in_png(cv::FileStorage &fs, ushort &frame_count, cv::Size frame_size, std::string
    temp_result_flow_path);

    void store_in_yaml(cv::FileStorage &fs, const cv::Point2i &l_pixelposition, const cv::Point2i
    &l_pixelmovement );

    virtual void process(cv::Size framesize)  {};

    std::vector<cv::Point2i> get() {
        return m_trajectory;
    }

};

class Achterbahn : public ObjectTrajectory {

public:

    Achterbahn() {};

    void process(cv::Size framesize) override ;

};

class ObjectProperties {

private:
    ObjectShape &obj_shape;
    ObjectTrajectory &obj_trajectory;

public:

    ObjectProperties( Dataset dataset, ObjectShape &shape, ObjectTrajectory &trajectory) : obj_shape ( shape ),
    obj_trajectory (
            trajectory) {
        shape.process();
        trajectory.process(dataset.getFrameSize());
    }

    cv::Mat getShape() {
        return obj_shape.get();
    }

    std::vector<cv::Point2i> getTrajectoryPoints() {
        return obj_trajectory.get();
    }

};


class ObjectFlow {

private:
    Dataset m_dataset;
    std::vector<std::pair<cv::Point2i, cv::Point2i> > m_flowvector_with_coordinate_gt;

public:
    ObjectFlow(Dataset dataset) : m_dataset(dataset) {}

    void extrapolate_flowpoints( cv::FileStorage fs, cv::Point2i pt, ushort width, ushort height, int xValue, int
    yValue, std::string image_path);

    void generate_base_flow_vector(std::vector<cv::Point2i> trajectory_points, const ushort start);

    void generate_extended_flow_vector(const int &max_skips);

    std::vector<std::pair<cv::Point2i, cv::Point2i> >  get_base_flow_vector() {
        return m_flowvector_with_coordinate_gt;
    }
};


class ObjectCollision {

};

class PlotCollision {

};

class PlotTrajectory {

};

#endif //MAIN_OBJECTTRAJECTORY_H

