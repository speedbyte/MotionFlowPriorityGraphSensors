//
// Created by veikas on 28.01.18.
//

#ifndef MAIN_OBJECTFLOW_H
#define MAIN_OBJECTFLOW_H


#include <opencv2/core/types.hpp>
#include <opencv2/core/persistence.hpp>
#include "Dataset.h"
#include "ObjectTrajectory.h"


class ObjectFlow {

private:
    std::vector<std::pair<cv::Point2i, cv::Point2i> > m_obj_flow_vector_baseframe;
    //std::vector<std::pair<cv::Point2i, cv::Point2i> > m_obj_flow_vector_baseframe;
    std::vector<std::vector<std::pair<cv::Point2i, cv::Point2i> > > m_obj_flow_vector_multiframe;

public:

    /* This needs to know the shape of the object to extrapolate */
    void generate_baseframe_flow_vector(const ushort &start_point, const std::vector<cv::Point2i>
    &trajectory_points);

    void generate_multiframe_flow_vector(const int &max_skips);

    void storeData(const std::vector<cv::Point2f> &prev_pts, std::vector<cv::Point2f> &next_pts, const
    std::vector<uchar> status);

    void store_in_png(cv::FileStorage &fs, ushort &frame_count, cv::Size frame_size, std::string
    temp_result_flow_path);

    void store_in_yaml(cv::FileStorage &fs, const cv::Point2i &l_pixelposition, const cv::Point2i
    &l_pixelmovement );

    std::vector<std::vector<std::pair<cv::Point2i, cv::Point2i> > >  get() {
        return m_obj_flow_vector_multiframe;
    }

};


#endif //MAIN_OBJECTFLOW_H
