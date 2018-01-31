//
// Created by veikas on 28.01.18.
//

#ifndef MAIN_OBJECTFLOW_H
#define MAIN_OBJECTFLOW_H


#include <opencv2/core/types.hpp>
#include <opencv2/core/persistence.hpp>
#include <kitti/io_flow.h>
#include "Dataset.h"
#include "ObjectTrajectory.h"

class ObjectFlow {

private:
    std::vector<std::pair<cv::Point2i, cv::Point2i> > m_object_flowvector_with_coordinate_gt;
    std::vector<std::vector<std::pair<cv::Point2i, cv::Point2i> > >
            m_object_frame_skips_extended_flowvector_with_coordinate_gt;

public:

    /* This needs to know the shape of the object to extrapolate */
    void extrapolate_flowpoints( FlowImage &F_gt_write, cv::FileStorage fs, cv::Point2i pt, int width, int height, int
    xValue, int yValue, const Dataset &dataset);

    void generate_base_flow_vector(const Dataset &m_dataset, const ushort &start_point, const std::vector<cv::Point2i>
    &trajectory_points);

    void generate_extended_flow_vector(const Dataset &dataset, const int &max_skips, cv::Mat data);

    void storeData(const std::vector<cv::Point2f> &prev_pts, std::vector<cv::Point2f> &next_pts, const
    std::vector<uchar> status);

    void store_in_png(cv::FileStorage &fs, ushort &frame_count, cv::Size frame_size, std::string
    temp_result_flow_path);

    void store_in_yaml(cv::FileStorage &fs, const cv::Point2i &l_pixelposition, const cv::Point2i
    &l_pixelmovement );

    std::vector<std::vector<std::pair<cv::Point2i, cv::Point2i> > >  get() {
        return m_object_frame_skips_extended_flowvector_with_coordinate_gt;
    }

};


#endif //MAIN_OBJECTFLOW_H
