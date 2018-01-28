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
    Dataset m_dataset;
    std::vector<std::pair<cv::Point2i, cv::Point2i> > m_flowvector_with_coordinate_gt;

public:
    ObjectFlow(Dataset dataset) : m_dataset(dataset) {}

    void extrapolate_flowpoints( cv::FileStorage fs, cv::Point2i pt, ushort width, ushort height, int xValue, int
    yValue, std::string image_path);

    void generate_base_flow_vector(ObjectTrajectory trajectory_points, const ushort start);

    void generate_extended_flow_vector(const int &max_skips);

    std::vector<std::pair<cv::Point2i, cv::Point2i> >  get_base_flow_vector() {
        return m_flowvector_with_coordinate_gt;
    }
};


#endif //MAIN_OBJECTFLOW_H
