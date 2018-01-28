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
#include "ObjectProperties.h"

class ObjectFlow {

private:
    Dataset m_dataset;
    std::vector<std::vector<std::pair<cv::Point2i, cv::Point2i> > > m_scene_flow_vector_with_coordinate_gt;

public:
    ObjectFlow(Dataset dataset) : m_dataset(dataset) {}

    /* This needs to know the shape of the object to extrapolate */
    void extrapolate_flowpoints( FlowImage &F_gt_write, cv::FileStorage fs, cv::Point2i pt, int width, int height, int
    xValue, int yValue);

    void generate_base_flow_vector(std::vector<ObjectProperties> list_objects);

    void generate_extended_flow_vector(const int &max_skips, std::vector<ObjectProperties> list_objects);
};


#endif //MAIN_OBJECTFLOW_H
