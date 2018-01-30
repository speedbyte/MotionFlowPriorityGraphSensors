//
// Created by veikas on 11.01.18.
//

#ifndef MAIN_GROUNDTRUTH_FLOW_H
#define MAIN_GROUNDTRUTH_FLOW_H


#include "Dataset.h"
#include "PlotFlow.h"
#include "ObjectTrajectory.h"
#include "GroundTruthScene.h"
#include "ObjectFlow.h"

/**
 * This class
 *  private - makes directories where synthetic images will be stored
 *  produces synthetic images
 *  private - makes directories where flow images will be stored
 *  produces flow images
 */

class GroundTruthFlow : public PlotFlow {

private:

    Dataset &m_dataset;
    std::vector<std::vector<std::pair<cv::Point2i, cv::Point2i> > > m_scene_flow_vector_with_coordinate_gt;


public:

    GroundTruthFlow( Dataset &dataset ) : m_dataset(dataset) {}

    void generate_gt_scene_flow_vector(std::vector<Objects> list_of_objects);

    void plot(std::string resultsordner);

    ~GroundTruthFlow(){
        std::cout << "killing previous GroundTruthFlow object\n" ;
    }


private:

    void prepare_directories();


    void store_in_yaml(cv::FileStorage &fs, const cv::Point2i &l_pixelposition, const cv::Point2i
            &l_pixelmovement);
};


#endif //MAIN_GROUNDTRUTH_FLOW_H
