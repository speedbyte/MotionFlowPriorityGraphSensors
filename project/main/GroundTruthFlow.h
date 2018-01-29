//
// Created by veikas on 11.01.18.
//

#ifndef MAIN_GROUNDTRUTH_FLOW_H
#define MAIN_GROUNDTRUTH_FLOW_H


#include "Dataset.h"
#include "PlotFlow.h"
#include "ObjectTrajectory.h"
#include "GroundTruthScene.h"

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
    std::vector<ObjectProperties> &m_list_objects;


public:

    GroundTruthFlow(Dataset &dataset, std::vector<ObjectProperties> &list_objects) : m_dataset(dataset), m_list_objects
            (list_objects) {}

    void generate_gt_flow();

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
