//
// Created by veikas on 11.01.18.
//

#ifndef MAIN_GROUNDTRUTH_FLOW_H
#define MAIN_GROUNDTRUTH_FLOW_H


#include "Dataset.h"
#include "PlotFlow.h"
#include "GroundTruthScene.h"
#include "FlowImageExtended.h"
#include "OpticalFlow.h"

/**
 * This class
 *  private - makes directories where synthetic images will be stored
 *  produces synthetic images
 *  private - makes directories where flow images will be stored
 *  produces flow images
 */


class GroundTruthFlow : public OpticalFlow {

private:


public:

    GroundTruthFlow( std::vector<Objects *> &list_gt_objects, std::vector<Objects *> &list_simulated_objects, std::vector<Objects *> &list_simulated_objects_base ) :
            OpticalFlow("ground_truth", list_gt_objects, list_simulated_objects, list_simulated_objects_base, 0) {
    }

    void generate_edge_images();

    ~GroundTruthFlow(){
        std::cout << "killing previous GroundTruthFlow object\n" ;
    }

    void CannyEdgeDetection(std::string temp_result_flow_path, std::string temp_result_edge_path);

    void prepare_directories(std::string noise, ushort fps, ushort stepSize) override;



};


#endif //MAIN_GROUNDTRUTH_FLOW_H
