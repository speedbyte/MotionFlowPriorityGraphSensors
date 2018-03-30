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

    GroundTruthFlow( std::vector<Objects *> &list_gt_objects ) :
            OpticalFlow(list_gt_objects) {

    }

    void generate_flow_frame();

    void generate_shape_points();

    void generate_edge_contour();

    ~GroundTruthFlow(){
        std::cout << "killing previous GroundTruthFlow object\n" ;
    }

private:

    void prepare_directories();



};


#endif //MAIN_GROUNDTRUTH_FLOW_H
