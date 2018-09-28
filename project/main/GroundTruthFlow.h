//
// Created by veikas on 11.01.18.
//

#ifndef MAIN_GROUNDTRUTHFLOW_H
#define MAIN_GROUNDTRUTHFLOW_H


#include "Dataset.h"
#include "PlotFlow.h"
#include "GenerateGroundTruthScene.h"
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

    GroundTruthFlow( std::vector<ushort> evaluation_list, std::string noise, std::vector<Sensors> &list_of_gt_sensors_base, std::vector<GroundTruthObjects *> &list_gt_objects ) :
            OpticalFlow(evaluation_list, noise, "ground_truth", list_of_gt_sensors_base, list_gt_objects, 0 ) {
    }

    ~GroundTruthFlow(){
        std::cout << "killing previous GroundTruthFlow object\n" ;
    }

    void prepare_groundtruth_flow_directories(std::string noise, ushort fps, ushort stepSize);

    void generate_flow_vector();

    void find_ground_truth_object_special_region_of_interest();

    void find_ground_truth_object_contour_region_of_interest();

    void save_ground_truth_object_contour_region_of_interest();
};


#endif //MAIN_GROUNDTRUTHFLOW_H
