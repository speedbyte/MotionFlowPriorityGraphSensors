//
// Created by veikas on 11.01.18.
//

#ifndef MAIN_PREPAREGROUNDTRUTH_H
#define MAIN_PREPAREGROUNDTRUTH_H


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


class PrepareGroundTruth : public OpticalFlow {

private:


public:

    PrepareGroundTruth( std::vector<ushort> evaluation_list, std::string weather, std::vector<GroundTruthObjects *> &list_gt_objects, std::vector<Objects*> &list_simulated_objects_base, std::vector<Objects*> &list_simulated_objects ) :
            OpticalFlow(evaluation_list, weather, "ground_truth", list_gt_objects, list_simulated_objects_base, list_simulated_objects, 0) {
    }

    ~PrepareGroundTruth(){
        std::cout << "killing previous PrepareGroundTruth object\n" ;
    }

    void prepare_directories(ushort SENSOR_COUNT, std::string noise, ushort fps, ushort stepSize) override;

    void generate_flow_vector(ushort SENSOR_COUNT);

    void find_ground_truth_object_special_region_of_interest(ushort SENSOR_COUNT);

};


#endif //MAIN_PREPAREGROUNDTRUTH_H
