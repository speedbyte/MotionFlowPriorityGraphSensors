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

    GroundTruthFlow( std::vector<ushort> evaluation_list, std::string weather, std::vector<Objects *> &list_gt_objects, std::vector<std::unique_ptr<Objects>> &list_simulated_objects_base, std::vector<Objects *> &list_gt_objects_base ) :
            OpticalFlow(evaluation_list, weather, "ground_truth", list_gt_objects, list_simulated_objects_base, list_gt_objects_base, 0) {
    }

    void generate_edge_images(ushort SENSOR_COUNT);
    void generate_depth_images(ushort SENSOR_COUNT);

    ~GroundTruthFlow(){
        std::cout << "killing previous GroundTruthFlow object\n" ;
    }

    void CannyEdgeDetection(std::string temp_result_flow_path, std::string temp_result_edge_path);

    void prepare_directories(ushort SENSOR_COUNT, std::string noise, ushort fps, ushort stepSize) override;



};


#endif //MAIN_GROUNDTRUTH_FLOW_H
