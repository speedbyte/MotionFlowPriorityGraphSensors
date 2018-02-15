//
// Created by veikas on 11.01.18.
//

#ifndef MAIN_GROUNDTRUTH_FLOW_H
#define MAIN_GROUNDTRUTH_FLOW_H


#include "Dataset.h"
#include "PlotFlow.h"
#include "ObjectTrajectory.h"
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

    std::vector<std::pair<GroundTruthObjects, GroundTruthObjects> > m_list_objects_combination;

    std::vector<GroundTruthObjects> &m_list_gt_objects;

    std::vector<Objects*> m_list_gt_objects_ptr;



public:

    GroundTruthFlow( std::vector<GroundTruthObjects> &list_objects ) : m_list_gt_objects(list_objects) {

        for ( unsigned i = 0; i < m_list_gt_objects.size(); i++ ) {
            m_list_gt_objects_ptr.push_back(&m_list_gt_objects.at(i));
        }

    }

    void generate_flow_frame();

    void make_video_from_png(const Dataset &dataset_path, std::string unterordner);

    ~GroundTruthFlow(){
        std::cout << "killing previous GroundTruthFlow object\n" ;
    }

    void generate_collision_points() {
        OpticalFlow::generate_collision_points(m_list_gt_objects_ptr);
    };


private:

    void prepare_directories();

};


#endif //MAIN_GROUNDTRUTH_FLOW_H
