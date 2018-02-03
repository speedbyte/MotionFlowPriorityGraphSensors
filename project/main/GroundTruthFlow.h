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

/**
 * This class
 *  private - makes directories where synthetic images will be stored
 *  produces synthetic images
 *  private - makes directories where flow images will be stored
 *  produces flow images
 */

class GroundTruthFlow {

private:


    std::vector<Objects> &m_list_objects;

public:

    GroundTruthFlow( std::vector<Objects> &list_objects ) : m_list_objects
            (list_objects) {}

    void extrapolate_flowpoints( std::string temp_gt_flow_image_path, unsigned frame_skip, unsigned
    frame_count, std::vector<Objects> list_objects);

    void generate_gt_scenepixel_displacement();

    void make_video_from_png(const Dataset &dataset_path, std::string unterordner);

    ~GroundTruthFlow(){
        std::cout << "killing previous GroundTruthFlow object\n" ;
    }

private:

    void prepare_directories();

    void calcCovarMatrix();

    void common(cv::Mat_<uchar> &samples_xy, std::vector<std::string> &list_gp_lines );


    };


#endif //MAIN_GROUNDTRUTH_FLOW_H
