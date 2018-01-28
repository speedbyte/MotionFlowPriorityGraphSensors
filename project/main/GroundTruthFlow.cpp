

#include <vector>
#include <cmath>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>
#include <map>
#include <chrono>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/tuple/tuple.hpp>
#include <png++/png.hpp>

#include "kitti/log_colormap.h"
#include <kitti/mail.h>
#include <kitti/io_flow.h>
#include <vires/vires_common.h>

#include "datasets.h"
#include "GroundTruthFlow.h"
#include "kbhit.h"

#include <unordered_map>
#include <bits/unordered_map.h>

#include "ObjectTrajectory.h"
#include "Dataset.h"
#include "GroundTruthScene.h"
#include "ObjectFlow.h"


//Creating a movement path. The path is stored in a x and y vector

using namespace std::chrono;


void GroundTruthFlow::prepare_directories() {

    char char_dir_append[20];

    if (!m_dataset.getBasePath().compare(CPP_DATASET_PATH) || !m_dataset.getBasePath().compare(VIRES_DATASET_PATH) ) {

        std::cout << "Creating GT Flow directories" << std::endl;
        // create flow directories
        for (int i = 1; i < 10; ++i) {
            sprintf(char_dir_append, "%02d", i);
            boost::filesystem::create_directories(m_dataset.getInputPath().string() + "/flow_occ_" + char_dir_append);
        }
        std::cout << "Ending GT Flow directories" << std::endl;
    }
}


void GroundTruthFlow::generate_gt_flow(void) {

    prepare_directories();

    /*
     * First create an object with an shape
     * Then define the object trajectory
     * Then copy the object shape on the object trajectory points
     * Then store the image in the ground truth image folder
     *
     * Then extrapolate the object trajectory with the above shape
     *
     * Then store the flow information in the flow folder
     */

    cv::Mat tempGroundTruthImage;
    tempGroundTruthImage.create(m_dataset.getFrameSize(), CV_8UC3);
    assert(tempGroundTruthImage.channels() == 3);

    const ushort start=60;

    std::map<std::string, double> time_map = {{"generate",0}, {"ground truth", 0}};

    auto tic = steady_clock::now();
    auto toc = steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();

    //Initialization
    //for moving the objects later

    //std::vector<cv::Point2i> shapes = m_gt_scene.getListObjectShapes();
    ObjectTrajectory trajectory_points = m_gt_scene.getListObjectTrajectories();

    ObjectFlow point(m_dataset);
    // create the base flow vector
    point.generate_base_flow_vector(trajectory_points, start);

    // extend the flow vectors by skipping frames and then storing them as pngs
    point.generate_extended_flow_vector(MAX_SKIPS);

    // plotVectorField (F_gt_write,m_base_directory_path_image_out.parent_path().string(),file_name);
    toc_all = steady_clock::now();
    time_map["ground truth"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth flow generation time - " << time_map["ground truth"]  << "ms" << std::endl;

}


void GroundTruthFlow::plot(std::string resultsordner) {

    PlotFlow::plot(m_dataset, resultsordner);

}

