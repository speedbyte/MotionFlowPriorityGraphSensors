

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
            // delete ground truth image and ground truth flow directories
            sprintf(char_dir_append, "%02d", i);
            boost::filesystem::path path = m_dataset.getGroundTruthFlowPath().string() + "/flow_occ_" + char_dir_append;
            if (boost::filesystem::exists(path)) {
                system(("rm -rf " + path.string()).c_str());
            }
            boost::filesystem::create_directories(path);
        }
        std::cout << "Ending GT Flow directories" << std::endl;
    }
}


void GroundTruthFlow::generate_gt_scene_flow_vector(std::vector<Objects> list_of_objects) {

    prepare_directories();

    cv::Mat tempGroundTruthImage;
    tempGroundTruthImage.create(m_dataset.getFrameSize(), CV_8UC3);
    assert(tempGroundTruthImage.channels() == 3);


    std::map<std::string, double> time_map = {{"generate",0}, {"ground truth", 0}};

    auto tic = steady_clock::now();
    auto toc = steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();


    std::cout << "ground truth flow will be stored in " << m_dataset.getGroundTruthFlowPath().string() << std::endl;

    for ( int i = 0; i < list_of_objects.size(); i++ ) {
        list_of_objects.at(i).generate_base_flow_vector();
        // extend the flow vectors by skipping frames and then storing them as pngs
        list_of_objects.at(i).generate_extended_flow_vector();
        m_scene_flow_vector_with_coordinate_gt.push_back(list_of_objects.at(i).getFlowPoints().get()[0]);
    }


    char folder_name_flow[50];
    cv::FileStorage fs;
    fs.open(m_dataset.getGroundTruthFlowPath().string() + "/" + folder_name_flow + "/" + "gt_flow.yaml",
            cv::FileStorage::WRITE);
    std::vector<std::vector<std::pair<cv::Point2i, cv::Point2i> > > objects;



    for ( int frame_skip = 1; frame_skip < list_of_objects.at(0).getFlowPoints().get().size() ;
          frame_skip++ ) {

        sprintf(folder_name_flow, "flow_occ_%02d", frame_skip);

        std::cout << "saving flow files for frame_skip " << frame_skip << std::endl;

        for (ushort frame_count = 0; frame_count < list_of_objects.at(0).getFlowPoints().get().at
                (frame_skip).size(); frame_count++) {

            /* What is the shape of the object ? */
            FlowImage F_gt_write(m_dataset.getFrameSize().width, m_dataset.getFrameSize().height);

            char file_name_image[50];
            sprintf(file_name_image, "000%03d_10.png", frame_count);
            std::string temp_gt_flow_image_path = m_dataset.getGroundTruthFlowPath().string() + "/" + folder_name_flow + "/"
                                                  + file_name_image;

            fs << "frame_count" << frame_count;
            for ( int i = 0; i < list_of_objects.size(); i++ ) {
                list_of_objects.at(i).getFlowPoints().extrapolate_flowpoints(F_gt_write, fs,
                                       cv::Point2i(list_of_objects.at(i).getFlowPoints().get().at
                                                           (frame_skip).at(frame_count).first.x,
                                                   list_of_objects.at(i).getFlowPoints().get().at
                                                           (frame_skip).at(frame_count).first.y),
                                                                             list_of_objects.at(i).getData().cols, list_of_objects.at(i).getData().rows,
                                       list_of_objects.at(i).getFlowPoints().get().at
                                               (frame_skip).at(frame_count).second.x,
                                       list_of_objects.at(i).getFlowPoints().get().at
                                               (frame_skip).at(frame_count).second.y, m_dataset);
            }
            F_gt_write.write(temp_gt_flow_image_path);
        }
        fs.release();
    }


    // plotVectorField (F_gt_write,m_base_directory_path_image_out.parent_path().string(),file_name);
    toc_all = steady_clock::now();
    time_map["ground truth"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth flow generation time - " << time_map["ground truth"]  << "ms" << std::endl;

}


void GroundTruthFlow::plot(std::string resultsordner) {

    PlotFlow::plot(m_dataset, resultsordner);

}

