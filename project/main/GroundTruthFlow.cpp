
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


void GroundTruthFlow::generate_gt_scene_flow_vector() {

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

    for ( int i = 0; i < m_list_objects.size(); i++ ) {
        m_list_objects.at(i).generate_base_flow_vector();
        // extend the flow vectors by skipping frames and then storing them as pngs
        m_list_objects.at(i).generate_extended_flow_vector();
    }

    char folder_name_flow[50];
    cv::FileStorage fs;
    fs.open(m_dataset.getGroundTruthFlowPath().string() + "/" + folder_name_flow + "/" + "gt_flow.yaml",
            cv::FileStorage::WRITE);
    std::vector<std::vector<std::pair<cv::Point2i, cv::Point2i> > > objects;



    for ( unsigned frame_skip = 1; frame_skip < m_list_objects.at(0).getFlowPoints().get().size() ;
          frame_skip++ ) {

        sprintf(folder_name_flow, "flow_occ_%02d", frame_skip);

        std::cout << "saving flow files for frame_skip " << frame_skip << std::endl;

        for (ushort frame_count = 0; frame_count < (m_list_objects.at(0).getFlowPoints().get().at
                (frame_skip-1)).size(); frame_count++) {

            char file_name_image[50];
            sprintf(file_name_image, "000%03d_10.png", frame_count);
            std::string temp_gt_flow_image_path = m_dataset.getGroundTruthFlowPath().string() + "/" + folder_name_flow + "/"
                                                  + file_name_image;

            fs << "frame_count" << frame_count;
            extrapolate_flowpoints(temp_gt_flow_image_path, frame_skip, frame_count, m_list_objects,
                                              m_dataset);
        }
        fs.release();
    }

    // plotVectorField (F_gt_write,m_base_directory_path_image_out.parent_path().string(),file_name);
    toc_all = steady_clock::now();
    time_map["ground truth"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth flow generation time - " << time_map["ground truth"]  << "ms" << std::endl;

}



void GroundTruthFlow::extrapolate_flowpoints(std::string temp_gt_flow_image_path, unsigned frame_skip, unsigned
frame_count,
                                        std::vector<Objects> list_objects, Dataset &dataset) {

    FlowImage F_gt_write(dataset.getFrameSize().width, dataset.getFrameSize().height);
    cv::Mat tempMatrix;
    tempMatrix.create(dataset.getFrameSize(),CV_32FC3);
    assert(tempMatrix.channels() == 3);
    //tempMatrix = cv::Scalar::all(0);
    for ( unsigned i = 0; i < list_objects.size(); i++ ) {

        // object shape
        int width = list_objects.at(i).getShapeImageData().get().cols;
        int height = list_objects.at(i).getShapeImageData().get().rows;

        // displacement
        cv::Point2i pt = list_objects.at(i).getFlowPoints().get().at(frame_skip-1).at(frame_count).first;
        cv::Point2f displacement = list_objects.at(i).getFlowPoints().get().at(frame_skip-1).at(frame_count).second;

        cv::Mat roi;
        roi = tempMatrix.
                colRange(pt.x, (pt.x + width)).
                rowRange(pt.y, (pt.y + height));
        //bulk storage
        roi = cv::Scalar(displacement.x, displacement.y, 1.0f);

/*
        //cv::Vec3f *dataPtr = tempMatrix.ptr<cv::Vec3f>(0); // pointer to the first channel of the first element in the
        // first row. The r, g b  value of single pixels are continous.
        float *array = (float *)malloc(3*sizeof(float)*dataset.getFrameSize().width*dataset.getFrameSize().height);
        cv::MatConstIterator_<cv::Vec3f> it = roi.begin<cv::Vec3f>();
        for (unsigned i = 0; it != roi.end<cv::Vec3f>(); it++ ) {
            for ( unsigned j = 0; j < 3; j++ ) {
                *(array + i ) = (*it)[j];
                i++;
            }
        }
        FlowImage temp = FlowImage(array, dataset.getFrameSize().width, dataset.getFrameSize().height );
        F_gt_write = temp;

 */
    }

    //Create png Matrix with 3 channels: x displacement. y displacment and Validation bit
    for (int32_t row=0; row<dataset.getFrameSize().height; row++) { // rows
        for (int32_t column=0; column<dataset.getFrameSize().width; column++) {  // cols
            if (tempMatrix.at<cv::Vec3f>(row,column)[2] > 0.5 ) {
                F_gt_write.setFlowU(column,row,tempMatrix.at<cv::Vec3f>(row,column)[1]);
                F_gt_write.setFlowV(column,row,tempMatrix.at<cv::Vec3f>(row,column)[0]);
                F_gt_write.setValid(column,row,1.0f);
                //trajectory.store_in_yaml(fs, cv::Point2i(row, column), cv::Point2i(xValue, yValue) );
            }
        }
    }
    F_gt_write.write(temp_gt_flow_image_path);
}


