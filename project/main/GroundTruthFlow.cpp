
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
#include <gnuplot-iostream/gnuplot-iostream.h>

#include "datasets.h"
#include "GroundTruthFlow.h"
#include "kbhit.h"

#include <unordered_map>
#include <bits/unordered_map.h>

#include "ObjectTrajectory.h"
#include "Dataset.h"
#include "GroundTruthScene.h"


//Creating a movement path. The path is stored in a x and y vector

using namespace std::chrono;


void GroundTruthFlow::prepare_directories() {

    m_resultordner="/generated";

    m_generatepath = Dataset::getGroundTruthPath().string() + m_resultordner;

    if (!Dataset::getDatasetPath().compare(CPP_DATASET_PATH) || !Dataset::getDatasetPath().compare(VIRES_DATASET_PATH)) {

        std::cout << "prepare gt_flow directories" << std::endl;

        OpticalFlow::prepare_directories();

    }
}

void GroundTruthFlow::generate_flow_frame() {

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the trajectory in a png file

    prepare_directories();

    cv::Mat tempGroundTruthImage;
    tempGroundTruthImage.create(Dataset::getFrameSize(), CV_8UC3);
    assert(tempGroundTruthImage.channels() == 3);


    std::map<std::string, double> time_map = {{"generate_single_flow_image",     0},
                                              {"generate_all_flow_image", 0}};

    auto tic_all = steady_clock::now();

    std::cout << "ground truth flow will be stored in " << m_generatepath << std::endl;

    char frame_skip_folder_suffix[50];
    cv::FileStorage fs;
    fs.open(m_flow_occ_path.string() + frame_skip_folder_suffix + "/" + "gt_flow.yaml",
            cv::FileStorage::WRITE);


    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

        sprintf(frame_skip_folder_suffix, "%02d", frame_skip);
        std::cout << "saving ground truth flow files for frame_skip " << frame_skip << std::endl;

        unsigned FRAME_COUNT = (unsigned)m_list_gt_objects.at(0)->get_obj_extrapolated_mean_pixel_centroid_pixel_displacement().at(frame_skip - 1).size();
        assert(FRAME_COUNT>0);

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
            char file_name_image[50];

            auto tic = steady_clock::now();

            std::cout << "frame_count " << frame_count << std::endl;

            sprintf(file_name_image, "000%03d_10.png", frame_count*frame_skip);
            std::string temp_gt_flow_image_path = m_flow_occ_path.string() + frame_skip_folder_suffix + "/" +
                    file_name_image;

            fs << "frame_count" << frame_count;

            FlowImageExtended F_png_write( Dataset::getFrameSize().width, Dataset::getFrameSize().height);

            cv::Mat tempMatrix;
            tempMatrix.create(Dataset::getFrameSize(), CV_32FC3);
            tempMatrix = cv::Scalar_<unsigned>(255,255,255);
            assert(tempMatrix.channels() == 3);


            for (unsigned i = 0; i < m_list_gt_objects.size(); i++) {

                // object image_data_and_shape
                int width = cvRound(m_list_gt_objects.at(i)->get_obj_extrapolated_shape_dimension().at(frame_skip-1).at(frame_count).x);
                int height = cvRound(m_list_gt_objects.at(i)->get_obj_extrapolated_shape_dimension().at(frame_skip-1).at(frame_count).y);

                if ( m_list_gt_objects.at(i)->get_obj_extrapolated_visibility().at(frame_skip - 1).at(frame_count) == true ) {

                    // gt_displacement
                    cv::Point2f next_pts = m_list_gt_objects.at(i)->get_obj_extrapolated_mean_pixel_centroid_pixel_displacement().at(frame_skip - 1)
                            .at(frame_count).first;
                    cv::Point2f displacement = m_list_gt_objects.at(i)->get_obj_extrapolated_mean_pixel_centroid_pixel_displacement().at(frame_skip
                                                                                                                      - 1).at(frame_count).second;

                    cv::Mat roi;
                    roi = tempMatrix.
                            colRange(cvRound(next_pts.x), cvRound(next_pts.x + width)).
                            rowRange(cvRound(next_pts.y), cvRound(next_pts.y + height));
                    //bulk storage
                    roi = cv::Scalar(displacement.x, displacement.y,
                                     static_cast<float>(m_list_gt_objects.at(i)->getObjectId()));

                }
            }

            //Create png Matrix with 3 channels: x displacement. y displacment and ObjectId
            for (int32_t row = 0; row < Dataset::getFrameSize().height; row++) { // rows
                for (int32_t column = 0; column < Dataset::getFrameSize().width; column++) {  // cols
                    if (tempMatrix.at<cv::Vec3f>(row, column)[2] > 0.5 ) {
                        F_png_write.setFlowU(column, row, tempMatrix.at<cv::Vec3f>(row, column)[1]);
                        F_png_write.setFlowV(column, row, tempMatrix.at<cv::Vec3f>(row, column)[0]);
                        F_png_write.setObjectId(column, row, tempMatrix.at<cv::Vec3f>(row, column)[2]);
                        //trajectory.store_in_yaml(fs, cv::Point2f(row, column), cv::Point2f(xValue, yValue) );
                    }
                }
            }

            F_png_write.writeExtended(temp_gt_flow_image_path);

            auto toc = steady_clock::now();
            time_map["generate_single_flow_image"] = duration_cast<milliseconds>(toc - tic).count();


        }
        fs.release();
    }

    std::cout << "end of saving ground truth flow files " << std::endl;

    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
    auto toc_all = steady_clock::now();
    time_map["generate_all_flow_image"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth flow generation time - " << time_map["generate_all_flow_image"] << "ms" << std::endl;



}


/*
        //cv::Vec3f *datagt_next_ptsr = tempMatrix.gt_next_ptsr<cv::Vec3f>(0); // pointer to the first channel of the first element in the
        // first row. The r, g b  value of single pixels are continous.
        float *array = (float *)malloc(3*sizeof(float)*Dataset::getFrameSize().width*Dataset::getFrameSize().height);
        cv::MatConstIterator_<cv::Vec3f> it = roi.begin<cv::Vec3f>();
        for (unsigned i = 0; it != roi.end<cv::Vec3f>(); it++ ) {
            for ( unsigned j = 0; j < 3; j++ ) {
                *(array + i ) = (*it)[j];
                i++;
            }
        }
        FlowImageExtended temp = FlowImageExtended(array, Dataset::getFrameSize().width, Dataset::getFrameSize().height );
        F_png_write = temp;

 */