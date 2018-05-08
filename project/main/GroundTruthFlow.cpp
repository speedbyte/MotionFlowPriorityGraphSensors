
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

#include "Dataset.h"
#include "GroundTruthScene.h"


//Creating a movement path. The path is stored in a x and y vector

using namespace std::chrono;


void GroundTruthFlow::prepare_directories() {

    mImageabholOrt = Dataset::getGroundTruthPath().string() + "/blue_sky";

    m_resultordner="/ground_truth";

    m_generatepath = Dataset::getGroundTruthPath().string() + m_resultordner;

    if (!Dataset::getDatasetPath().compare(CPP_DATASET_PATH) || !Dataset::getDatasetPath().compare(VIRES_DATASET_PATH)) {

        std::cout << "prepare gt_flow directories" << std::endl;

        OpticalFlow::prepare_directories();

    }
}

void GroundTruthFlow::generate_flow_frame() {

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the position in a png file

    prepare_directories();

    cv::Mat tempGroundTruthImage;
    tempGroundTruthImage.create(Dataset::getFrameSize(), CV_8UC3);
    assert(tempGroundTruthImage.channels() == 3);


    std::map<std::string, double> time_map = {{"generate_single_flow_image", 0},
                                              {"generate_all_flow_image", 0}};

    auto tic_all = steady_clock::now();

    std::cout << "ground truth flow will be stored in " << m_generatepath << std::endl;

    char sensor_index_folder_suffix[50];
    //cv::FileStorage fs;
    //fs.open(m_flow_occ_path.string() + sensor_index_folder_suffix + "/" + "gt_flow.yaml", cv::FileStorage::WRITE);


    for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

        sprintf(sensor_index_folder_suffix, "%02d", sensor_index);
        std::cout << "saving ground truth flow files for sensor_index " << sensor_index << std::endl;

        unsigned FRAME_COUNT = (unsigned)m_ptr_list_gt_objects.at(0)->get_obj_extrapolated_shape_pixel_point_pixel_displacement().at(sensor_index).size();
        assert(FRAME_COUNT>0);

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
            
            char file_name_input_image[50], file_name_image_edge[50];

            std::cout << "frame_count " << frame_count << std::endl;

            sprintf(file_name_input_image, "000%03d_10.png", frame_count);
            sprintf(file_name_image_edge, "000%03d_10_edge.png", frame_count);

            std::string temp_gt_flow_image_path = m_flow_occ_path.string() + sensor_index_folder_suffix + "/" +
                    file_name_input_image;

            std::string temp_gt_image_path = mImageabholOrt.string() + "_" + std::to_string(sensor_index) + "/" +
                                             file_name_input_image;

            std::string temp_result_edge_path = m_flow_occ_path.string() + sensor_index_folder_suffix + "/" +
                                                  file_name_image_edge;

            //fs << "frame_count" << frame_count;

            FlowImageExtended F_png_write( Dataset::getFrameSize().width, Dataset::getFrameSize().height);

            cv::Mat tempMatrix;
            tempMatrix.create(Dataset::getFrameSize(), CV_32FC3);
            tempMatrix = cv::Scalar_<unsigned>(255,255,255);
            assert(tempMatrix.channels() == 3);

            for (unsigned obj_index = 0; obj_index < m_ptr_list_gt_objects.size(); obj_index++) {

                // object image_data_and_shape

                int width = cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(frame_count).m_object_dimensions_px.dim_width_m);
                int height = cvRound(m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at(sensor_index).at(frame_count).m_object_dimensions_px.dim_height_m);

                //if ( m_ptr_list_gt_objects.at(obj_index)->get_obj_extrapolated_visibility().at(sensor_index).at(frame_count) == true ) {

                if ( width  != 0) {

                    float columnBegin = m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (sensor_index).at(frame_count).m_region_of_interest_px.x;
                    float rowBegin = m_ptr_list_gt_objects.at(obj_index)->getExtrapolatedGroundTruthDetails().at
                            (sensor_index).at(frame_count).m_region_of_interest_px.y;


                    // gt_displacement
                    cv::Point2f displacement = m_ptr_list_gt_objects.at(obj_index)->get_obj_extrapolated_pixel_position_pixel_displacement().at(sensor_index).at(frame_count).second;

                    cv::Mat roi;
                    roi = tempMatrix.
                            colRange(cvRound(columnBegin), cvRound(columnBegin + width)).
                            rowRange(cvRound(rowBegin), cvRound(rowBegin + height));
                    //bulk storage
                    //roi = cv::Scalar(displacement.x, displacement.y,
                    //                 static_cast<float>(m_ptr_list_gt_objects.at(obj_index)->getObjectId()));
                    roi = cv::Scalar(displacement.x, displacement.y,
                                     static_cast<float>(1.0f));

                }
            }

            //Create png Matrix with 3 channels: x displacement. y displacment and ObjectId
            // v corresponds to next row.
            for (int32_t row = 0; row < Dataset::getFrameSize().height; row++) { // rows
                for (int32_t col = 0; col < Dataset::getFrameSize().width; col++) {  // cols
                    if (tempMatrix.at<cv::Vec3f>(row, col)[2] > 0.5 ) {
                        /*inline void setFlowU (const int32_t u,const int32_t v,const float val) {
                            data_[3*(v*width_+u)+0] = val;
                        }*/
                        F_png_write.setFlowU(col, row, tempMatrix.at<cv::Vec3f>(row, col)[0]);
                        F_png_write.setFlowV(col, row, tempMatrix.at<cv::Vec3f>(row, col)[1]);
                        //F_png_write.setObjectId(col, row, tempMatrix.at<cv::Vec3f>(row, col)[2]);
                        F_png_write.setValid(col, row, true);
                        //position.store_in_yaml(fs, cv::Point2f(row, column), cv::Point2f(xValue, yValue) );
                    }
                }
            }

            F_png_write.writeExtended(temp_gt_flow_image_path);
            CannyEdgeDetection(temp_gt_image_path, temp_result_edge_path);


        }
        //fs.release();
    }

    std::cout << "end of saving ground truth flow files " << std::endl;

    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);

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


