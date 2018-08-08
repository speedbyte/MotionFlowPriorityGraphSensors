
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


void GroundTruthFlow::prepare_directories(ushort SENSOR_COUNT, std::string noise, ushort fps, ushort stepSize) {

    m_GroundTruthImageLocation = Dataset::getGroundTruthPath().string() + "/blue_sky";

    m_resultordner="/ground_truth";

    m_generatepath = Dataset::getGroundTruthPath().string() + m_resultordner;

    if (!Dataset::getDatasetPath().compare(CPP_DATASET_PATH) || !Dataset::getDatasetPath().compare(VIRES_DATASET_PATH)) {

        prepare_directories_common(SENSOR_COUNT);

    }
}

void GroundTruthFlow::CannyEdgeDetection(std::string flow_path, std::string edge_path) {

    cv::Mat src, src_gray;
    cv::Mat dst, detected_edges, blurred_image;

    int edgeThresh = 1;
    int lowThreshold=15;

    int const max_lowThreshold = 100;
    int ratio = 10;
    int kernel_size = 3;
    std::string window_name = "Edge Map";
    std::string path;
    src = cv::imread(flow_path);

    if( !src.data ) {
        std::cout << "no image found";
        exit(-1);
    }

    /// Create a matrix of the same type and size as src (for dst)
    dst.create( src.size(), src.type() );

    /// Convert the image to grayscale
    cv::cvtColor( src, src_gray, CV_BGR2GRAY );

    /// Create a window
    //cv::namedWindow( window_name, CV_WINDOW_AUTOSIZE );

    /// Create a Trackbar for user to enter threshold
    //cv::createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

    /// Reduce noise with a kernel 3x3
    cv::blur( src_gray, blurred_image, cv::Size(3,3) );

    /// Canny detector
    cv::Canny( blurred_image, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

    /// Using Canny's output as a mask, we display our result
    dst = cv::Scalar::all(0);

    src.copyTo( dst, detected_edges);
    cv::imwrite( edge_path, dst );

    //cv::imshow( window_name, dst);

    /// Wait until user exit program by pressing a key
    //cv::waitKey(0);
}


void GroundTruthFlow::generate_edge_images(ushort SENSOR_COUNT) {

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the position in a png file

    auto tic_all = steady_clock::now();

    char sensor_index_folder_suffix[50];
    for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

        unsigned FRAME_COUNT = (unsigned)m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(sensor_index).size();
        assert(FRAME_COUNT>0);
        cv::Mat image_02_frame = cv::Mat::zeros(Dataset::getFrameSize(), CV_32FC3);
        sprintf(sensor_index_folder_suffix, "%02d", m_evaluation_list.at(sensor_index));
        std::cout << "saving edge files in edge/ for sensor_index  " <<  sensor_index_folder_suffix << std::endl;

        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

            char file_name_input_image[50];
            std::cout << "current_frame_index " << current_frame_index << std::endl;
            ushort evaluation_frame_index = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                    (0).at(current_frame_index).frame_no;
            sprintf(file_name_input_image, "000%03d_10.png", evaluation_frame_index);
            std::string input_image_path = m_GroundTruthImageLocation.string() + "_" + sensor_index_folder_suffix + "/" + file_name_input_image;
            image_02_frame = cv::imread(input_image_path, CV_LOAD_IMAGE_COLOR);
            if ( image_02_frame.data == NULL ) {
                std::cerr << input_image_path << " not found" << std::endl;
                throw ("No image file found error");
            }

            std::string edge_path = m_edge_path.string() + sensor_index_folder_suffix + "/" + file_name_input_image;
            CannyEdgeDetection(input_image_path, edge_path);
        }
    }

    std::cout << "end of saving ground truth flow files " << std::endl;

}


void GroundTruthFlow::generate_depth_images(ushort SENSOR_COUNT) {

    // reads the flow vector array already created at the time of instantiation of the object.
    // Additionally stores the frames in a png file
    // Additionally stores the position in a png file

    auto tic_all = steady_clock::now();

    char sensor_index_folder_suffix[50];
    for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

        unsigned FRAME_COUNT = (unsigned)m_ptr_list_gt_objects.at(0)->get_object_extrapolated_point_displacement().at(sensor_index).size();
        assert(FRAME_COUNT>0);
        cv::Mat image_02_frame = cv::Mat::zeros(Dataset::getFrameSize(), CV_32FC1);
        sprintf(sensor_index_folder_suffix, "%02d", m_evaluation_list.at(sensor_index));
        std::cout << "saving depth files in edge/ for sensor_index  " << sensor_index_folder_suffix << std::endl;

        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {

            char file_name_input_image[50];
            std::cout << "current_frame_index " << current_frame_index << std::endl;
            ushort evaluation_frame_index = m_ptr_list_gt_objects.at(0)->getExtrapolatedGroundTruthDetails().at
                    (0).at(current_frame_index).frame_no;
            sprintf(file_name_input_image, "depth_000%03d_10.png", evaluation_frame_index);
            std::string input_image_path = m_GroundTruthImageLocation.string() + "_" + sensor_index_folder_suffix + "/" + file_name_input_image;
            image_02_frame = cv::imread(input_image_path, CV_LOAD_IMAGE_ANYDEPTH);

            if ( image_02_frame.data == NULL ) {
                std::cerr << input_image_path << " not found" << std::endl;
                throw ("No image file found error");
            }

            printf("%d", image_02_frame.at<char>((unsigned)m_ptr_list_gt_objects.at(1)->getExtrapolatedGroundTruthDetails().at(m_evaluation_list.at(sensor_index)).at(current_frame_index).m_object_location_camera_px.cog_px.y,(unsigned)m_ptr_list_gt_objects.at(1)->getExtrapolatedGroundTruthDetails().at(m_evaluation_list.at(sensor_index)).at(current_frame_index).m_object_location_camera_px.cog_px.x));
            //cv::imshow("depth_image", image_02_frame);
            //cv::waitKey(0);
        }
    }

    std::cout << "end of showing ground truth depth files " << std::endl;

}


/*
        //cv::Vec3f *datagt_next_ptsr = flowFrame.gt_next_ptsr<cv::Vec3f>(0); // pointer to the first channel of the first element in the
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


