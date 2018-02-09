
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

    char char_dir_append[20];

    if (!Dataset::getBasePath().compare(CPP_DATASET_PATH) || !Dataset::getBasePath().compare(VIRES_DATASET_PATH)) {

        std::string m_resultordner="";

        std::cout << "Creating GT Flow directories" << std::endl;
        // create flow directories
        boost::filesystem::path path;
        for (int i = 1; i < MAX_SKIPS; ++i) {
            // delete ground truth image and ground truth flow directories
            sprintf(char_dir_append, "%02d", i);
            path = Dataset::getGroundTruthFlowPath().string() + "/" + m_resultordner +
                                           "/flow_obj_" + char_dir_append;
            if (boost::filesystem::exists(path)) {
                system(("rm -rf " + path.string()).c_str());
            }
            boost::filesystem::create_directories(path);

            path = Dataset::getGroundTruthFlowPath().string() + "/" + m_resultordner +
                                                  "/flow_occ_" + char_dir_append;

            if (boost::filesystem::exists(path)) {
                system(("rm -rf " + path.string()).c_str());
            }
            boost::filesystem::create_directories(path);
        }
        std::cout << "Ending GT Flow directories" << std::endl;
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


    std::map<std::string, double> time_map = {{"generate",     0},
                                              {"ground truth", 0}};

    auto tic = steady_clock::now();
    auto toc = steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();

    std::cout << "ground truth flow will be stored in " << Dataset::getGroundTruthFlowPath().string() << std::endl;

    char folder_name_flow[50];
    cv::FileStorage fs;
    fs.open(Dataset::getGroundTruthFlowPath().string() + "/" + folder_name_flow + "/" + "gt_flow.yaml",
            cv::FileStorage::WRITE);


    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

        sprintf(folder_name_flow, "flow_occ_%02d", frame_skip);
        std::cout << "saving flow files for frame_skip " << frame_skip << std::endl;

        unsigned FRAME_COUNT = (unsigned)m_list_gt_objects.at(0).get_obj_extrapolated_pixel_centroid_pixel_displacement_mean().at(frame_skip - 1).size();
        assert(FRAME_COUNT>0);

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
            char file_name_image[50];
            std::cout << "frame_count " << frame_count << std::endl;

            sprintf(file_name_image, "000%03d_10.png", frame_count);
            std::string temp_gt_flow_image_path =
                    Dataset::getGroundTruthFlowPath().string() + "/" + folder_name_flow + "/"
                    + file_name_image;

            fs << "frame_count" << frame_count;

            FlowImageExtended F_png_write( Dataset::getFrameSize().width, Dataset::getFrameSize().height);

            cv::Mat tempMatrix;
            tempMatrix.create(Dataset::getFrameSize(), CV_32FC3);
            tempMatrix = cv::Scalar_<unsigned>(255,255,255);
            assert(tempMatrix.channels() == 3);


            for (unsigned i = 0; i < m_list_gt_objects.size(); i++) {

                // object image_data_and_shape
                int width = m_list_gt_objects.at(i).getWidth();
                int height = m_list_gt_objects.at(i).getHeight();

                if ( m_list_gt_objects.at(i).get_obj_extrapolated_visibility().at(frame_skip - 1).at(frame_count) == true ) {

                    // gt_displacement
                    cv::Point2f next_pts = m_list_gt_objects.at(i).get_obj_extrapolated_pixel_centroid_pixel_displacement_mean().at(frame_skip - 1)
                            .at(frame_count).first;
                    cv::Point2f displacement = m_list_gt_objects.at(i).get_obj_extrapolated_pixel_centroid_pixel_displacement_mean().at(frame_skip
                                                                                                                       - 1)
                            .at(frame_count).second;

                    cv::Point2f gt_line_pts = m_list_gt_objects.at(i).get_line_parameters().at(frame_skip - 1)
                            .at(frame_count).second;


                    cv::Mat roi;
                    roi = tempMatrix.
                            colRange(cvRound(next_pts.x), cvRound(next_pts.x + width)).
                            rowRange(cvRound(next_pts.y), cvRound(next_pts.y + height));
                    //bulk storage
                    roi = cv::Scalar(displacement.x, displacement.y,
                                     static_cast<float>(m_list_gt_objects.at(i).getObjectId()));

                    // cv line is intelligent and it can also project to values not within the frame size including negative values.
                    cv::line(tempMatrix, next_pts, gt_line_pts, cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0);
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

        }
        fs.release();
    }

    // plotVectorField (F_png_write,m__directory_path_image_out.parent_path().string(),file_name);
    toc_all = steady_clock::now();
    time_map["ground truth"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth flow generation time - " << time_map["ground truth"] << "ms" << std::endl;

}





void GroundTruthFlow::make_video_from_png(const Dataset &dataset_path, std::string unterordner) {

    cv::VideoWriter video_write;
    cv::Mat temp_image;

    boost::filesystem::directory_iterator end_iter;

    boost::filesystem::path dir_path = dataset_path.getGroundTruthFlowPath();

    std::cout << dir_path.string() << std::endl;
    assert(boost::filesystem::exists(dir_path) != 0);

    std::string file_name, path;
    boost::filesystem::path temp;
    bool video_writer_init = false;

    for (boost::filesystem::directory_iterator dir_iter(dir_path); dir_iter != end_iter; ++dir_iter) {
        if (boost::filesystem::is_regular_file(dir_iter->status())) {
            std::string extension = boost::filesystem::extension(*dir_iter);
            if (extension == ".png") {
                std::cout << *dir_iter << std::endl;
                temp = *dir_iter;
                temp_image = cv::imread(temp.string(), cv::IMREAD_COLOR);
                if (video_writer_init == false) {
                    if (!video_write.open((dir_path.string() + "movement_video.avi"), CV_FOURCC('D', 'I', 'V', 'X'),
                                          30.0,
                                          cv::Size(temp_image.cols, temp_image.rows), true)) {
                        std::cerr << "failed to initialise the video write" << std::endl;
                        throw;
                    }
                    if (!video_write.isOpened()) {
                        std::cerr << "Could not open video" << std::endl;
                    }

                    video_writer_init = true;
                }
                /*cv::namedWindow("video", CV_WINDOW_AUTOSIZE);
                cv::imshow("video", temp_image);
                cv::waitKey(1000);*/
                video_write.write(temp_image);
            } else {
                std::cout << "ignoring extension : " << extension << " path " << *dir_iter << std::endl;
            }
        }
        cv::destroyAllWindows();
    }

    video_write.release();
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