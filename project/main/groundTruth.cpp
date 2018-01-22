

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

#include <kitti/mail.h>
#include <kitti/io_flow.h>
#include <vires/vires_common.h>

#include <gnuplot-iostream/gnuplot-iostream.h>

#include "datasets.h"
#include "groundTruth.h"
#include "kbhit.h"

#include <unordered_map>
#include <bits/unordered_map.h>


//Creating a movement path. The path is stored in a x and y vector

using namespace std::chrono;

GroundTruth::GroundTruth(std::string dataset_path, std::string unterordner, std::string resultordner) {

    m_dataset_path = dataset_path;

    boost::filesystem::path temp;

    cv::Point2i l_pixel_position, l_pixel_movement;

    temp = m_dataset_path + unterordner + std::string("dummy.txt");
    m_base_directory_path_input_in = temp.parent_path(); //data/stereo_flow/

    temp = m_dataset_path + resultordner + std::string("dummy.txt");
    m_base_directory_path_result_out = temp.parent_path(); // results/

    m_frame_size.width = 1242;
    m_frame_size.height = 375;

    std::vector<ushort> theta;
    for ( ushort frame_count = 0; frame_count < MAX_ITERATION_THETA; frame_count++) {
        theta.push_back(frame_count);
    }
    // Prepare points
    for ( int i = 0; i< MAX_ITERATION_THETA; i++) {

        l_pixel_position.x = (static_cast<ushort>((m_frame_size.width/2) + (500 * cos(theta[i] *CV_PI / 180.0) /
                                                                   (1.0 + std::pow(sin(theta[i] * CV_PI / 180.0), 2)))));

        l_pixel_position.y = (static_cast<ushort>((m_frame_size.height/2) + (55 * (cos(theta[i] * CV_PI / 180.0) *
                                                                          sin(theta[i] * CV_PI / 180.0)) / (0.2 +std::pow(sin(theta[i] * CV_PI / 180.0),2)))));

        m_trajectory_1.push_back(l_pixel_position);

    }

    m_pedesterianImage.create(object_height, object_width, CV_8UC3);

    uchar r = 0;
    uchar b = 0;

    r = 0;
    b = 0;
    for (int k = 0; k < (object_height - 1); k++) {
        for (int j = 0; j < (object_width -1 ); j++) {
            m_pedesterianImage.at<cv::Vec3b>(k, j)[0] = b;
            m_pedesterianImage.at<cv::Vec3b>(k, j)[1] = 0;
            m_pedesterianImage.at<cv::Vec3b>(k, j)[2] = r;
            r = r + (uchar)2;
            b = b + (uchar)2;
            if (r > 254)
                r = 130;
        }
        if (b > 254)
            b = 46;
    }

}

/**
 * This function means that the directories would be deleted and then created.
 * Hence only those datasets that has been synthetically produced on the test bench by us, should be deleted.
 * The results directory can be generally deleted, because they are all created by us.
 *
 * dataset/results/flow_occ_<algorithm>_<source>_<click_speed>_<noise_type>
 *
 * @param dataset_path
 * @param result_sha
 * @return
 */
void GroundTruth::prepare_result_directories(std::string resultordner) {

    char char_dir_append[20];
    if ( boost::filesystem::exists(m_base_directory_path_result_out) ) {
        system(("rm -rf " + m_base_directory_path_result_out.string() + "/" + resultordner).c_str());
    }
    std::cout << "Creating directories" << std::endl;
    boost::filesystem::create_directories(m_base_directory_path_result_out.string());
    // create flow directories
    for (int i = 1; i < 10; ++i) {
        sprintf(char_dir_append, "%02d", i);
        boost::filesystem::create_directories(m_base_directory_path_result_out.string() + "/" + resultordner +
        "/flow_occ_" + char_dir_append);
        boost::filesystem::create_directories(m_base_directory_path_result_out.string() + "/" + resultordner +
                                                      "/plots_" + char_dir_append);
    }
    std::cout << "Ending directories" << std::endl;
}

void GroundTruth::prepare_gt_data_and_gt_flow_directories() {

    char char_dir_append[20];

    if (!m_dataset_path.compare(CPP_DATASET_PATH) || !m_dataset_path.compare(VIRES_DATASET_PATH) ) {

        // delete ground truth image and ground truth flow directories
        if ( boost::filesystem::exists(m_base_directory_path_input_in) ) {
            system(("rm -rf " + m_base_directory_path_input_in.string()).c_str()); // data/stereo_flow/
        }

        // create base directories
        boost::filesystem::create_directories(m_base_directory_path_input_in.string());
        std::cout << "Creating directories" << std::endl;
        boost::filesystem::create_directories(m_base_directory_path_input_in.string() + "/image_02");

        // create flow directories
        for (int i = 1; i < 10; ++i) {
            sprintf(char_dir_append, "%02d", i);
            boost::filesystem::create_directories(m_base_directory_path_input_in.string() + "/flow_occ_" + char_dir_append);
        }
        std::cout << "Ending directories" << std::endl;
    }
}


void GroundTruth::generate_gt_image_and_gt_flow(void) {

    prepare_gt_data_and_gt_flow_directories();

    cv::Mat tempGroundTruthImage;
    tempGroundTruthImage.create(m_frame_size, CV_8UC3);
    assert(tempGroundTruthImage.channels() == 3);

    const ushort start=60;

    std::map<std::string, double> time_map = {{"generate",0}, {"ground truth", 0}};

    std::cout << "ground truth images will be stored in " << m_base_directory_path_input_in.string() << std::endl;

    auto tic = steady_clock::now();
    auto toc = steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();

    //Initialization
    ushort current_index = 0;
    //for moving the objects later
    char folder_name_flow[50];
    char file_name_image[50];

    current_index = start;

    for (ushort frame_count=0; frame_count < MAX_ITERATION_GT; frame_count++) {

        sprintf(file_name_image, "000%03d_10.png", frame_count);
        std::string input_image_file_with_path = m_base_directory_path_input_in.string() + "/image_02/" + file_name_image;

        tempGroundTruthImage = cv::Scalar::all(0);

        //draw new ground truth image.
        m_pedesterianImage.copyTo(tempGroundTruthImage(
                cv::Rect(m_trajectory_1.at(current_index).x, m_trajectory_1.at(current_index).y, object_width,
                         object_height)));
        toc = steady_clock::now();
        time_map["generate"] = duration_cast<milliseconds>(toc - tic).count();
        cv::imwrite(input_image_file_with_path, tempGroundTruthImage);
        current_index++;
        if ((current_index) >= m_trajectory_1.size() ) {
            current_index = 0;
        }
    }

    current_index = start;
    cv::Point2i l_pixel_position, l_pixel_movement;

    std::vector<std::pair<cv::Point2i, cv::Point2i> > m_flow_matrix_gt_single_points;

    for (ushort frame_count=0; frame_count < MAX_ITERATION_GT; frame_count++) {
        // The first frame is the reference frame, hence it is skipped
        if ( frame_count > 0 ) {
            //If we are at the end of the path vector, we need to reset our iterators
            if ((current_index) >= m_trajectory_1.size()) {
                current_index = 0;
                l_pixel_movement.x = m_trajectory_1.at(current_index).x - m_trajectory_1.at(m_trajectory_1.size() - 1).x;
                l_pixel_movement.y = m_trajectory_1.at(current_index).y - m_trajectory_1.at(m_trajectory_1.size() - 1).y;
                l_pixel_position = m_trajectory_1.at(current_index);
            } else {
                l_pixel_movement.x = m_trajectory_1.at(current_index).x - m_trajectory_1.at(current_index - (ushort) 1).x;
                l_pixel_movement.y = m_trajectory_1.at(current_index).y - m_trajectory_1.at(current_index - (ushort) 1).y;
                l_pixel_position = m_trajectory_1.at(current_index);
            }

            printf("%u, %u , %u, %u, %u, %d, %d\n", frame_count, start, current_index, l_pixel_position.x, l_pixel_position.y,
                   l_pixel_movement.x, l_pixel_movement.y);

            // make m_flow_matrix_gt_single_points with smallest resolution.
            m_flow_matrix_gt_single_points.push_back(std::make_pair(l_pixel_position, l_pixel_movement));
        }
        else {
            m_flow_matrix_gt_single_points.push_back(std::make_pair(cv::Point2i(0,0), cv::Point2i(0,0)));
        }
        current_index++;
    }

    for ( int frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++ ){
        int temp_flow_x = 0, temp_flow_y = 0 ;
        sprintf(folder_name_flow, "flow_occ_%02d", frame_skip);
        cv::FileStorage fs;
        fs.open(m_base_directory_path_input_in.string() + "/" + folder_name_flow + "/" + "gt_flow.yaml",
                cv::FileStorage::WRITE);
        for (ushort frame_count=1; frame_count < MAX_ITERATION_GT; frame_count++) {
            // The first frame is the reference frame.
                //the below code has to go through consecutive frames
            if ( frame_count%frame_skip != 0 ) {
                temp_flow_x += m_flow_matrix_gt_single_points.at(frame_count).second.x;
                temp_flow_y += m_flow_matrix_gt_single_points.at(frame_count).second.y;
                continue;
            }
            temp_flow_x += m_flow_matrix_gt_single_points.at(frame_count).second.x;
            temp_flow_y += m_flow_matrix_gt_single_points.at(frame_count).second.y;
            fs << "frame_count" << frame_count;
            sprintf(file_name_image, "000%03d_10.png", frame_count);
            std::string temp_gt_flow_path = m_base_directory_path_input_in.string() + "/" + folder_name_flow + "/"
                                         + file_name_image;
            extrapolate_objects( fs, cv::Point2i(m_flow_matrix_gt_single_points.at(frame_count).first.x,
                                              m_flow_matrix_gt_single_points.at
                                         (frame_count).first.y),
                                 object_width, object_height, temp_flow_x, temp_flow_y, temp_gt_flow_path );
            temp_flow_x = 0, temp_flow_y = 0 ;
        }
        fs.release();
    }

    // plotVectorField (F_gt_write,m_base_directory_path_image_out.parent_path().string(),file_name);
    toc_all = steady_clock::now();
    time_map["ground truth"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth generation time - " << time_map["ground truth"]  << "ms" << std::endl;

}

/**
 *
 * @param pt - start from pt
 * @param width - extrapolate pt.y + width
 * @param height - extrapolate pt.x + height
 * @param xValue - what x value?
 * @param yValue - what y value?
 * @param image_path - where should the extrapolated image be stored?
 */

void GroundTruth::extrapolate_objects( cv::FileStorage fs, cv::Point2i pt, ushort width, ushort height, int xValue, int
yValue, std::string image_path) {

    cv::Mat tempMatrix;
    tempMatrix.create(m_frame_size,CV_32FC3);
    assert(tempMatrix.channels() == 3);

    tempMatrix = cv::Scalar::all(0);
    cv::Mat roi;
    roi = tempMatrix.
            colRange(pt.x, (pt.x + width)).
            rowRange(pt.y, (pt.y + height));
    //bulk storage
    roi = cv::Scalar(xValue, yValue, 1.0f);

    // TODO take all the non 0 data in a float matrix and then call FlowImage Constructor with additional data
    // parameter
    //Create png Matrix with 3 channels: x displacement. y displacment and Validation bit
    FlowImage F_gt_write(m_frame_size.width, m_frame_size.height);
    for (int32_t row=0; row<m_frame_size.height; row++) { // rows
        for (int32_t column=0; column<m_frame_size.width; column++) {  // cols
            if (tempMatrix.at<cv::Vec3f>(row,column)[2] > 0.5 ) {
                F_gt_write.setFlowU(column,row,yValue);
                F_gt_write.setFlowV(column,row,xValue);
                F_gt_write.setValid(column,row,1.0f);
                store_in_yaml(fs, row, column, xValue, yValue );
            }
        }
    }
    F_gt_write.write(image_path);

}

void GroundTruth::store_in_yaml(cv::FileStorage &fs, ushort currentPixelPositionX,
                                ushort currentPixelPositionY, int XMovement, int YMovement ) {

    fs << "gt flow png file read" << "[";
    fs << "{:" << "row" <<  currentPixelPositionX << "col" << currentPixelPositionY << "displacement" << "[:";
    fs << XMovement;
    fs << YMovement;
    fs << 1;
    fs << "]" << "}";
    fs << "]";
}

void GroundTruth::calculate_flow(const boost::filesystem::path dataset_path, const std::string input_image_folder,
                                 ALGO_TYPES algo, FRAME_TYPES frame_types, NOISE_TYPES noise) {

    std::string resultordner = "results_";
    switch ( algo ) {
        case lk: {
            resultordner += "LK_";
            break;
        }
        case fb: {
            resultordner += "FB_";
            break;
        }
        default: {
            throw("algorithm not yet supported");
            break;
        }
    }

    switch ( noise ) {
        case no_noise: {
            resultordner += "no_noise/";
            break;
        }
        case static_bg_noise: {
            resultordner += "static_bg_noise/";
            break;
        }
        case static_fg_noise: {
            resultordner += "static_fg_noise/";
            break;
        }
        case dynamic_bg_noise: {
            resultordner += "dynamic_bg_noise/";
            break;
        }
        case dynamic_fg_noise: {
            resultordner += "dynamic_fg_noise/";
            break;
        }
        default: {
            throw("algorithm not yet supported");
            break;
        }
    }

    prepare_result_directories(resultordner);


    for ( int frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++ ){

        char folder_name_flow[50];
        char file_name_image[50];

        std::vector<unsigned> x_pts;
        std::vector<double> y_pts;
        std::vector<unsigned> z_pts;
        std::vector<float> time;
        double sum_time = 0;

        std::vector<boost::tuple<std::vector<unsigned>, std::vector<double>> > pts_exectime;

        bool needToInit = true;
        std::vector<cv::Point2f> prev_pts;
        std::vector<cv::Point2f> next_pts;

        std::cout << "results will be stored in " << resultordner << std::endl;

        if ( frame_types == video_frames) {
            cv::VideoCapture cap;
            cap.open(m_base_directory_path_input_in.string() + "image_02/movement.avi");
            if (!cap.isOpened()) {
                std::cout << "Could not initialize capturing...\n";
                return;
            }
        }
        cv::Mat curGray, prevGray;
        sprintf(folder_name_flow, "flow_occ_%02d", frame_skip);
        std::string results_flow_matrix_str = m_base_directory_path_result_out.string() + "/" + resultordner + "/" +
                folder_name_flow + "/" + "result_flow.yaml";
        cv::VideoWriter video_out;

        if ( frame_types == video_frames)
        {
            boost::filesystem::path video_out_path = m_base_directory_path_result_out.string() + "/" + std::string("/video/OpticalFlow.avi");
            assert(boost::filesystem::exists(video_out_path.parent_path()) != 0);
            //frame_size.height =	(unsigned) cap.get(CV_CAP_PROP_FRAME_HEIGHT );
            //frame_size.width =	(unsigned) cap.get(CV_CAP_PROP_FRAME_WIDTH );
            video_out.open(video_out_path.string(),CV_FOURCC('D','I','V','X'), 5, m_frame_size);
            printf("Writer eingerichtet\n");
        }

        cv::Mat image_02_frame = cv::Mat::zeros(m_frame_size, CV_8UC3);
        cv::Mat flowImage(m_frame_size, CV_32FC3);

        cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
        cv::Size subPixWinSize(10, 10), winSize(21, 21);

        const int MAX_COUNT = 5000;

        cv::namedWindow(resultordner, CV_WINDOW_AUTOSIZE);

        //how many interations(frames)?
        auto tic = steady_clock::now();
        auto toc = steady_clock::now();

        ushort collision = 0, iterator = 0, sIterator = 0;
        std::vector<ushort> xPos, yPos;



        std::map<std::string, double> time_map = {{"generate",0},
                                                  {"ground truth", 0},
                                                  {"FB", 0},
                                                  {"LK", 0},
                                                  {"movement", 0},
                                                  {"collision", 0},
        };

        bool plotTime = 1;
        std::vector<bool> error(2);
        error.at(0) = 0;
        error.at(1) = 0;

        std::string temp_result_flow_path;
        cv::FileStorage fs;
        fs.open(results_flow_matrix_str, cv::FileStorage::WRITE);


        for (ushort frame_count=0; frame_count < MAX_ITERATION_RESULTS; frame_count++) {
            //draw new ground truth flow.
            if ( frame_count%frame_skip != 0 ) {
                continue;
            }
            sprintf(file_name_image, "000%03d_10.png", frame_count);
            flowImage = cv::Scalar(0,0,0);
            assert(flowImage.channels() == 3);
            // Break out of the loop if the user presses the Esc key
            char c = (char) cv::waitKey(10);
            switch (c) {
                case 27:
                    break;
                case 'r':
                    needToInit = true;
                    break;
                case 'c':
                    prev_pts.clear();
                    next_pts.clear();
                    break;
                default:
                    break;
            }

            //cap >> image_02_frame;
            //if (image_02_frame.empty())
            //    break;

            std::string input_image_file_with_path = m_base_directory_path_input_in.string() + "/image_02/" + file_name_image;

            image_02_frame = cv::imread(input_image_file_with_path, CV_LOAD_IMAGE_COLOR);

            if ( image_02_frame.data == NULL ) {
                std::cerr << input_image_file_with_path << " not found" << std::endl;
                throw ("No image file found error");
            }

            temp_result_flow_path = m_base_directory_path_result_out.string() + "/" + resultordner + "/" +
                                    folder_name_flow + "/" + file_name_image;

            // Convert to grayscale
            cv::cvtColor(image_02_frame, curGray, cv::COLOR_BGR2GRAY);

            cv::Mat flow_frame( m_frame_size, CV_32FC2 );

            std::vector<std::pair<cv::Point2i, cv::Point2i> > m_flow_matrix_result;

            if ( fb == algo ) {

                tic = steady_clock::now();

                if (prevGray.data) {
                    // Initialize parameters for the optical calculate_flow algorithm
                    float pyrScale = 0.5;
                    int numLevels = 3;
                    int windowSize = 15;
                    int numIterations = 3;
                    int neighborhoodSize = 5;
                    float stdDeviation = 1.2;

                    // Calculate optical calculate_flow map using Farneback algorithm
                    // Farnback returns displacement frame and LK returns points.
                    cv::calcOpticalFlowFarneback(prevGray, curGray, flow_frame, pyrScale, numLevels, windowSize,
                                                 numIterations, neighborhoodSize, stdDeviation,
                                                 cv::OPTFLOW_FARNEBACK_GAUSSIAN);
                    // OPTFLOW_USE_INITIAL_FLOW didnt work and gave NaNs

                    // Draw the optical calculate_flow map
                    int stepSize = 16;

                    // Draw the uniform grid of points on the input image along with the motion vectors
                    for (int row = 0; row < image_02_frame.rows; row += stepSize) {
                        for (int col = 0; col < image_02_frame.cols; col += stepSize) {
                            // Circles to indicate the uniform grid of points
                            //cv::circle(image_02_frame, cv::Point(x, y), 1, cv::Scalar(0, 0, 0), -1, 8);

                            cv::Point2i l_pixel_position, l_pixel_movement;

                            float valx = flow_frame.at<cv::Point2f>(row, col).x;
                            float valy = flow_frame.at<cv::Point2f>(row, col).y;

                            if (( cvFloor(std::abs(valx)) != 0 && cvFloor(std::abs(valy)) != 0 ) ) {

                                printf("flow_frame.at<cv::Point2f>(%d, %d).x =  %f\n", row, col, valx);
                                printf("flow_frame.at<cv::Point2f>(%d, %d).y =  %f\n", row, col, valy);

                                l_pixel_movement.x = cvRound(valx + 0.5 );

                                l_pixel_movement.y = cvRound(valy + 0.5 );

                                // l_pixel_position is the new pixel position !
                                l_pixel_position.x = cvRound(col + l_pixel_movement.x);
                                l_pixel_position.y = cvRound(row + l_pixel_movement.y);

                                //printf("(iteration %u, coordinates x y (%i,%i) ->  Vx, Vy (%d,%d) \n", frame_count,
                                //       l_pixel_position.y, l_pixel_position.x, l_pixel_movement.x, l_pixel_movement.y);

                                // Lines to indicate the motion vectors
                                cv::arrowedLine( image_02_frame, cv::Point(col, row), l_pixel_position, cv::Scalar(0,
                                                                                                                   255, 0));
                                m_flow_matrix_result.push_back(std::make_pair(l_pixel_position, l_pixel_movement));
                            }
                        }
                    }
                }
                toc = steady_clock::now();
                time_map["FB"] = duration_cast<milliseconds>(toc - tic).count();
                y_pts.push_back(time_map["FB"]);
            }

            else if ( lk == algo ) {
                tic = steady_clock::now();
                // Calculate optical calculate_flow map using LK algorithm
                if (prevGray.data) {  // Calculate only on second or subsequent images.
                    std::vector<uchar> status;
                    std::vector<float> err;
                    /*if (prevGray.empty()) {
                        curGray.copyTo(prevGray);
                    }*/
                    cv::calcOpticalFlowPyrLK(prevGray, curGray, prev_pts, next_pts, status,
                                             err, winSize, 5, termcrit, 0, 0.001);

                    unsigned count = 0;
                    int minDist = 1;

                    for (unsigned i = 0; i < next_pts.size(); i++) {
                        /* If the new point is within 'minDist' distance from an existing point, it will not be tracked */
                        // auto dist = cv::norm(prev_pts[i] - next_pts[i]);
                        cv::Point2i l_pixel_position, l_pixel_movement;

                        l_pixel_movement.x = cvRound(next_pts[i].x - prev_pts[i].x + 0.5);
                        l_pixel_movement.y = cvRound(next_pts[i].y - prev_pts[i].y + 0.5);

                        double dist;
                        dist = pow(l_pixel_movement.x,2)+pow(l_pixel_movement.y,2);
                        //calculating distance by euclidean formula
                        dist = sqrt(dist);

                        if ( dist <= minDist ) {
                            printf("minimum distance for %i is %f\n", i, dist);
                            continue;
                        }

                        // Check if the status vector is good
                        if (!status[i])
                            continue;

                        next_pts[count++] = next_pts[i];
                        // draw lines where there is some displacement
                        if ( l_pixel_movement.x != 0 && l_pixel_movement.y!= 0) {

                            // l_pixel_position is the new pixel position !
                            l_pixel_position.x = std::abs(cvRound(next_pts[i].x));
                            l_pixel_position.y = std::abs(cvRound(next_pts[i].y));

                            printf("(iteration %u, coordinates x y (%i,%i) ->  Vx, Vy (%d,%d) \n", i,
                                   l_pixel_position.y, l_pixel_position.x, l_pixel_movement.x, l_pixel_movement.y);

                            // Lines to indicate the motion vectors
                            cv::arrowedLine(image_02_frame, prev_pts[i], l_pixel_position, cv::Scalar(0, 255, 0));
                            m_flow_matrix_result.push_back(std::make_pair(l_pixel_position, l_pixel_movement));
                        }
                    }
                    next_pts.resize(count);
                }
                else {
                    needToInit = true;
                }
                if (needToInit) {
                    // automatic initialization
                    cv::goodFeaturesToTrack(curGray, next_pts, MAX_COUNT, 0.01, 10, cv::Mat(), 3, false, 0.04);
                    // Refining the location of the feature points
                    assert(next_pts.size() <= MAX_COUNT );
                    std::cout << next_pts.size();
                    std::vector<cv::Point2f> currentPoint;
                    std::swap(currentPoint, next_pts);
                    next_pts.clear();
                    for (unsigned i = 0; i < currentPoint.size(); i++) {
                        std::vector<cv::Point2f> tempPoints;
                        tempPoints.push_back(currentPoint[i]);
                        // Function to refine the location of the corners to subpixel accuracy.
                        // Here, 'pixel' refers to the image patch of size 'windowSize' and not the actual image pixel
                        cv::cornerSubPix(curGray, tempPoints, subPixWinSize, cv::Size(-1, -1), termcrit);
                        next_pts.push_back(tempPoints[0]);
                    }
                    printf("old next_pts size is %ld and new next_pts size is %ld\n", currentPoint.size(), next_pts.size());
                }

                toc = steady_clock::now();
                time_map["LK"] = duration_cast<milliseconds>(toc - tic).count();
                y_pts.push_back(time_map["LK"]);
                time.push_back(duration_cast<milliseconds>(toc - tic).count());
            }


            if (prevGray.data) {

                //Create png Matrix with 3 channels: x displacement. y displacment and Validation bit
                //kitti uses col, row specification
                FlowImage F_result_write(m_frame_size.width, m_frame_size.height);
                std::vector<std::pair<cv::Point2i, cv::Point2i> >::iterator it ;
                fs << "frame_count" << frame_count;
                for ( it = m_flow_matrix_result.begin(); it != m_flow_matrix_result.end(); it++ )
                {
                    F_result_write.setFlowU((*it).first.x,(*it).first.y,(*it).second.x);
                    F_result_write.setFlowV((*it).first.x,(*it).first.y,(*it).second.y);
                    F_result_write.setValid((*it).first.x,(*it).first.y,(bool)1.0f);
                    store_in_yaml(fs, (*it).first.y, (*it).first.x, (*it).second.x, (*it).second.y  );
                }
                F_result_write.write(temp_result_flow_path);
            }

            tic = steady_clock::now();
            auto end = steady_clock::now();
            x_pts.push_back(frame_count);

            if ( frame_types == video_frames) {
                video_out.write(image_02_frame);
            }

            // Display the output image
            cv::imshow(resultordner, image_02_frame);
            needToInit = false;
            prev_pts.clear();
            std::swap(next_pts, prev_pts);
            std::swap(prevGray, curGray);
        }
        fs.release();

        for(auto &n : time)
            sum_time +=n;

        std::cout << "Noise " << noise  << ", Zeit " << sum_time << std::endl;
        std::cout << "time_map LK " << time_map["LK"] << std::endl;

        auto max = (std::max_element(y_pts.begin(), y_pts.end())).operator*();

        pts_exectime.push_back(boost::make_tuple(x_pts, y_pts));
        if ( frame_types == video_frames) {
            video_out.release();
        }
        cv::destroyAllWindows();

        // gnuplot_2d
        Gnuplot gp2d;
        gp2d << "set xrange [0:" + std::to_string(MAX_ITERATION_RESULTS) + "]\n";
        gp2d << "set yrange [0:" + std::to_string(max*2) + "]\n";
        std::string tmp = std::string(" with points title ") + std::string("'") + input_image_folder + std::string(" y "
                                                                                                                      "axis - ms, x axis - image_02_frame\n'");
        //gp2d << "plot" << gp2d.binFile2d(pts_exectime, "record") << tmp;
    }

}

void GroundTruth::generate_gt_image_and_gt_flow_vires() {

    prepare_gt_data_and_gt_flow_directories();

    char command[1024];


    sprintf(command,"cd %s; %s",(m_dataset_path + std::string("../../")).c_str(),"bash vtdSendandReceive.sh");
    std::cout << command << std::endl;
    system(command);

    std::cout << " I am out of bash" << std::endl;

    ViresInterface vi;
    std::string m_server;
    boost::filesystem::path m_ts_gt_out_dir;

    int initCounter = 6;

    // initalize the server variable
    std::string serverName = "127.0.0.1";

    vi.setServer(serverName.c_str());

    fprintf(stderr, "ValidateArgs: key = 0x%x, checkMask = 0x%x, mForceBuffer = %d\n",
            vi.getShmKey(), vi.getCheckMask(), vi.getForceBuffer());

    // open the network connection to the taskControl (so triggers may be sent)
    fprintf(stderr, "creating network connection....\n");
    vi.openNetwork();  // this is blocking until the network has been opened
    vi.openNetwork_GT();



    // now: open the shared memory (try to attach without creating a new segment)
    fprintf(stderr, "attaching to shared memory 0x%x....\n", vi.getShmKey());

    while (!vi.getShmPtr()) {
        vi.openShm();
        usleep(1000);     // do not overload the CPU
    }

    fprintf(stderr, "...attached! Reading now...\n");

    // now check the SHM for the time being
    bool breaking = false;
    int count = 0;
    while (1) {

        // Break out of the loop if the user presses the Esc key
        int c =  kbhit();
        switch (c) {
            case 9:
                breaking = true;
                break;
            default:
                break;
        }
        if ( breaking ) {
            break;
        }

        if ( count++ > 100 ) {
            breaking = true;
        }

        vi.readNetwork();

        if (initCounter <= 0)
            vi.checkShm();

        // has an image arrived or do the first frames need to be triggered
        //(first image will arrive with a certain image_02_frame delay only)
        if (vi.getHaveImage() || (initCounter-- > 0)) {
            vi.sendRDBTrigger();

        }
        // ok, reset image indicator
        vi.setHaveImage(0);

        usleep(10000); // sleep for 10 ms
        std::cout << "getting data from VIRES\n";
    }

    sprintf(command,"cd %s; %s",(m_dataset_path + std::string("../../")).c_str(),"bash vtdStop.sh");
    std::cout << command << std::endl;
    system(command);
}

void GroundTruth::plot(std::string resultsordner) {

    char folder_name_flow[50], folder_name_plot[50];
    char file_name_image[50];
    cv::Mat showErrorImage;

    for ( int frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++ ){

        sprintf(folder_name_flow, "flow_occ_%02d", frame_skip);
        sprintf(folder_name_plot, "plots_%02d", frame_skip);
        cv::namedWindow(folder_name_flow, CV_WINDOW_AUTOSIZE);

        for (ushort frame_count=1; frame_count < MAX_ITERATION_RESULTS; frame_count++) {
            if ( frame_count%frame_skip != 0 ) {
                continue;
            }
            sprintf(file_name_image, "000%03d_10.png", frame_count);
            std::string temp_gt_flow_path = m_base_directory_path_input_in.string() + "/" + folder_name_flow + "/"
                                         + file_name_image;
            std::string temp_result_flow_path = m_base_directory_path_result_out.string() + "/" + resultsordner + "/" +
                    folder_name_flow + "/"
                                         + file_name_image;
            std::string temp_plot_flow_path = m_base_directory_path_result_out.string() + "/" + resultsordner + "/" +
                                                folder_name_plot + "/"
                                                + file_name_image;
            FlowImage gt_flow_read(temp_gt_flow_path);
            FlowImage result_flow_read(temp_result_flow_path);

            png::image<png::rgb_pixel> errorImage(m_frame_size.width, m_frame_size.height);

            // all black when the flow is identical. logcolor = false
            // all blue when the flow is identical. logcolor = true
            errorImage = gt_flow_read.errorImage(result_flow_read, result_flow_read, true);
            errorImage.write(temp_plot_flow_path);

            showErrorImage = cv::imread(temp_plot_flow_path, CV_LOAD_IMAGE_ANYCOLOR);
            cv::imshow(folder_name_flow, showErrorImage);
            cv::waitKey(1000);
        }

        cv::destroyAllWindows();
    }
}
