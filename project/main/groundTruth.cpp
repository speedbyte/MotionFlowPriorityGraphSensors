

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

#include <unordered_map>
#include <bits/unordered_map.h>


//Creating a movement path. The path is stored in a x and y vector

using namespace std::chrono;

GroundTruth::GroundTruth(std::string dataset_path, std::string unterordner, std::string resultordner) {

    m_dataset_path = dataset_path;

    boost::filesystem::path temp;

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

        m_position.x = (static_cast<ushort>((m_frame_size.width/2) + (500 * cos(theta[i] *CV_PI / 180.0) /
                                                                   (1.0 + std::pow(sin(theta[i] * CV_PI / 180.0), 2)))));

        m_position.y = (static_cast<ushort>((m_frame_size.height/2) + (55 * (cos(theta[i] * CV_PI / 180.0) *
                                                                          sin(theta[i] * CV_PI / 180.0)) / (0.2 +std::pow(sin(theta[i] * CV_PI / 180.0),2)))));

        m_position_matrix.push_back(m_position);

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

    m_position = {0,0};
    m_movement = {0,0};
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
        system(("rm -rf " + m_base_directory_path_result_out.string()).c_str());
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

    m_fs.open(m_base_directory_path_input_in.string() + "flow_occ_01/gt_flow.yaml", cv::FileStorage::WRITE);

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
    char file_name[20];

    current_index = start;

    for (ushort frame_count=0; frame_count < MAX_ITERATION; frame_count++) {

        sprintf(file_name, "000%03d_10.png", frame_count);
        std::string temp_image_path = m_base_directory_path_input_in.string() + "/image_02/" + file_name;

        tempGroundTruthImage = cv::Scalar::all(0);

        //draw new ground truth image.
        m_pedesterianImage.copyTo(tempGroundTruthImage(
                cv::Rect(m_position_matrix.at(current_index).x, m_position_matrix.at(current_index).y, object_width,
                         object_height)));
        toc = steady_clock::now();
        time_map["generate"] = duration_cast<milliseconds>(toc - tic).count();
        cv::imwrite(temp_image_path, tempGroundTruthImage);
        if ((current_index) >= m_position_matrix.size() ) {
            current_index = 0;
        }
        current_index++;
    }

    current_index = start;

    for (ushort frame_count=0; frame_count < MAX_ITERATION; frame_count++) {

        //If we are at the end of the path vector, we need to reset our iterators
        if ((current_index) >= m_position_matrix.size()) {
            current_index = 0;
            m_movement.x = m_position_matrix.at(current_index).x - m_position_matrix.at(m_position_matrix.size() - 1).x;
            m_movement.y = m_position_matrix.at(current_index).y - m_position_matrix.at(m_position_matrix.size() - 1).y;
            m_position = m_position_matrix.at(current_index);
        } else {
            m_movement.x = m_position_matrix.at(current_index).x - m_position_matrix.at(current_index - (ushort) 1).x;
            m_movement.y = m_position_matrix.at(current_index).y - m_position_matrix.at(current_index - (ushort) 1).y;
            m_position = m_position_matrix.at(current_index);
        }

        printf("%u, %u , %u, %u, %u, %d, %d\n", frame_count, start, current_index, m_position.x, m_position.y,
               m_movement.x, m_movement.y);

        m_flow_matrix.push_back(std::make_pair(m_position, m_movement));
        current_index++;
    }


    for (ushort frame_count=0; frame_count < MAX_ITERATION; ) {
        sprintf(file_name, "000%03d_10.png", frame_count);
        std::string temp_flow_path = m_base_directory_path_input_in.string() + "/flow_occ_01/" + file_name;
        if ( frame_count > 0 ) {
            //draw new ground truth flow.
            extrapolate_objects( cv::Point2i(m_flow_matrix.at(frame_count).first.x, m_flow_matrix.at
                                         (frame_count).first.y),
                                 object_width, object_height, m_flow_matrix.at(frame_count).second.x, m_flow_matrix.at
                            (frame_count).second.y, temp_flow_path );
        }
        frame_count = frame_count + (ushort)1;
    }

    int temp_flow_x = 0, temp_flow_y = 0 ;
    for ( int frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++ ){
        for (ushort frame_count=0; frame_count < MAX_ITERATION; frame_count++) {
            if ( frame_count > 0 ) {
                //draw new ground truth flow.
                if ( frame_count%frame_skip != 0 ) {
                    temp_flow_x += m_flow_matrix.at(frame_count).second.x;
                    temp_flow_y += m_flow_matrix.at(frame_count).second.y;
                    continue;
                }
                temp_flow_x = 0; temp_flow_y = 0;
                sprintf(file_name, "_%02d/000%03d_10.png", frame_skip, frame_count);
                std::string temp_flow_path = m_base_directory_path_input_in.string() + "/flow_occ" + file_name;
                extrapolate_objects( cv::Point2i(m_flow_matrix.at(frame_count).first.x, m_flow_matrix.at
                                             (frame_count).first.y),
                                     object_width, object_height, temp_flow_x, temp_flow_x, temp_flow_path );
            }
        }
    }

    // plotVectorField (F_gt_write,m_base_directory_path_image_out.parent_path().string(),file_name);
    m_fs.release();
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

void GroundTruth::extrapolate_objects( cv::Point2i pt, ushort width, ushort height, int xValue, int yValue,
                                       std::string image_path) {

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

    //Create png Matrix with 3 channels: x displacement. y displacment and Validation bit
    FlowImage F_gt_write(m_frame_size.width, m_frame_size.height);
    for (int32_t v=0; v<m_frame_size.height; v++) { // rows
        for (int32_t u=0; u<m_frame_size.width; u++) {  // cols
            if (tempMatrix.at<cv::Vec3f>(v,u)[2] > 0.5 ) {
                F_gt_write.setFlowU(u,v,tempMatrix.at<cv::Vec3f>(v,u)[0]);
                F_gt_write.setFlowV(u,v,tempMatrix.at<cv::Vec3f>(v,u)[1]);
                F_gt_write.setValid(u,v,(bool)tempMatrix.at<cv::Vec3f>(v,u)[2]);
            }
        }
    }
    F_gt_write.write(image_path);
}

void GroundTruth::store_in_yaml(const std::string &temp_flow_path, int frame_count, ushort currentPixelPositionX,
                                ushort currentPixelPositionY, int XMovement, int YMovement ) {

    //store_in_yaml(temp_flow_path, frame_count, currentPixelPositionX, currentPixelPositionX, XMovement,
    // m_movement.y);
    m_fs << "frame_count" << frame_count;
    m_fs << "gt flow png file read" << "[";
    m_fs << "{:" << "row" <<  currentPixelPositionX << "col" << currentPixelPositionY << "displacement" << "[:";
    m_fs << XMovement;
    m_fs << YMovement;
    m_fs << 1;
    m_fs << "]" << "}";
    m_fs << "]";
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

    char file_name[50];
    char xFlow[100];
    char yFlow[100];

    std::vector<unsigned> x_pts;
    std::vector<double> y_pts;
    std::vector<unsigned> z_pts;
    std::vector<float> time;
    double sum_time = 0;

    std::vector<boost::tuple<std::vector<unsigned>, std::vector<double>> > pts_exectime;

    bool needToInit = true;
    std::vector<cv::Point2f> prev_pts;
    std::vector<cv::Point2f> next_pts;
    cv::Mat curGray, prevGray;

    std::cout << "results will be stored in " << resultordner << std::endl;

    if ( frame_types == video_frames) {
        cv::VideoCapture cap;
        cap.open(m_base_directory_path_input_in.string() + "image_02/movement.avi");
        if (!cap.isOpened()) {
            std::cout << "Could not initialize capturing...\n";
            return;
        }
    }
    std::string results_flow_matrix_str = resultordner + "flow_occ_01/result_flow.yaml";

    cv::FileStorage fs;
    fs.open(results_flow_matrix_str, cv::FileStorage::WRITE);


    cv::Size_<unsigned> frame_size(1242,375);
    cv::VideoWriter video_out;

    if ( frame_types == video_frames)
    {
        boost::filesystem::path video_out_path = dataset_path.string() + resultordner + std::string("/video/OpticalFlow.avi");
        assert(boost::filesystem::exists(video_out_path.parent_path()) != 0);
        //frame_size.height =	(unsigned) cap.get(CV_CAP_PROP_FRAME_HEIGHT );
        //frame_size.width =	(unsigned) cap.get(CV_CAP_PROP_FRAME_WIDTH );
        video_out.open(video_out_path.string(),CV_FOURCC('D','I','V','X'), 5, frame_size);
        printf("Writer eingerichtet\n");
    }

    cv::Mat frame = cv::Mat::zeros(frame_size, CV_8UC3);
    cv::Mat flowImage(cv::Size(1242,375), CV_32FC3, cv::Scalar(255,255,255));

    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
    cv::Size subPixWinSize(10, 10), winSize(21, 21);

    const int MAX_COUNT = 5000;
    cv::Mat pyramid1, pyramid2;

    pyramid1.create(frame_size, CV_8UC1);
    pyramid2.create(frame_size, CV_8UC1);

    ushort frame_count = 0;

    cv::namedWindow(resultordner, CV_WINDOW_AUTOSIZE);

    //how many interations(frames)?
    auto tic = steady_clock::now();
    auto toc = steady_clock::now();

    ushort collision = 0, iterator = 0, sIterator = 0;
    std::vector<ushort> xPos, yPos;


    cv::Mat flow_frame( frame_size, CV_8UC3 );

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

    while ( true ) {

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

        //cap >> frame;
        //if (frame.empty())
        //    break;

        fs << "frame_count" << frame_count;

        sprintf(file_name, "000%03d_10.png", frame_count);
        std::string temp_image_path = m_base_directory_path_input_in.string() + "/image_02/" + file_name;

        frame = cv::imread(temp_image_path, CV_LOAD_IMAGE_COLOR);

        if ( frame.data == NULL ) {
            std::cerr << temp_image_path << " not found" << std::endl;
            throw ("No image file found error");
        }

        std::string temp_result_flow_path = m_base_directory_path_result_out.string() + "/" + resultordner +
                "/flow_occ_01/" + file_name;

        // Convert to grayscale
        cv::cvtColor(frame, curGray, cv::COLOR_BGR2GRAY);

        //printf("%u, %u , %u, %u, %u\n", x, start, iterator, secondstart, sIterator);

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
                cv::calcOpticalFlowFarneback(prevGray, curGray, flow_frame, pyrScale, numLevels, windowSize,
                                             numIterations,
                                             neighborhoodSize, stdDeviation, cv::OPTFLOW_USE_INITIAL_FLOW);

                // Draw the optical calculate_flow map
                int stepSize = 16;

                // Draw the uniform grid of points on the input image along with the motion vectors
                for (int y = 0; y < frame.rows; y += stepSize) {
                    for (int x = 0; x < frame.cols; x += stepSize) {
                        // Circles to indicate the uniform grid of points
                        cv::circle(frame, cv::Point(x, y), 1, cv::Scalar(0, 0, 0), -1, 8);

                        // Lines to indicate the motion vectors
                        cv::Point2f pt = flow_frame.at<cv::Point2f>(y, x);
                        cv::arrowedLine(frame, cv::Point(x, y), cv::Point(cvRound(x + pt.x), cvRound(y + pt.y)),
                                        cv::Scalar(0,
                                                   255, 0));

                        int row_coordinate = (int)(y);
                        int col_coordinate = (int)(x);
                        float Vx = (cvRound(x+pt.x) - x);
                        float Vy = (cvRound(y+pt.y) - y);
                        if ( Vx != 0 && Vy!= 0) {
                            printf("(iteration %u, coordinates x y (%i,%i) ->  Vx, Vy (%f,%f) \n", frame_count,
                                   row_coordinate, col_coordinate, Vx, Vy);
                            flowImage.at<cv::Vec3f>(row_coordinate, col_coordinate)[0] = cvRound(Vx+0.5);
                            flowImage.at<cv::Vec3f>(row_coordinate, col_coordinate)[1] = cvRound(Vy+0.5);
                            flowImage.at<cv::Vec3f>(row_coordinate, col_coordinate)[2] = 1.0f;
                        }
                    }
                }
            }
            toc = steady_clock::now();
            time_map["FB"] = duration_cast<milliseconds>(toc - tic).count();
            y_pts.push_back(time_map["FB"]);
        }

        else if (lk == algo) {
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

                std::ofstream flowX;
                flowX.open(xFlow);

                std::ofstream flowY;
                flowY.open(yFlow);
                for (unsigned i = 0; i < next_pts.size(); i++) {
                    /* If the new point is within 'minDist' distance from an existing point, it will not be tracked */
                    // auto dist = cv::norm(prev_pts[i] - next_pts[i]);
                    double x = prev_pts[i].x - next_pts[i].x;
                    double y = prev_pts[i].y - next_pts[i].y;
                    double dist;
                    dist = pow(x,2)+pow(y,2);           //calculating distance by euclidean formula
                    dist = sqrt(dist);                  //sqrt is function in math.h
                    if ( dist <= minDist && frame_count != 1 ) { // frame_count = 1 is a hack because the first and
                        // the second frame is identical and hence the flow distance will always be 0, leading to
                        // errors.
                        printf("minimum distance for %i is %f\n", i, dist);
                        continue;
                    }

                    if(noise == 0) {
                        sprintf(xFlow, "../../FlowTextFiles/Slow/no_noise/FlowX/x%03d.txt", frame_count);
                        sprintf(yFlow, "../../FlowTextFiles/Slow/no_noise/FlowY/y%03d.txt", frame_count);
                    }

                    if(noise == 1){
                        sprintf(xFlow, "../../FlowTextFiles/Slow/static_BG/FlowX/x%03d.txt", frame_count);
                        sprintf(yFlow, "../../FlowTextFiles/Slow/static_BG/FlowY/y%03d.txt", frame_count);
                    }

                    if(noise == 2){
                        sprintf(xFlow, "../../FlowTextFiles/Slow/static_FG/FlowX/x%03d.txt", frame_count);
                        sprintf(yFlow, "../../FlowTextFiles/Slow/static_FG/FlowY/y%03d.txt", frame_count);
                    }

                    if(noise == 3){
                        sprintf(xFlow, "../../FlowTextFiles/Slow/dynamic_BG/FlowX/x%03d.txt", frame_count);
                        sprintf(yFlow, "../../FlowTextFiles/Slow/dynamic_BG/FlowY/y%03d.txt", frame_count);
                    }

                    if(noise == 4){
                        sprintf(xFlow, "../../FlowTextFiles/Slow/dynamic_FG/FlowX/x%03d.txt", frame_count);
                        sprintf(yFlow, "../../FlowTextFiles/Slow/dynamic_FG/FlowY/y%03d.txt", frame_count);
                    }



                    for (int k = 0; k < prev_pts.size(); k++) {
                        flowX << (int) prev_pts[k].x << " " << next_pts[k].x - prev_pts[k].x << std::endl;
                        flowY << (int) prev_pts[k].y << " " << next_pts[k].y - prev_pts[k].y << std::endl;
                    }
                    // Check if the status vector is good
                    if (!status[i])
                        continue;

                    next_pts[count++] = next_pts[i];
                    //cv::circle(frame, next_pts[count], 3, cv::Scalar(0, 255, 0), -1, 8);
                    cv::arrowedLine(frame, prev_pts[i], next_pts[i], cv::Scalar(0, 255, 0), 1, CV_AA, 0);

                    int row_coordinate = std::abs(cvRound(next_pts[i].y));
                    int col_coordinate = std::abs(cvRound(next_pts[i].x));
                    float Vx = (next_pts[i].x - prev_pts[i].x);
                    float Vy = (next_pts[i].y - prev_pts[i].y);
                    printf("(iteration %u, x y (%i,%i) -> ( Vx, Vy)(%f,%f) \n", frame_count, row_coordinate,
                           col_coordinate, Vx, Vy);
                    if(row_coordinate < 375 && col_coordinate < 1242) {
                        flowImage.at<cv::Vec3f>(row_coordinate, col_coordinate)[0] = cvRound(Vx);
                        flowImage.at<cv::Vec3f>(row_coordinate, col_coordinate)[1] = cvRound(Vy);
                        flowImage.at<cv::Vec3f>(row_coordinate, col_coordinate)[2] = 1.0f;
                    }
                }
                next_pts.resize(count);
                //printf(" new size is %i for frame number %u\n", count, frame_count);
                z_pts.push_back(count);
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
                //std::swap(currentPoint, next_pts);
            }

            toc = steady_clock::now();
            time_map["LK"] = duration_cast<milliseconds>(toc - tic).count();
            y_pts.push_back(time_map["LK"]);
            time.push_back(duration_cast<milliseconds>(toc - tic).count());
        }

        if (prevGray.data) {
            //Create png Matrix with 3 channels: x displacement. y displacment and Validation bit
            FlowImage F_result_write(frame_size.width, frame_size.height);
            for (int32_t v=0; v<frame_size.height; v++) { // rows
                for (int32_t u=0; u<frame_size.width; u++) {  // cols
                    if (flowImage.at<cv::Vec3f>(v,u)[2] > 0.5 ) {
                        F_result_write.setFlowU(u,v,flowImage.at<cv::Vec3f>(v,u)[0]);
                        F_result_write.setFlowV(u,v,flowImage.at<cv::Vec3f>(v,u)[1]);
                        F_result_write.setValid(u,v,(bool)flowImage.at<cv::Vec3f>(v,u)[2]);
                    }
                }
            }
            F_result_write.write(temp_result_flow_path);

            FlowImage F_result_read;
            F_result_read.read(temp_result_flow_path);

            for (int32_t v=0; v<F_result_read.height(); v++) { // rows
                for (int32_t u=0; u<F_result_read.width(); u++) {  // cols
                    if ( F_result_read.isValid(u,v) ) {
                        fs << "png file read" << "[";
                        fs << "{:" << "row" <<  v << "col" << u << "displacement" << "[:";
                        fs << F_result_read.getFlowU(u,v);
                        fs << F_result_read.getFlowV(u,v);
                        fs << F_result_read.isValid(u,v);
                        fs << "]" << "}";
                        fs << "]";
                    }
                }
            }
        }


        tic = steady_clock::now();

        auto end = steady_clock::now();

        x_pts.push_back(frame_count);


        if ( frame_types == video_frames) {
            video_out.write(frame);
        }

        // Display the output image
        cv::imshow(resultordner, frame);
        needToInit = false;
        prev_pts.clear();
        std::swap(next_pts, prev_pts);
        std::swap(prevGray, curGray);
        frame_count++;
        if ( frame_count == MAX_ITERATION ) {
            break;
        }
    }

    for(auto &n : time)
        sum_time +=n;

    std::cout << "Noise " << noise  << ", Zeit " << sum_time << std::endl;
    std::cout << "time_map LK " << time_map["LK"] << std::endl;

    fs.release();
    auto max = (std::max_element(y_pts.begin(), y_pts.end())).operator*();

    pts_exectime.push_back(boost::make_tuple(x_pts, y_pts));
    if ( frame_types == video_frames) {
        video_out.release();
    }
    cv::destroyAllWindows();

    // gnuplot_2d
    Gnuplot gp2d;
    gp2d << "set xrange [0:" + std::to_string(MAX_ITERATION) + "]\n";
    gp2d << "set yrange [0:" + std::to_string(max*2) + "]\n";
    std::string tmp = std::string(" with lines title ") + std::string("'") + input_image_folder + std::string(" y axis - ms, x axis - frame\n'");
    gp2d << "plot" << gp2d.binFile2d(pts_exectime, "record") << tmp;

}


void GroundTruth::generate_gt_image_and_gt_flow_vires() {

    prepare_gt_data_and_gt_flow_directories();

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
    while (1) {
        vi.readNetwork();

        if (initCounter <= 0)
            vi.checkShm();

        // has an image arrived or do the first frames need to be triggered
        //(first image will arrive with a certain frame delay only)
        if (vi.getHaveImage() || (initCounter-- > 0)) {
            vi.sendRDBTrigger();

        }

        // ok, reset image indicator
        vi.setHaveImage(0);

        usleep(10000);
        std::cout << "getting data from VIRES\n";
    }
}

void GroundTruth::test_kitti_original() {

    // read arguments
    std::string result_sha = "self_generated_OF";
    std::string user_sha = "vagrawal";

    // init notification mail
    Mail *mail;
    mail = new Mail("vagrawal@hs-esslingen.de");
    mail->msg("Thank you for participating in our evaluation!");

    // run evaluation
    //bool success = eval(result_sha,mail);
    //mail->finalize(success,"flow",result_sha,user_sha);
    // send mail and exit
    delete mail;
}


