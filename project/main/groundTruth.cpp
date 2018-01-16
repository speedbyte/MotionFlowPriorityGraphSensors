

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

#include <kitti/mail.h>
#include <kitti/io_flow.h>
#include <vires/vires_common.h>

#include "datasets.h"
#include "groundTruth.h"

//Creating a movement path. The path is stored in a x and y vector

using namespace std::chrono;


GroundTruth::GroundTruth(boost::filesystem::path dataset_path, std::string dataordner) {

    m_dataset_path = dataset_path;
    m_dataordner = dataordner;

    m_base_directory_path_image_out = m_dataset_path.string() + m_dataordner + std::string("image_02/dummy.txt");
    m_base_directory_path_flow_out = m_dataset_path.string() + m_dataordner + std::string("flow_occ_00/dummy.txt");
    m_base_directory_path_video_out = m_base_directory_path_image_out.parent_path();
    m_base_directory_path_video_out += std::string("/movement_video.avi");

    m_gt_flow_matrix_str = m_base_directory_path_flow_out.parent_path().string() + "/gt_flow.yaml";

    m_frame_size.width = 1242;
    m_frame_size.height = 375;

    std::vector<ushort> theta;
    for ( ushort frame_count = 0; frame_count < MAX_ITERATION_THETA; frame_count++) {
        theta.push_back(frame_count);
    }
    // Prepare points
    for ( int i = 0; i< MAX_ITERATION_THETA; i++) {

        m_position.x = (static_cast<ushort>((m_frame_size.width/2) + (500 * cos(theta[i] *
                                                                                                           CV_PI / 180.0) /
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
    m_gt_image_path = m_base_directory_path_image_out.parent_path().string();
    m_gt_flow_path = m_base_directory_path_flow_out.parent_path().string();

    m_position = {0,0};
    m_movement = {0,0};
}

void GroundTruth::prepare_gt_dataandflow_directories() {

    boost::filesystem::path result_dir_path = m_dataset_path;
    result_dir_path += m_dataordner;
    char char_dir_append[20];

    if (!m_dataset_path.compare(CPP_DATASET_PATH) || !m_dataset_path.compare(VIRES_DATASET_PATH) ) {

        if ( boost::filesystem::exists(result_dir_path) ) {
            system(("rm -rf " + result_dir_path.string()).c_str()); // data/stereo_flow/
        }
        boost::filesystem::create_directories(result_dir_path.string());

        std::cout << "Creating directories" << std::endl;
        boost::filesystem::create_directories(result_dir_path.string() + "/image_02");
        for (int i = 0; i < 10; ++i) {
            sprintf(char_dir_append, "%02d", i);
            boost::filesystem::create_directories(result_dir_path.string() + "/flow_occ_" + char_dir_append);
        }
        std::cout << "Ending directories" << std::endl;
    }
}

void GroundTruth::generate_gt_image_and_gt_flow(void) {

    prepare_gt_dataandflow_directories();
    m_fs.open(m_gt_flow_matrix_str, cv::FileStorage::WRITE);

    assert(boost::filesystem::exists(m_base_directory_path_image_out.parent_path()) != 0);
    assert(boost::filesystem::exists(m_base_directory_path_flow_out.parent_path()) != 0);

    cv::Mat tempGroundTruthImage;
    tempGroundTruthImage.create(m_frame_size, CV_8UC3);
    assert(tempGroundTruthImage.channels() == 3);

    const ushort start=60;

    std::map<std::string, double> time_map = {{"generate",0}, {"ground truth", 0}};

    std::cout << "ground truth images will be stored in " << m_base_directory_path_image_out.parent_path().string() << std::endl;

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

        sprintf(file_name, "/000%03d_10.png", frame_count);
        std::string temp_image_path = m_gt_image_path + file_name;

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

    for (ushort frame_count=0; frame_count < MAX_ITERATION; frame_count++) {
        sprintf(file_name, "/000%03d_10.png", frame_count);
        std::string temp_flow_path = m_gt_flow_path + file_name;
        if ( frame_count > 0 ) {
            //draw new ground truth flow.
            extrapolate_objects( cv::Point2i(m_flow_matrix.at(frame_count).first.x, m_flow_matrix.at
                                         (frame_count).first.y),
                                 object_width, object_height, m_flow_matrix.at(frame_count).second.x, m_flow_matrix.at
                            (frame_count).second.y, temp_flow_path );
            //store_in_yaml(temp_flow_path, frame_count, currentPixelPositionX, currentPixelPositionX, XMovement,
            // m_movement.y);
        }
    }

    // plotVectorField (F_gt_write,m_base_directory_path_image_out.parent_path().string(),file_name);
    m_fs.release();
    toc_all = steady_clock::now();
    time_map["ground truth"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth generation time - " << time_map["ground truth"]  << "ms" << std::endl;

}

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

    m_fs << "frame_count" << frame_count;
    m_fs << "gt flow png file read" << "[";
    m_fs << "{:" << "row" <<  currentPixelPositionX << "col" << currentPixelPositionY << "displacement" << "[:";
    m_fs << XMovement;
    m_fs << YMovement;
    m_fs << 1;
    m_fs << "]" << "}";
    m_fs << "]";
}

void GroundTruth::generate_gt_image_and_gt_flow_vires() {

    prepare_gt_dataandflow_directories();

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


