

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
    m_base_directory_path_image_out = m_dataset_path.string() + m_dataordner + std::string("image_02_0/dummy.txt");
    m_base_directory_path_flow_out = m_dataset_path.string() + m_dataordner + std::string("flow_occ_0/dummy.txt");
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
        m_xPos.push_back(static_cast<ushort>((m_frame_size.width/2) + (500 * cos(theta[i] * CV_PI / 180.0) /
                                                                   (1.0 + std::pow(sin(theta[i] * CV_PI / 180.0), 2)))));

        m_yPos.push_back(static_cast<ushort>((m_frame_size.height/2) + (55 * (cos(theta[i] * CV_PI / 180.0) *
                                                                          sin(theta[i] * CV_PI / 180.0)) / (0.2 +std::pow(sin(theta[i] * CV_PI / 180.0),2)))));
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

    m_groundTruthImage.create(m_frame_size, CV_8UC3);
    m_absoluteGroundTruthFlow.create(m_frame_size,CV_32FC3);
    m_absolutePixelLocation.create(m_frame_size,CV_16UC3);
    assert(m_absoluteGroundTruthFlow.channels() == 3);
    assert(m_absolutePixelLocation.channels() == 3);
    m_absoluteGroundTruthFlow = cv::Scalar::all(0);
    m_absolutePixelLocation = cv::Scalar::all(0);

}

void GroundTruth::prepare_gt_dataandflow_directories(int frame_skip) {

    boost::filesystem::path result_dir_path = m_dataset_path;
    result_dir_path = m_dataset_path;
    result_dir_path += m_dataordner;
    if (!m_dataset_path.compare(CPP_DATASET_PATH) || !m_dataset_path.compare(VIRES_DATASET_PATH) ) {

        /* prepare diectory for ground truth synthetic images */
        result_dir_path = m_dataset_path;
        result_dir_path += m_dataordner;

        if ( boost::filesystem::exists(result_dir_path) ) {
            system(("rm -rf " + result_dir_path.string()).c_str()); // data/stereo_flow/
        }

        boost::filesystem::create_directories(result_dir_path.string());
        std::cout << "Creating directories" << std::endl;
        boost::filesystem::create_directories(result_dir_path.string() + "/image_02_" + std::to_string(frame_skip));
        boost::filesystem::create_directories(result_dir_path.string() + "/flow_occ_" + std::to_string(frame_skip));
        std::cout << "Ending directories" << std::endl;
    }
}

void GroundTruth::generate_gt_image_and_gt_flow(void) {

    prepare_gt_dataandflow_directories(0);

    assert(boost::filesystem::exists(m_base_directory_path_image_out.parent_path()) != 0);
    assert(boost::filesystem::exists(m_base_directory_path_flow_out.parent_path()) != 0);

    ushort start=60;

    std::map<std::string, double> time_map = {{"generate",0}, {"ground truth", 0}};

    std::cout << "ground truth images will be stored in " << m_base_directory_path_image_out.parent_path().string() << std::endl;

    cv::Mat test_absolute_frame(m_frame_size, CV_16UC3);


    auto tic = steady_clock::now();
    auto toc = steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();

    //Initialization
    ushort iterator = 0;
    signed short XMovement = 0,YMovement = 0;

    ushort actualX;
    ushort actualY;

    const ushort xOrigin = m_xPos.at(start);  // return row pixel
    const ushort yOrigin = m_yPos.at(start);  // return col pixel

    //for moving the objects later
    actualX = xOrigin;
    actualY = yOrigin;

    char file_name[20];

    cv::FileStorage fs;
    fs.open(m_gt_flow_matrix_str, cv::FileStorage::WRITE);

    tic_all = steady_clock::now();
    for (ushort frame_count=0; frame_count < MAX_ITERATION; frame_count++) {

        //Used to store the GT images for the kitti devkit

        sprintf(file_name, "000%03d_10", frame_count);

        std::string gt_image_path_str = m_base_directory_path_image_out.parent_path().string() + "/" + std::string(file_name) + ".png";
        std::string gt_abs_pixel_location_str = m_base_directory_path_flow_out.parent_path().string() +
                                                "/abs_px_location_" +
                                                std::string(file_name)
                                                + ".png";
        std::string gt_flow_path_str = m_base_directory_path_flow_out.parent_path().string() + "/" + std::string
                                                                                                            (file_name) + ".png";

        printf("%u, %u , %u, %u, %u, %u, %u\n", frame_count, start, iterator, actualX, actualY,
               XMovement, YMovement);

        /*auto circularNext = [&m_xPos](auto it) { ++it; return (it == m_xPos.end()) ? m_xPos.begin() : it; };
        for (auto first = m_xPos.begin() + 59, second = m_xPos.begin() + 60; ; first = circularNext(first), second = circularNext(second)) {
            std::cout << *second - *first;
        }*/

        //If we are at the end of the path vector, we need to reset our iterators
        if ((start+iterator) >= m_xPos.size() ) {
            start = 0;
            iterator = 0;
            XMovement = m_xPos.at(start+iterator) - m_xPos.at(m_xPos.size() - 1);
            YMovement = m_yPos.at(start+iterator) - m_yPos.at(m_yPos.size() - 1);
        } else {
            XMovement = m_xPos.at(start+iterator) - m_xPos.at(start+iterator-(ushort)1);
            YMovement = m_yPos.at(start+iterator) - m_yPos.at(start+iterator-(ushort)1);
        }
        //absolutePixelLocation.at<cv::Vec3s>(m_yPos.at(start+iterator), m_xPos.at(start+iterator))[0] = m_yPos.at(0);
        //absolutePixelLocation.at<cv::Vec3s>(m_yPos.at(start+iterator), m_xPos.at(start+iterator))[1] = m_xPos.at(0);

        tic = steady_clock::now();
        //reset the image to white
        m_groundTruthImage = cv::Scalar::all(255);
        //draw new image.
        m_pedesterianImage.copyTo(m_groundTruthImage(cv::Rect(actualX, actualY, object_width, object_height)));
        toc = steady_clock::now();
        time_map["generate"] =  duration_cast<milliseconds>(toc - tic).count();
        cv::imwrite(gt_image_path_str, m_groundTruthImage);

        // calculating the relative Ground Truth for the Kitti devkit and store it in a png file
        // Displacements between -512 to 512 are allowed. Smaller than -512 and greater than 512 will result in an
        // overflow. The final value to be stored in U16 in the form of val*64+32768

        test_absolute_frame = cv::Scalar::all(255);

        cv::Mat roi;
        roi = m_absoluteGroundTruthFlow.
                colRange(actualX, ( actualX + object_width )).
                rowRange(actualY, (actualY + object_height));

        // store displacement in the matrix
        roi = cv::Scalar((XMovement), (YMovement), 1.0f);

        cv::Mat roi_absolute;
        roi_absolute = m_absolutePixelLocation.
                colRange(actualX, ( actualX + object_width )).
                rowRange(actualY, (actualY + object_height));

        roi_absolute = cv::Scalar(m_yPos.at(start+iterator), m_xPos.at(start+iterator), 1.0f);

        cv::imwrite(gt_abs_pixel_location_str, m_absolutePixelLocation);

        fs << "frame_count" << frame_count;

        //fs << "ground displacement obj1" << "[";
        int row = actualY;
        int col = actualX;
        //fs << "{:" << "row" <<  row << "col" << col << "displacement" << "[:";
        //fs << absoluteGroundTruthFlow.at<cv::Vec3f>(row, col)[0];
        //fs << absoluteGroundTruthFlow.at<cv::Vec3f>(row, col)[1];
        //fs << absoluteGroundTruthFlow.at<cv::Vec3f>(row, col)[2];
        //fs << "]" << "}";
        //fs << "]";
        //fs << "complete" << absoluteGroundTruthFlow;

        //Create png Matrix with 3 channels: x displacement. y displacment and Validation bit
        FlowImage F_gt_write(m_frame_size.width, m_frame_size.height);
        for (int32_t v=0; v<m_frame_size.height; v++) { // rows
            for (int32_t u=0; u<m_frame_size.width; u++) {  // cols
                if (absoluteGroundTruthFlow.at<cv::Vec3f>(v,u)[2] > 0.5 ) {
                    F_gt_write.setFlowU(u,v,absoluteGroundTruthFlow.at<cv::Vec3f>(v,u)[0]);
                    F_gt_write.setFlowV(u,v,absoluteGroundTruthFlow.at<cv::Vec3f>(v,u)[1]);
                    F_gt_write.setValid(u,v,(bool)absoluteGroundTruthFlow.at<cv::Vec3f>(v,u)[2]);
                }
            }
        }
        if ( frame_count > 0 ) {
            F_gt_write.write(gt_flow_path_str);
            FlowImage F_gt_read;
            F_gt_read.read(gt_flow_path_str);

            for (int32_t v=0; v<F_gt_read.height(); v++) { // rows
                for (int32_t u=0; u<F_gt_read.width(); u++) {  // cols
                    if (v == row && u == col) {
                        fs << "png file read" << "[";
                        fs << "{:" << "row" <<  v << "col" << u << "displacement" << "[:";
                        fs << F_gt_read.getFlowU(u,v);
                        fs << F_gt_read.getFlowV(u,v);
                        fs << F_gt_read.isValid(u,v);
                        fs << "]" << "}";
                        fs << "]";
                    }
                }
            }
        }

        //plotVectorField (F_gt_write,m_base_directory_path_image_out.parent_path().string(),file_name);

        iterator++;

        actualX = actualX + XMovement;
        actualY = actualY + YMovement;
        std::cout << "generate frame - " << time_map["generate"]  << "ms" << std::endl;

    }

    fs.release();
    toc_all = steady_clock::now();
    time_map["ground truth"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth generation time - " << time_map["ground truth"]  << "ms" << std::endl;

}

void GroundTruth::generate_gt_image_and_gt_flow_vires() {

    prepare_gt_dataandflow_directories(0);

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


