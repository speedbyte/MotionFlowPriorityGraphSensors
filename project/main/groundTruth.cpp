

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
    assert(boost::filesystem::exists(m_base_directory_path_image_out.parent_path()) != 0);
    m_base_directory_path_flow_out = m_dataset_path.string() + m_dataordner + std::string("flow_occ_0/dummy.txt");
    assert(boost::filesystem::exists(m_base_directory_path_flow_out.parent_path()) != 0);
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

    ushort start=60;

    std::map<std::string, double> time_map = {{"generate",0}, {"ground truth", 0}};

    std::cout << "ground truth images will be stored in " << m_base_directory_path_image_out.parent_path().string() << std::endl;

    cv::Mat relativeGroundTruth(m_frame_size,CV_32FC3,cv::Scalar(0,0,0));
    cv::Mat absolutePixelLocation(m_frame_size,CV_16UC3,cv::Scalar(0,0,0));

    cv::Mat test_frame = cv::Mat::zeros(m_frame_size, CV_8UC3);
    cv::Mat test_absolute_frame = cv::Mat::zeros(m_frame_size, CV_16UC3);

    auto tic = steady_clock::now();
    auto toc = steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();

    //Initialization
    ushort iterator = 0;
    short XMovement = 0,YMovement = 0;

    ushort actualX;
    ushort actualY;



    ushort xOrigin = m_xPos.at(start);  // return row pixel
    ushort yOrigin = m_yPos.at(start);  // return col pixel

    //for moving the objects later
    actualX = xOrigin;
    actualY = yOrigin;

    test_frame = cv::Scalar((rand()%255),(rand()%255),0);
    char file_name[20];


    cv::FileStorage fs;
    fs.open(m_gt_flow_matrix_str, cv::FileStorage::WRITE);

    tic_all = steady_clock::now();
    for (ushort frame_count=0; frame_count < MAX_ITERATION; frame_count++) {

        //Used to store the GT images for the kitti devkit
        relativeGroundTruth = cv::Scalar::all(0);
        absolutePixelLocation = cv::Scalar::all(0);

        sprintf(file_name, "000%03d_10", frame_count);

        std::string gt_image_path_str = m_base_directory_path_image_out.parent_path().string() + "/" + std::string(file_name) + ".png";
        std::string gt_abs_pixel_location_str = m_base_directory_path_flow_out.parent_path().string() +
                                                "/abs_px_location_" +
                                                std::string(file_name)
                                                + ".png";
        std::string gt_flow_path_str = m_base_directory_path_flow_out.parent_path().string() + "/" + std::string
                                                                                                            (file_name) + ".png";

        printf("%u, %u , %u, %u, %u, %u\n", frame_count, start, iterator, actualX, actualY,
               XMovement);

        //If we are at the end of the path vector, we need to reset our iterators
        if ((iterator+start) >= m_xPos.size()) {
            start = 0;
            iterator = 0;
            XMovement = m_xPos.at(0) - m_xPos.at(m_xPos.size() - 1);
            YMovement = m_yPos.at(0) - m_yPos.at(m_yPos.size() - 1);
            absolutePixelLocation.at<cv::Vec3s>(m_yPos.at(0), m_xPos.at(0))[0] = m_xPos.at(0);
            absolutePixelLocation.at<cv::Vec3s>(m_yPos.at(0), m_xPos.at(0))[1] = m_yPos.at(0);
        } else {
            XMovement = m_xPos.at(start+iterator) - m_xPos.at(start+iterator-(ushort)1);
            YMovement = m_yPos.at(start+iterator) - m_yPos.at(start+iterator-(ushort)1);
            absolutePixelLocation.at<cv::Vec3s>(m_yPos.at(start+iterator), m_xPos.at(start+iterator))[0] = m_xPos.at
                    (start+iterator);
            absolutePixelLocation.at<cv::Vec3s>(m_yPos.at(start+iterator), m_xPos.at(start+iterator))[1] = m_yPos.at
                    (start+iterator);
        }

        //Object specification
        std::vector<ushort> XSpec;
        for ( ushort i = 0; i < object_width; i++) {
            XSpec.push_back(actualX+i);
        }

        std::vector<ushort> YSpec;
        for ( ushort i = 0; i < object_height; i++) {
            YSpec.push_back(actualY+i);
        }

        // create the frame
        tic = steady_clock::now();

        uchar r = 0;
        uchar b = 0;

        //reset the image to white
        test_frame = cv::Scalar::all(255);
        test_absolute_frame = cv::Scalar::all(255);

        //draw new image.
        r = 0;
        b = 0;
        for (int k = YSpec.at(0); k < YSpec.at(YSpec.size() - 1); k++) {
            for (int j = XSpec.at(0); j < XSpec.at(XSpec.size() - 1); j++) {
                test_frame.at<cv::Vec3b>(k, j)[0] = b;
                test_frame.at<cv::Vec3b>(k, j)[1] = 0;
                test_frame.at<cv::Vec3b>(k, j)[2] = r;
                r = r + (uchar)2;
                b = b + (uchar)2;
                if (r > 254)
                    r = 130;
            }
            if (b > 254)
                b = 46;
        }

        toc = steady_clock::now();
        time_map["generate"] =  duration_cast<milliseconds>(toc - tic).count();

        cv::imwrite(gt_image_path_str, test_frame);

        // calculating the relative Ground Truth for the Kitti devkit and store it in a png file
        // Displacements between -512 to 512 are allowed. Smaller than -512 and greater than 512 will result in an
        // overflow. The final value to be stored in U16 in the form of val*64+32768

        assert(relativeGroundTruth.channels() == 3);
        assert(absolutePixelLocation.channels() == 3);

        cv::Mat roi;
        roi = relativeGroundTruth.
                colRange(XSpec.at(0), XSpec.at(XSpec.size()-1)).
                rowRange(YSpec.at(0), YSpec.at(YSpec.size()-1));

        // store displacement in the matrix
        roi = cv::Scalar((XMovement), (YMovement), 1.0f);

        cv::Mat roi_absolute;
        roi_absolute = absolutePixelLocation.
                colRange(m_xPos.at(start+iterator)-(object_width/2),m_xPos.at(start+iterator)+(object_width/2)).
                rowRange(m_yPos.at(start+iterator)-(object_height/2),m_yPos.at(start+iterator)+(object_height/2));

        roi_absolute = cv::Scalar(m_xPos.at(start+iterator), m_yPos.at(start+iterator), 1.0f);

        cv::imwrite(gt_abs_pixel_location_str, test_absolute_frame);

        fs << "frame_count" << frame_count;

        //fs << "ground displacement obj1" << "[";
        int row = YSpec.at(0);
        int col = XSpec.at(0);
        //fs << "{:" << "row" <<  row << "col" << col << "displacement" << "[:";
        //fs << relativeGroundTruth.at<cv::Vec3f>(row, col)[0];
        //fs << relativeGroundTruth.at<cv::Vec3f>(row, col)[1];
        //fs << relativeGroundTruth.at<cv::Vec3f>(row, col)[2];
        //fs << "]" << "}";
        //fs << "]";
        //fs << "complete" << relativeGroundTruth;

        //Create png Matrix with 3 channels: x displacement. y displacment and Validation bit
        FlowImage F_gt_write(m_frame_size.width, m_frame_size.height);
        for (int32_t v=0; v<m_frame_size.height; v++) { // rows
            for (int32_t u=0; u<m_frame_size.width; u++) {  // cols
                if (relativeGroundTruth.at<cv::Vec3f>(v,u)[2] > 0.5 ) {
                    F_gt_write.setFlowU(u,v,relativeGroundTruth.at<cv::Vec3f>(v,u)[0]);
                    F_gt_write.setFlowV(u,v,relativeGroundTruth.at<cv::Vec3f>(v,u)[1]);
                    F_gt_write.setValid(u,v,(bool)relativeGroundTruth.at<cv::Vec3f>(v,u)[2]);
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


