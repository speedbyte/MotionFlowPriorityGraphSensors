//
// Created by veikas on 27.01.18.
//

#include <iostream>
#include <boost/filesystem/operations.hpp>
#include <opencv2/core/mat.hpp>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <png++/rgb_pixel.hpp>
#include <png++/image.hpp>
#include <vires/vires/viRDBIcd.h>
#include "GroundTruthScene.h"
#include "kbhit.h"
#include "scp.h"
#include "ViresObjects.h"
using namespace std::chrono;


void GroundTruthScene::prepare_directories() {

    char char_dir_append[20];
    boost::filesystem::path path;

    path = Dataset::getGroundTruthPath();
    // delete input_folder completely
    if (boost::filesystem::exists(Dataset::getGroundTruthPath())) {
        system(("rm -rf " + path.string()).c_str()); //
    }
    boost::filesystem::create_directories(path);

    path =  Dataset::getGroundTruthPath().string() + "/image_02";
    boost::filesystem::create_directories(path);

    for (int i = 1; i <= m_list_objects.size(); i++) {
        sprintf(char_dir_append, "%02d", i);
        path = Dataset::getGroundTruthPath().string() +  "/generated/trajectory_obj_" + char_dir_append;
        boost::filesystem::create_directories(path);
    }

    std::cout << "Creating GT Scene directories" << std::endl;
    boost::filesystem::create_directories(Dataset::getGroundTruthPath().string());
    std::cout << "Ending GT Scene directories" << std::endl;

}


void GroundTruthSceneInternal::generate_gt_scene(void) {

    prepare_directories();

    /*
     * First create an object with an image_data_and_shape
     * Then define the object trajectory
     * Then copy the object image_data_and_shape on the object trajectory points
     * Then store the image in the ground truth image folder
     *
     * Then extrapolate the object trajectory with the above image_data_and_shape
     *
     * Then store the flow information in the flow folder
     */

    //m_shapes.process();
    //m_trajectories.process(Dataset::getFrameSize());

    cv::Mat tempGroundTruthImage;
    tempGroundTruthImage.create(Dataset::getFrameSize(), CV_32FC3);
    assert(tempGroundTruthImage.channels() == 3);

    cv::Mat tempGroundTruthTrajectory;
    tempGroundTruthTrajectory.create(Dataset::getFrameSize(), CV_32FC3);
    assert(tempGroundTruthTrajectory.channels() == 3);
    tempGroundTruthTrajectory = cv::Scalar::all(255);

    cv::Mat tempGroundTruthTrajectory_2;
    tempGroundTruthTrajectory_2.create(Dataset::getFrameSize(), CV_32FC3);
    assert(tempGroundTruthTrajectory_2.channels() == 3);
    tempGroundTruthTrajectory_2 = cv::Scalar::all(255);


    std::map<std::string, double> time_map = {{"generate",0},{"ground truth", 0}};

    std::cout << "ground truth images will be stored in " << Dataset::getGroundTruthPath().string() << std::endl;

    auto tic = steady_clock::now();
    auto toc = steady_clock::now();


    for ( int i = 0; i < m_list_objects.size(); i++ ) {
        printf("registered objects name %s with object id %u\n", m_list_objects.at(i).getObjectName().c_str(),
                       m_list_objects.at(i).getObjectId());
     }

    char file_name_image[50];

    cv::Mat image_data_and_shape;
    cv::Mat trajectoryShape;

    const ushort frame_skip = 1; // image is generated only once irrespective of skips.

    for (ushort frame_count = 0; frame_count < MAX_ITERATION_GT; frame_count++) {

        sprintf(file_name_image, "000%03d_10.png", frame_count*frame_skip);
        std::string input_image_file_with_path = Dataset::getGroundTruthPath().string() + "/image_02/" +
                file_name_image;

        tempGroundTruthImage = m_canvas.getImageShapeAndData().get().clone();

        //draw new ground truth image.

        char frame_skip_folder_suffix[50];

        for ( unsigned  i = 0; i < m_list_objects.size(); i++ ) {

            sprintf(frame_skip_folder_suffix, "generated/trajectory_obj_%02d", m_list_objects.at(i).getObjectId());
            std::string trajectory_image_file_with_path = Dataset::getGroundTruthPath().string() + "/" +
                    frame_skip_folder_suffix + "/" + file_name_image;

            image_data_and_shape = m_list_objects.at(i).getImageShapeAndData().get().clone();
            trajectoryShape = m_list_objects.at(i).getImageShapeAndData().get().clone();

            if ( ( m_list_objects.at(i).get_obj_base_visibility().at(frame_count))
                    ) {
                image_data_and_shape.copyTo(tempGroundTruthImage(
                        cv::Rect(cvRound(m_list_objects.at(i).getTrajectoryPoints()
                                                 .getTrajectory().at(frame_count).x),
                                 cvRound(m_list_objects.at(i).getTrajectoryPoints()
                                         .getTrajectory().at(frame_count).y), image_data_and_shape.cols,
                                 image_data_and_shape.rows)));


                if (m_list_objects.at(i).getObjectId() == 1) {
                    trajectoryShape = cv::Scalar(255, 0, 0);
                    trajectoryShape.copyTo(tempGroundTruthTrajectory(
                            cv::Rect(cvRound(m_list_objects.at(i).getTrajectoryPoints()
                                                     .getTrajectory().at(frame_count).x), cvRound(m_list_objects.at
                                    (i).getTrajectoryPoints().getTrajectory().at(frame_count).y), image_data_and_shape.cols, image_data_and_shape.rows)));
                    cv::imwrite(trajectory_image_file_with_path, tempGroundTruthTrajectory);
                }

                if (m_list_objects.at(i).getObjectId() == 2) {
                    trajectoryShape = cv::Scalar(0, 255, 0);
                    trajectoryShape.copyTo(tempGroundTruthTrajectory_2(
                            cv::Rect(cvRound(m_list_objects.at(i).getTrajectoryPoints()
                                                     .getTrajectory().at(frame_count).x), cvRound(m_list_objects.at(i).getTrajectoryPoints()
                                                                                                                                               .getTrajectory().at(frame_count).y), image_data_and_shape.cols, image_data_and_shape.rows)));
                    cv::imwrite(trajectory_image_file_with_path, tempGroundTruthTrajectory_2);
                }
            }
        }

        toc = steady_clock::now();
        time_map["generate"] = duration_cast<milliseconds>(toc - tic).count();
        cv::imwrite(input_image_file_with_path, tempGroundTruthImage);

    }

    time_map["ground truth"] = duration_cast<milliseconds>(toc - tic).count();
    std::cout << "ground truth generation time - " << time_map["ground truth"] << "ms" << std::endl;

}

void GroundTruthSceneExternal::generate_gt_scene() {

    char command[1024];

    std::cout << "ground truth images will be stored in " << Dataset::getGroundTruthPath().string() << std::endl;

    std::string project = "Movement";

    sprintf(command, "cd %s../../ ; bash vtdSendandReceive.sh %s %s", (Dataset::getBasePath().string()).c_str(),
            project.c_str(), m_scenario.c_str());
    std::cout << command << std::endl;
    system(command);

    std::cout << " I am out of bash" << std::endl;

    //Framework::ViresInterface vi;
    std::string m_server;
    boost::filesystem::path m_ts_gt_out_dir;

    int initCounter = 6;

    // initalize the server variable
    std::string serverName = "127.0.0.1";

    setServer(serverName.c_str());

    fprintf(stderr, "ValidateArgs: key = 0x%x, checkMask = 0x%x, mForceBuffer = %d\n",
            getShmKey(), getCheckMask(), getForceBuffer());

    bool connected_trigger_port = false;
    bool connected_module_manager_port = false;
    bool connected_scp_port = false;

    int scpSocket = openNetwork(SCP_DEFAULT_PORT);
    std::cout << "scp socket - " << scpSocket << std::endl;
    if ( scpSocket != -1 )  { // this is blocking until the network has been opened
        connected_scp_port = true;
    }

    sendSCPMessage(scpSocket, "<SimCtrl><Apply/></SimCtrl>"); // keine hochkommas.

    sleep(5); // Give some time before you start.

    sendSCPMessage(scpSocket, project_name);
    sleep(1);

    std::string::size_type position = scenario_name.find("truck.xml");
    if ( position != std::string::npos ) {
        scenario_name.replace(position, std::string("truck.xml").length(), std::string("traffic_demo.xml"));
    }

    readScpNetwork(scpSocket);

    sendSCPMessage(scpSocket, scenario_name.c_str());
    sleep(1);

    sendSCPMessage(scpSocket, module_manager);
    sleep(1);

    // open the network connection to the taskControl (so triggers may be sent)
    fprintf(stderr, "creating network connection....\n");
    int triggerSocket = openNetwork(DEFAULT_PORT);
    std::cout << "trigger socket - " <<  triggerSocket << std::endl;
    if ( triggerSocket != -1 )  { // this is blocking until the network has been opened
        connected_trigger_port = true;
    }

    int moduleManagerSocket = openNetwork(DEFAULT_RX_PORT);
    std::cout << "mm socket - " << moduleManagerSocket << std::endl;
    if ( moduleManagerSocket != -1 )  { // this is blocking until the network has been opened
        connected_module_manager_port = true;
    }


    if ( connected_trigger_port && connected_module_manager_port && connected_scp_port ) {
        // now: open the shared memory (try to attach without creating a new segment)
        fprintf(stderr, "attaching to shared memory 0x%x....\n", getShmKey());

        while (!getShmPtr()) {
            openShm();
            usleep(1000);     // do not overload the CPU
        }

        fprintf(stderr, "...attached! Reading now...\n");

        // now check the SHM for the time being
        bool breaking = false;
        int count = 0;
        while (1) {

            // Break out of the loop if the user presses the Esc key
            int c = kbhit();

            switch (c) {
                case 9:
                    breaking = true;
                    break;
                default:
                    break;
            }

            if (breaking) {
                break;
            }

            if (getSimFrame() > MAX_ITERATION_GT) {
                breaking = true;
            }

            readNetwork(moduleManagerSocket);  // this calls parseRDBMessage() in vires_common.cpp

            if (initCounter <= 0)
                checkShm();

            // has an image arrived or do the first frames need to be triggered
            //(first image will arrive with a certain image_02_frame delay only)
            if (getHaveImage() || (initCounter-- > 0)) {
                sendRDBTrigger(triggerSocket);
                std::cout << getSimFrame() << std::endl;
            }
            // ok, reset image indicator
            setHaveImage(0);

            usleep(10000); // sleep for 10 ms
            std::cout << "getting data from VIRES\n";
        }
    }

    sprintf(command, "cd %s; %s", (Dataset::getBasePath().string() + std::string("../../")).c_str(),
            "bash vtdStop.sh");
    std::cout << command << std::endl;
    system(command);
}


void GroundTruthSceneExternal::parseStartOfFrame(const double &simTime, const unsigned int &simFrame) {
    fprintf(stderr, "I am in GroundTruthFlow %d\n,", RDB_PKG_ID_START_OF_FRAME);
    fprintf(stderr, "RDBHandler::parseStartOfFrame: simTime = %.3f, simFrame = %d\n", simTime, simFrame);
}

void GroundTruthSceneExternal::parseEndOfFrame(const double &simTime, const unsigned int &simFrame) {
    fprintf(stderr, "headers %d\n,", RDB_PKG_ID_END_OF_FRAME);
    fprintf(stderr, "RDBHandler::parseEndOfFrame: simTime = %.3f, simFrame = %d\n", simTime, simFrame);
}

void GroundTruthSceneExternal::parseEntry(RDB_OBJECT_CFG_t *data, const double &simTime, const unsigned int &
simFrame, const
                                          unsigned short &pkgId, const unsigned short &flags,
                                          const unsigned int &elemId, const unsigned int &totalElem) {

    RDB_OBJECT_CFG_t *object = reinterpret_cast<RDB_OBJECT_CFG_t *>(data); /// raw image data
    std::cout << object->type;

}

void GroundTruthSceneExternal::parseEntry(RDB_OBJECT_STATE_t *data, const double &simTime, const unsigned int &
simFrame, const
                                          unsigned
                                          short &pkgId, const unsigned short &flags, const unsigned int &elemId,
                                          const unsigned int &totalElem) {

    RDB_OBJECT_STATE_t *object = reinterpret_cast<RDB_OBJECT_STATE_t *>(data); /// raw image data
//        fprintf( stderr, "handleRDBitem: handling object state\n" );
//        fprintf( stderr, "    simTime = %.3lf, simFrame = %d\n", simTime, simFrame );
//        fprintf( stderr, "    object = %s, id = %d\n", data->base.name, data->base.id );
//        fprintf( stderr, "    position = %.3lf / %.3lf / %.3lf\n", data->base.pos.x, data->base.pos.y, data->base
//                .pos.z );
    ViresObjects viresObjects = ViresObjects();
    viresObjects.objectProperties = *object;
    viresObjects.frame_count = simFrame;

    if (strcmp(data->base.name, "New Character") == 0) {

        fprintf(stderr, "INDICATOR: %d %.3lf %.3lf %.3lf %.3lf \n",
                simFrame, data->base.pos.x,
                object->base.pos.y, data->base.geo.dimX, data->base.geo.dimY);
    } else if (strcmp(data->base.name, "New Character01") == 0) {
        fprintf(stderr, "INDICATOR2: %d %.3lf %.3lf %.3lf %.3lf \n",
                simFrame, data->base.pos.x,
                data->base.pos.y, data->base.geo.dimX, data->base.geo.dimY);
    }
}

void GroundTruthSceneExternal::parseEntry(RDB_IMAGE_t *data, const double &simTime, const unsigned int &simFrame,
                                          const
                                          unsigned short &pkgId, const unsigned short &flags,
                                          const unsigned int &elemId, const unsigned int &totalElem) {

    if (!data)
        return;
    fprintf(stderr, "handleRDBitem: image\n");
    fprintf(stderr, "    simTime = %.3lf, simFrame = %d, mLastShmFrame = %d\n", simTime, simFrame, getLastShmFrame());
    fprintf(stderr, "    width / height = %d / %d\n", data->width, data->height);
    fprintf(stderr, "    dataSize = %d\n", data->imgSize);

    // ok, I have an image:
    setHaveImage(1);
    char *image_data_ = NULL;
    RDB_IMAGE_t *image = reinterpret_cast<RDB_IMAGE_t *>(data); /// raw image data

    /// RDB image information of \see image_data_
    RDB_IMAGE_t image_info_;
    memcpy(&image_info_, image, sizeof(RDB_IMAGE_t));

    if (NULL == image_data_) {
        image_data_ = reinterpret_cast<char *>(malloc(image_info_.imgSize));
    } else {
        image_data_ = reinterpret_cast<char *>(realloc(image_data_, image_info_.imgSize));
    }
    // jump data header
    memcpy(image_data_, reinterpret_cast<char *>(image) + sizeof(RDB_IMAGE_t), image_info_.imgSize);

    if (image_info_.imgSize == image_info_.width * image_info_.height * 3) {
        png::image<png::rgb_pixel> save_image(image_info_.width, image_info_.height);
        unsigned int count = 0;
        for (int32_t v = 0; v < image_info_.height; v++) {
            for (int32_t u = 0; u < image_info_.width; u++) {
                png::rgb_pixel val;
                val.red = (unsigned char) image_data_[count++];
                val.green = (unsigned char) image_data_[count++];
                val.blue = (unsigned char) image_data_[count++];
                //val.alpha = (unsigned char)image_data_[count++];
                save_image.set_pixel(u, v, val);
            }
        }

        char file_name_image[500];

        if (simFrame > 0) {
            sprintf(file_name_image, "000%03d_10.png", (simFrame - 7));
            std::string input_image_file_with_path = Dataset::getGroundTruthPath().string() + "/" + file_name_image;
            save_image.write(input_image_file_with_path);
        }
    } else {
        fprintf(stderr, "ignoring file with %d channels\n", image_info_.imgSize / (image_info_
                                                                                           .width *
                                                                                   image_info_.height));
    }
}
