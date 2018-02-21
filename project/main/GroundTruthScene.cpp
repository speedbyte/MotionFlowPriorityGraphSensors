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
#include <vires-interface/Common/viRDBIcd.h>
#include <sys/time.h>
#include "GroundTruthScene.h"
#include "kbhit.h"
#include "ViresObjects.h"
using namespace std::chrono;

void GroundTruthScene::prepare_directories() {


    m_groundtruthpath = Dataset::getGroundTruthPath(); // data/stereo_flow

    m_generatepath = m_groundtruthpath.string() + "/" +  m_environment + "/";

    if (!m_datasetpath.string().compare(CPP_DATASET_PATH) || !m_datasetpath.string().compare(VIRES_DATASET_PATH)) {

        std::cout << "prepare gt_scene directories" << std::endl;

        if (boost::filesystem::exists(m_generatepath)) {
            system(("rm -rf " + m_generatepath.string()).c_str());
        }
        boost::filesystem::create_directories(m_generatepath);

        char char_dir_append[20];
        boost::filesystem::path path;

        for (int i = 0; i < m_list_objects.size(); i++) {

            sprintf(char_dir_append, "%02d", i);
            m_trajectory_obj_path = m_generatepath.string() + "trajectory_obj_";
            path = m_trajectory_obj_path.string() + char_dir_append;
            boost::filesystem::create_directories(path);

        }

    }
}

void GroundTruthScene::setTrajectory(std::vector<cv::Point2f> trajectory, MyTrajectory mytrajectory) {



}

std::vector<cv::Point2f> GroundTruthScene::readTrajectoryFromFile(std::string trajectoryFileName) {
    cv::FileStorage trajectory;
    trajectory.open(trajectoryFileName, cv::FileStorage::APPEND);
    trajectory.release();

}

void GroundTruthSceneInternal::generate_gt_scene(void) {


    // Trajectories
    MyTrajectory myTrajectory2;
    MyTrajectory myTrajectory1;
    MyTrajectory myTrajectory;

    std::vector<cv::Point2f> trajectory = readTrajectoryFromFile("filename");
    setTrajectory(trajectory, myTrajectory1);



    /*
    myTrajectory1.pushTrajectoryPoints(cv::Point2f(50,25));
    myTrajectory1.pushTrajectoryPoints(cv::Point2f(100,50));
    myTrajectory1.pushTrajectoryPoints(cv::Point2f(150,75));
    myTrajectory1.pushTrajectoryPoints(cv::Point2f(200,100));
    myTrajectory1.pushTrajectoryPoints(cv::Point2f(250,125));

    myTrajectory2.pushTrajectoryPoints(cv::Point2f(700,250));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f(800,225));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f(900,200));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f(1000,175));
    myTrajectory2.pushTrajectoryPoints(cv::Point2f(1100,150));
*/
    /*
    cv::RNG rng(-1);
    for ( unsigned i = 0 ; i < MAX_ITERATION_THETA; i++ ) {
        float a        = (float) rng.uniform(100., 1000.);
        float b        = (float) rng.uniform(100., 300.);
        cv::Point2f points(a,b);
        myTrajectory1.pushTrajectoryPoints(points);
        myTrajectory2.pushTrajectoryPoints(points);
    }
*/
    //std::cout << myTrajectory1.getTrajectory();

    Achterbahn achterbahn1, achterbahn2;
    achterbahn1.process(Dataset::getFrameSize());
    //achterbahn1.setDynamic();
    achterbahn2.process(Dataset::getFrameSize());
    //achterbahn2.setDynamic();

    Rectangle rectangle1(5, 5); // width, height
    Rectangle rectangle2(20,70); // width, height
    //Rectangle myShape(5, 5); // width, height
    //Circle circle;
    //Ramp ramp;
    //NegativeRamp negativeRamp;

    ColorfulNoise colorfulNoise;
    NoNoise noNoise;

    GroundTruthObjects obj2(rectangle2, achterbahn1, 60, colorfulNoise, "rectangle_long");
    GroundTruthObjects obj3(rectangle2, achterbahn2, 120, colorfulNoise, "random_object");

    //GroundTruthObjects obj2(rectangle2, myTrajectory1, 0, colorfulNoise, "right_man");
    //GroundTruthObjects obj3(rectangle2, myTrajectory2, 0, colorfulNoise, "left_man");


    //list_of_gt_objects.push_back(obj1);
    m_list_objects.push_back(obj2);
    m_list_objects.push_back(obj3);

    //list_of_gt_objects.push_back(obj3);
    //list_of_gt_objects.push_back(obj4);
    //list_of_gt_objects.push_back(obj5);
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

    prepare_directories();

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


    std::map<std::string, double> time_map = {{"generate_single_scene_image",0},{"generate_all_scene_image", 0}};

    std::cout << "generate_gt_scene at " << m_groundtruthpath.string() << std::endl;

    char file_name_image[50];

    cv::Mat image_data_and_shape;
    cv::Mat trajectoryShape;

    const ushort frame_skip = 1; // image is generated only once irrespective of skips.

    auto tic_all = steady_clock::now();

    for (ushort frame_count = 0; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_IMAGES; frame_count++) {

        auto tic = steady_clock::now();

        sprintf(file_name_image, "000%03d_10.png", frame_count*frame_skip);
        std::string input_image_file_with_path = m_generatepath.string() + file_name_image;

        tempGroundTruthImage = m_canvas.getImageShapeAndData().get().clone();

        //draw new ground truth image.

        char frame_skip_folder_suffix[50];

        for ( unsigned  i = 0; i < m_list_objects.size(); i++ ) {

            sprintf(frame_skip_folder_suffix, "%02d", m_list_objects.at(i).getObjectId());
            std::string trajectory_image_file_with_path = m_trajectory_obj_path.string() +
                    frame_skip_folder_suffix + "/" + file_name_image;

            image_data_and_shape = m_list_objects.at(i).getImageShapeAndData().get().clone();
            trajectoryShape = m_list_objects.at(i).getImageShapeAndData().get().clone();

            if ( ( m_list_objects.at(i).get_obj_base_visibility().at(frame_count))
                    ) {

                image_data_and_shape.copyTo(tempGroundTruthImage(
                        cv::Rect(cvRound(m_list_objects.at(i).get_obj_base_pixel_point_pixel_displacement().at(frame_count).first.x),
                                 cvRound(m_list_objects.at(i).get_obj_base_pixel_point_pixel_displacement().at(frame_count).first.y),
                                image_data_and_shape.cols,
                                image_data_and_shape.rows)));


                if (m_list_objects.at(i).getObjectId() == 0) {
                    trajectoryShape = cv::Scalar(255, 0, 0);
                    trajectoryShape.copyTo(tempGroundTruthTrajectory(
                            cv::Rect(cvRound(m_list_objects.at(i).getTrajectoryPoints()
                                                     .getTrajectory().at(frame_count).x), cvRound(m_list_objects.at
                                    (i).getTrajectoryPoints().getTrajectory().at(frame_count).y), image_data_and_shape.cols, image_data_and_shape.rows)));
                    cv::imwrite(trajectory_image_file_with_path, tempGroundTruthTrajectory);
                }

                if (m_list_objects.at(i).getObjectId() == 1) {
                    trajectoryShape = cv::Scalar(0, 255, 0);
                    trajectoryShape.copyTo(tempGroundTruthTrajectory_2(
                            cv::Rect(cvRound(m_list_objects.at(i).getTrajectoryPoints()
                                                     .getTrajectory().at(frame_count).x), cvRound(m_list_objects.at(i).getTrajectoryPoints()
                                                                                                                                               .getTrajectory().at(frame_count).y), image_data_and_shape.cols, image_data_and_shape.rows)));
                    cv::imwrite(trajectory_image_file_with_path, tempGroundTruthTrajectory_2);
                }
            }
        }

        cv::imwrite(input_image_file_with_path, tempGroundTruthImage);
        auto toc = steady_clock::now();
        time_map["generate_single_scene_image"] = duration_cast<milliseconds>(toc - tic).count();

    }

    auto toc_all = steady_clock::now();
    time_map["generate_all_scene_image"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth scene generation time - " << time_map["generate_all_scene_image"] << "ms" << std::endl;

}

void GroundTruthSceneExternal::generate_gt_scene() {

    prepare_directories();

    char command[1024];

    std::string project = "Movement";

    std::vector<std::string> list_of_scenarios = {"carTypesComplete.xml", "crossing8Demo.xml", "crossing8DualExt.xml",
            "crossing8Static.xml", "HighwayPulk.xml", "invisibleCar.xml", "ParkPerp.xml", "RouteAndPathShapeSCP.xml",
            "staticCar.xml", "TownActionsPath.xml", "TownPathLong.xml", "traffic_demo2Ext.xml",
            "trafficDemoClosePath.xml", "trafficDemoPath.xml", "trafficDemoPed.xml", "traffic_demoReverse.xml",
            "trafficDemoTrailer.xml", "trafficDemoUK.xml", "traffic_demo.xml"
                    "car.xml", "moving_car_near.xml", "moving_car.xml", "moving_truck.xml", "moving.xml", "one.xml",
            "truck.xml", "two.xml"};

    sprintf(command, "cd %s../../ ; bash vtdSendandReceive.sh %s", (m_datasetpath.string()).c_str(), project.c_str());
    std::cout << command << std::endl;
    system(command);

    std::cout << " I am out of bash" << std::endl;

    sleep(5); // Give some time before you send SCP commands.

    // std::string m_server;
    boost::filesystem::path m_ts_gt_out_dir;

    int initCounter = 6;

    // initalize the server variable
    std::string serverName = "127.0.0.1";

    setServer(serverName.c_str());

    fprintf(stderr, "ValidateArgs: key = 0x%x, checkMask = 0x%x, mForceBuffer = %d\n", mShmKey, getCheckMask(),
            getForceBuffer());

    bool connected_trigger_port = false;
    bool connected_module_manager_port = false;
    bool connected_scp_port = false;

    int scpSocket = openNetwork(SCP_DEFAULT_PORT);
    std::cout << "scp socket - " << scpSocket << std::endl;
    if (scpSocket != -1) { // this is blocking until the network has been opened
        connected_scp_port = true;
    }

    sleep(1); // Give some time before you send the next SCP command.

    sendSCPMessage(scpSocket, apply.c_str());

    sleep(5); // This is very important !! Mimimum 5 seconds of wait, till you start the simulation

    sendSCPMessage(scpSocket, project_name.c_str());

    sleep(1);

    sendSCPMessage(scpSocket, rdbtrigger_portnumber.c_str());

    sleep(1);

    sendSCPMessage(scpSocket, scenario_name.c_str());

    sleep(1);

    sendSCPMessage(scpSocket, module_manager.c_str());

    sleep(1);

    sendSCPMessage(scpSocket, camera_parameters.c_str());

    sleep(1);

    sendSCPMessage(scpSocket, display_parameters.c_str());

    sleep(2);

    sendSCPMessage(scpSocket, m_environment_scp_message.c_str());

    sleep(1);

    sendSCPMessage(scpSocket, message_scp.c_str());

    sleep(1);

    //sendSCPMessage(scpSocket, popup_scp.c_str());

    //sleep(1);

    sendSCPMessage(scpSocket, eyepoint.c_str());

    sleep(1);


    sendSCPMessage(scpSocket, elevation.c_str());

    sleep(1);

    sprintf(command, "cd %s../../ ; bash vtdRunScp.sh", (m_datasetpath.string()).c_str());
    std::cout << command << std::endl;
    system(command);

    sleep(10);  // Give some time before you start the trigger and module manager ports.

    //readScpNetwork(scpSocket);

    //readScpNetwork(scpSocket);

    //sleep(1);


    // open the network connection to the taskControl (so triggers may be sent)
    fprintf(stderr, "creating network connection....\n");
    int triggerSocket = openNetwork(DEFAULT_PORT);
    std::cout << "trigger socket - " << triggerSocket << std::endl;
    if (triggerSocket != -1) { // this is blocking until the network has been opened
        connected_trigger_port = true;
    }

    int moduleManagerSocket = openNetwork(DEFAULT_RX_PORT);
    std::cout << "mm socket - " << moduleManagerSocket << std::endl;
    if (moduleManagerSocket != -1) { // this is blocking until the network has been opened
        connected_module_manager_port = true;
    }

    if (connected_trigger_port && connected_module_manager_port && connected_scp_port) {
        // open the shared memory for IG image output (try to attach without creating a new segment)
        fprintf(stderr, "openCommunication: attaching to shared memory (IG image output) 0x%x....\n", mShmKey);

        while (!getShmPtr()) {
            openShm(mShmKey);
            usleep(1000);     // do not overload the CPU
        }

        // now check the SHM for the time being
        bool breaking = false;
        int count = 0;

        try {
            while (1) {

                // Break out of the loop if the user presses the Esc key
                /*
                int c = kbhit();

                switch (c) {
                    case 9:
                        breaking = true;
                        break;
                    default:
                        break;
                } */

                if (breaking) {
                    break;
                }

                if (mSimFrame > MAX_ITERATION_GT_SCENE_GENERATION_DYNAMIC) {
                    breaking = true;
                }

                int lastSimFrame = mLastNetworkFrame;

                readNetwork(moduleManagerSocket);  // this calls parseRDBMessage() in vires_common.cpp

                if (lastSimFrame < 0) {
                    checkShm();  //empty IG buffer of spurious images
                }

                bool haveNewFrame = (lastSimFrame != mLastNetworkFrame);

                // now read IG output
                if (mHaveImage)
                    fprintf(stderr, "main: checking for IG image\n");

                while (mCheckForImage) {
                    checkShm();

                    mCheckForImage = !mHaveImage;

                    usleep(10);

                    if (!mCheckForImage) {
                        //fprintf( stderr, "main: got it!\n" );
                    }
                }

                if (haveNewFrame) {
                    //fprintf( stderr, "main: new simulation frame (%d) available, mLastIGTriggerFrame = %d\n",
                    //                 mLastNetworkFrame, mLastIGTriggerFrame );

                    mHaveFirstFrame = true;
                }

                // has an image arrived or do the first frames need to be triggered
                //(first image will arrive with a certain image_02_frame delay only)


                if (!mHaveFirstImage || mHaveImage || haveNewFrame || !mHaveFirstFrame) {
                    // do not initialize too fast
                    if (!mHaveFirstImage || !mHaveFirstFrame)
                        usleep(100000);   // 10Hz

                    bool requestImage = (mLastNetworkFrame >= (mLastIGTriggerFrame + IMAGE_SKIP_FACTOR_DYNAMIC));

                    if (requestImage) {
                        mLastIGTriggerFrame = mLastNetworkFrame;
                        mCheckForImage = true;
                    }

                    //fprintf( stderr, "sendRDBTrigger: sending trigger, deltaT = %.4lf, requestImage = %s\n", mDeltaTime,
                    //         requestImage ? "true" : "false" );
                    sendRDBTrigger(triggerSocket, mSimTime, mSimFrame, requestImage, mDeltaTime);

                    // increase internal counters
                    mSimTime += mDeltaTime;
                    mSimFrame++;

                    // calculate the timing statistics
                    if (mHaveImage)
                        //calcStatistics();

                        mHaveImage = false;
                }

                usleep(10000); // sleep for 10 ms
                //std::cout << "getting data from VIRES\n";
            }
        }
        catch (...) {
            sprintf(command, "cd %s../../ ; bash vtdStop.sh", (m_datasetpath.string()).c_str());
            std::cout << command << std::endl;
            system(command);
            std::cout << "End of generation" << std::endl;
            return;
        };

        try {

            Noise noNoise;
            Rectangle myShape(40,40);
            GroundTruthObjects character(myShape, myTrajectoryVector.at(0), 0, noNoise, "New Character");
            GroundTruthObjects character_01(myShape, myTrajectoryVector.at(1), 0, noNoise, "New Character01");

            m_list_objects.push_back(character);
            m_list_objects.push_back(character_01);

        }
        catch (...)
        {
            sprintf(command, "cd %s../../ ; bash vtdStop.sh", (m_datasetpath.string()).c_str());
            std::cout << command << std::endl;
            system(command);
            std::cout << "End of generation" << std::endl;

        }

    }

    sprintf(command, "cd %s../../ ; bash vtdStop.sh", (m_datasetpath.string()).c_str());
    std::cout << command << std::endl;
    system(command);
    std::cout << "End of generation" << std::endl;

}


void GroundTruthSceneExternal::parseStartOfFrame(const double &simTime, const unsigned int &simFrame) {
    //fprintf(stderr, "I am in GroundTruthFlow %d\n,", RDB_PKG_ID_START_OF_FRAME);
    //fprintf(stderr, "RDBHandler::parseStartOfFrame: simTime = %.3f, simFrame = %d\n", simTime, simFrame);
}

void GroundTruthSceneExternal::parseEndOfFrame(const double &simTime, const unsigned int &simFrame) {

    mLastNetworkFrame = simFrame;

    //fprintf(stderr, "headers %d\n,", RDB_PKG_ID_END_OF_FRAME);
    //fprintf(stderr, "RDBHandler::parseEndOfFrame: simTime = %.3f, simFrame = %d\n", simTime, simFrame);
}

/** ------ state of an object (may be extended by the next structure) ------- */
typedef struct
{
    uint32_t            id;                         /**< unique object ID                                              @unit _                                   */
    uint8_t             category;                   /**< object category                                               @unit @link RDB_OBJECT_CATEGORY @endlink  */
    uint8_t             type;                       /**< object type                                                   @unit @link RDB_OBJECT_TYPE     @endlink  */
    uint16_t            visMask;                    /**< visibility mask                                               @unit @link RDB_OBJECT_VIS_FLAG @endlink  */
    char                name[RDB_SIZE_OBJECT_NAME]; /**< symbolic name                                                 @unit _                                   */
    RDB_GEOMETRY_t      geo;                        /**< info about object's geometry                                  @unit m,m,m,m,m,m                         */
    RDB_COORD_t         pos;                        /**< position and orientation of object's reference point          @unit m,m,m,rad,rad,rad                   */
    uint32_t            parent;                     /**< unique ID of parent object                                    @unit _                                   */
    uint16_t            cfgFlags;                   /**< configuration flags                                           @unit @link RDB_OBJECT_CFG_FLAG @endlink  */
    int16_t             cfgModelId;                 /**< visual model ID (configuration parameter)                     @unit _                                   */
} RDB_OBJECT_STATE_BASE_DUMMY_t;

/** ------ extended object data (e.g. for dynamic objects) ------- */
typedef struct
{
    RDB_COORD_t         speed;                      /**< speed and rates                                               @unit m/s,m/s,m/s,rad/s,rad/s,rad/s           */
    RDB_COORD_t         accel;                      /**< acceleration                                                  @unit m/s2,m/s2,m/s2,rad/s2,rad/s2/rad/s2     */
    float               traveledDist;               /**< traveled distance                                             @unit m                                      a */
    uint32_t            spare[3];                   /**< reserved for future use                                       @unit _                                       */
} RDB_OBJECT_STATE_EXT_DUMMY_t;

/** ------ sensor definition and state ------ */
typedef struct
{
    uint32_t    id;                          /**< id of the sensor                                      @unit _                                      */
    uint8_t     type;                        /**< type of the sensor                                    @unit @link RDB_SENSOR_TYPE     @endlink     */
    uint8_t     hostCategory;                /**< category of the object hosting the sensor             @unit @link RDB_OBJECT_CATEGORY @endlink     */
    uint16_t    spare0;                      /**< for future use                                        @unit _                                      */
    uint32_t    hostId;                      /**< unique id of the sensor's host                        @unit _                                      */
    char        name[RDB_SIZE_OBJECT_NAME];  /**< name of the sensor                                    @unit _                                      */
    float       fovHV[2];                    /**< field-of-view (horizontal/vertical)                   @unit rad,rad                                */
    float       clipNF[2];                   /**< clipping ranges (near/far)                            @unit m,m                                    */
    RDB_COORD_t pos;                         /**< position and orientation of sensor's reference point  @unit m,m,m,rad,rad,rad                      */
    RDB_COORD_t originCoordSys;              /**< position and orientation of sensor's coord origin     @unit m,m,m,rad,rad,rad                      */
    float       fovOffHV[2];                 /**< field-of-view offset (horizontal/vertical)            @unit rad, rad                              B */
    int32_t     spare[2];                    /**< for future use                                        @unit _                                      */
} RDB_SENSOR_STATE_DUMMY_t;

/** ------ information about an object registered within a sensor ------ */
typedef struct
{
    uint8_t     category;    /**< object category                                                                @unit @link RDB_OBJECT_CATEGORY    @endlink   */
    uint8_t     type;        /**< object type                                                                    @unit @link RDB_OBJECT_TYPE        @endlink   */
    uint16_t    flags;       /**< sensor object flags                                                            @unit @link RDB_SENSOR_OBJECT_FLAG @endlink   */
    uint32_t    id;          /**< id of the object                                                               @unit _                                       */
    uint32_t    sensorId;    /**< id of the detecting sensor                                                     @unit _                                       */
    double      dist;        /**< distance between object and referring device                                   @unit m                                       */
    RDB_COORD_t sensorPos;   /**< position and orientation of object in sensor coord                             @unit m,m,m,rad,rad,rad                       */
    int8_t      occlusion;   /**< degree of occlusion for viewer (-1 = not valid, 0..127 = 0..100% occlusion)    @unit [-1, 0..127]                            */
    uint8_t     spare0[3];   /**< for future use                                                                 @unit _                                       */
    uint32_t    spare[3];    /**< for future use                                                                 @unit _                                       */
} RDB_SENSOR_OBJECT_DUMMY_t;

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
//        fprintf( stderr, "    simTime = %.3lf, simFrame = %d\n", simTime, simFrame );
    ViresObjects viresObjects = ViresObjects();
    viresObjects.objectProperties = *object;

    if ( ( mSimFrame % IMAGE_SKIP_FACTOR_DYNAMIC== 0 ) && mSimFrame > 1 && data->base.type == RDB_OBJECT_TYPE_PLAYER_PEDESTRIAN) {

        fprintf(stderr, "%s: %d %.3lf %.3lf %.3lf %.3lf \n",
                    data->base.name, simFrame, data->base.pos.x, object->base.pos.y, data->base.geo.dimX, data->base
                            .geo.dimY);

        printf("%d.pushTrajectoryPoints(cv::Point2f((float)%f, (float)%f))\n", data->base.id, data->base.pos.x, data->base.pos.y);
        std::cout << data->base.type;
        myTrajectoryVector.at(data->base.id-3).pushTrajectoryPoints(cv::Point2f((float)data->base.pos.x, (float)
                data->base.pos.y));
    }
    else {
        //std::cout << data->base.type << std::endl;
    }
}

void GroundTruthSceneExternal::parseEntry(RDB_IMAGE_t *data, const double &simTime, const unsigned int &simFrame,
                                          const
                                          unsigned short &pkgId, const unsigned short &flags,
                                          const unsigned int &elemId, const unsigned int &totalElem) {

    if (!data)
        return;
    //fprintf(stderr, "handleRDBitem: image\n");
    //fprintf(stderr, "    simTime = %.3lf, simFrame = %d, mLastShmFrame = %d\n", simTime, simFrame, getLastShmFrame());
    //fprintf(stderr, "    width / height = %d / %d\n", data->width, data->height);
    //fprintf(stderr, "    dataSize = %d\n", data->imgSize);

    // ok, I have an image:

    //analyzeImage(  data  , simFrame, 0 );
    mHaveImage      = true;
    mHaveFirstImage = true;

    fprintf( stderr, "------------------------------------------------------------------------------------\n");
    fprintf( stderr, "simFrame = %d, simTime = %.3f, dataSize = %d\n", mSimFrame, mSimTime, data->imgSize);
    fprintf( stderr, "------------------------------------------------------------------------------------\n");

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

        //fprintf(stderr, "got a RGB image with %d channels\n", image_info_.imgSize / (image_info_.width * image_info_
        //.height));

        char file_name_image[50];


        if (simFrame > 1) {
            sprintf(file_name_image, "000%03d_10.png", mImageCount);
            std::string input_image_file_with_path = m_generatepath.string() + file_name_image;
            save_image.write(input_image_file_with_path);
            mImageCount++;
        }
    } else {
        fprintf(stderr, "ignoring file with %d channels\n", image_info_.imgSize / (image_info_
                                                                                           .width *
                                                                                   image_info_.height));
    }
}

double GroundTruthSceneExternal::getTime()
{
    struct timeval tme;
    gettimeofday(&tme, 0);

    double now = tme.tv_sec + 1.0e-6 * tme.tv_usec;

    if ( mStartTime < 0.0 )
        mStartTime = now;

    return now;
}


void GroundTruthSceneExternal::calcStatistics()
{
    double now = getTime();

    double dt = now - mStartTime;

    if ( dt < 1.e-6 )
        return;

    fprintf( stderr, "calcStatistics: received %d/%d images in %.3lf seconds (i.e. %.3lf/%.3lf images per second ), total number of errors = %d\n",
             mTotalNoImages, dt, mTotalNoImages / dt, mTotalErrorCount );
}

void GroundTruthSceneExternal::analyzeImage( RDB_IMAGE_t* img, const unsigned int & simFrame, unsigned int index )
{
    static unsigned int sLastImgSimFrame =  0;

    if ( !img || ( index > 1 ) )
        return;

    if ( img->id == 0 )
        return;

    fprintf( stderr, "analyzeImage: simframe = %d, index = %d: have image no. %d, size = %d bytes, pixelFormat = %d\n",
             simFrame, index, img->id, img->imgSize, img->pixelFormat );

    if ( img->pixelFormat == RDB_PIX_FORMAT_RGB32F )		// some analysis
    {
        float *imgData = ( float* ) ( ( ( char* ) img ) + sizeof( RDB_IMAGE_t ) );

        for ( int i = 0; i < 10; i++ )	// first 10 pixels
        {
            fprintf( stderr, "r / g / b = %.3f / %.3f / %.3f\n", imgData[0], imgData[1], imgData[2] );
            imgData += 3;
        }
    }
    else if ( img->pixelFormat == RDB_PIX_FORMAT_RGB8 )		// some analysis
    {
        unsigned char *imgData = ( unsigned char* ) ( ( ( char* ) img ) + sizeof( RDB_IMAGE_t ) );

        for ( int i = 0; i < 10; i++ )	// first 10 pixels
        {
            fprintf( stderr, "r / g / b = %d / %d / %d\n", imgData[0], imgData[1], imgData[2] );
            imgData += 3;
        }
    }


    //if ( ( myImg->id > 3 ) && ( ( myImg->id - mLastImageId ) != IMAGE_SKIP_FACTOR_DYNAMIC ) )
    if ( ( simFrame != sLastImgSimFrame ) && ( img->id > 3 ) && ( ( simFrame - sLastImgSimFrame ) != IMAGE_SKIP_FACTOR_DYNAMIC ) )
    {
        fprintf( stderr, "WARNING: parseRDBMessageEntry: index = %d, delta of image ID out of bounds: delta = %d\n", index, simFrame - sLastImgSimFrame );
        mTotalErrorCount++;
    }

    mLastImageId    = img->id;
    mTotalNoImages++;

    sLastImgSimFrame = simFrame;
}
