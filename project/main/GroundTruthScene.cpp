//
// Created by veikas on 27.01.18.
//

#include <iostream>
#include <boost/filesystem/operations.hpp>
#include <opencv2/core/mat.hpp>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <png++/rgb_pixel.hpp>
#include <png++/image.hpp>
#include <vires-interface/Common/viRDBIcd.h>
#include <sys/time.h>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv/cv.hpp>
#include "GroundTruthScene.h"
#include "kbhit.h"
#include "ViresObjects.h"
#include "ObjectMetaData.h"
#include <vires-interface/vires_configuration.h>
#include "Utils.h"

using namespace std::chrono;


void GroundTruthScene::prepare_directories(ushort sensor_group_index) {

    m_groundtruthpath = Dataset::getGroundTruthPath(); // data/stereo_flow

    m_generatepath = m_groundtruthpath.string() + "/" + m_environment;

    if (m_regenerate_yaml_file) {
        if (m_datasetpath.string() == std::string(CPP_DATASET_PATH) || m_datasetpath.string() == std::string(VIRES_DATASET_PATH)) {

            std::cout << "prepare gt_scene directories" << std::endl;

            std::string m_generatepath_sensor = m_generatepath.string() + "_" + std::to_string(sensor_group_index);
            if (boost::filesystem::exists(m_generatepath_sensor)) {
                system(("rm -rf " + m_generatepath_sensor).c_str());
            }
            boost::filesystem::create_directories(m_generatepath_sensor);

            char char_dir_append[20];
            boost::filesystem::path path;

            for (int i = 0; i < m_list_gt_objects.size(); i++) {

                sprintf(char_dir_append, "%02d", i);
                m_position_object_path = m_generatepath_sensor + "position_object_";
                path = m_position_object_path.string() + char_dir_append;
                //boost::filesystem::create_directories(path);
            }

        }
    }
}

void GroundTruthSceneExternal::startEvaluating(std::unique_ptr<Noise> &noise) {


    for (ushort obj_index = 0; obj_index < viresObjects.at(m_evaluation_sensor_list.at(0)).get_ptr_customObjectMetaDataList().size(); obj_index++) {

        GroundTruthObjects gt_obj;

        // how many objects were found. The assumption is that in each frame, the number of objects would be constant.
        for (ushort sensor_group_index = 0; sensor_group_index < m_evaluation_sensor_list.size(); sensor_group_index++ ) {

            std::cout << "send object name " << viresObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index)->getObjectName() << std::endl;

            if ( sensor_group_index == 0 ) {
                gt_obj = GroundTruthObjects(viresObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index)->getObjectShape(),
                        viresObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index)->getObjectStartPoint(), noise,
                        viresObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index)->getObjectName());
                m_list_gt_objects.push_back(gt_obj);
            }
            // each object is monitored by n sensor group.
            m_list_gt_objects.at(obj_index).beginGroundTruthGeneration(sensor_group_index, *viresObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index));

        }
        if ( m_evaluation_sensor_list.size() > 1 ) {
            m_list_gt_objects.at(obj_index).generate_combined_sensor_data();
        }

    }

    for (ushort sen_index = 0; sen_index < viresObjects.at(m_evaluation_sensor_list.at(0)).get_ptr_customSensorMetaDataList().size(); sen_index++) {

        // how many sensors were found. The assumption is that in each frame, the number of sensors would be constant.


        for (ushort sensor_group_index = 0; sensor_group_index < m_evaluation_sensor_list.size(); sensor_group_index++ ) {

            Sensors gt_sen(viresObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customSensorMetaDataList().at(0)->getSensorStartPoint(), noise,
                    viresObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customSensorMetaDataList().at(0)->getSensorName());
            m_list_gt_sensors.push_back(gt_sen);
            // each sensor is monitored by n sensor group.

            m_list_gt_sensors.at(sen_index).beginGroundTruthGeneration(*viresObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customSensorMetaDataList().at(sen_index));

        }

    }

}

void GroundTruthSceneInternal::startEvaluating(std::unique_ptr<Noise> &noise) {


    for (ushort obj_index = 0; obj_index < cppObjects.at(m_evaluation_sensor_list.at(0)).get_ptr_customObjectMetaDataList().size(); obj_index++) {

        GroundTruthObjects gt_obj;

        // how many objects were found. The assumption is that in each frame, the number of objects would be constant.

        for (ushort sensor_group_index = 0; sensor_group_index < m_evaluation_sensor_list.size(); sensor_group_index++ ) {

            std::cout << "send object name " << cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index)->getObjectName() << std::endl;

            if ( sensor_group_index == 0 ) {
                gt_obj = GroundTruthObjects(cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index)->getObjectShape(),
                        cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index)->getObjectStartPoint(), noise,
                        cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index)->getObjectName());
                m_list_gt_objects.push_back(gt_obj);
            }
            // each object is monitored by n sensor group.
            m_list_gt_objects.at(obj_index).beginGroundTruthGeneration(sensor_group_index, *cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index));

        }
        if ( m_evaluation_sensor_list.size() > 1 ) {
            m_list_gt_objects.at(obj_index).generate_combined_sensor_data();
        }

    }

}

void GroundTruthSceneInternal::generate_gt_scene(void) {

    /*
    cv::RNG rng(-1);
    for ( unsigned i = 0 ; i < MAX_ITERATION_THETA; i++ ) {
        float a        = (float) rng.uniform(100., 1000.);
        float b        = (float) rng.uniform(100., 300.);
        cv::Point2f points(a,b);
    }
*/
    std::unique_ptr<Noise> colorfulNoise = std::make_unique<ColorfulNoise>();

    if (m_environment == "blue_sky") {

        cppObjects.push_back(CppObjects(0,m_generatepath));
        cppObjects.push_back(CppObjects(1,m_generatepath));

        if (m_regenerate_yaml_file) {

            for (ushort sensor_group_index = 0; sensor_group_index < m_generation_sensor_list.size(); sensor_group_index++ ) {

                boost::filesystem::remove("../position_cpp_" + std::to_string(sensor_group_index) + ".yml");
                std::cout << "generate_gt_scene at " << m_groundtruthpath.string() << " for " << sensor_group_index << std::endl;

                std::unique_ptr<Noise> noise;
                if (m_environment == "night") {
                    BlackNoise blackNoise;
                    noise = std::make_unique<BlackNoise>(blackNoise);
                } else {
                    WhiteNoise whiteNoise;
                    noise = std::make_unique<WhiteNoise>(whiteNoise);
                }
                cppObjects.at(sensor_group_index).process(noise);
                cppObjects.at(m_generation_sensor_list.at(sensor_group_index)).writePositionInYaml("cpp_");

            }
        } else { // genreate yaml file
            for (ushort sensor_group_index = 0; sensor_group_index < m_generation_sensor_list.size(); sensor_group_index++ ) {
                cppObjects.at(sensor_group_index).readPositionFromFile("cpp_");
                cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).calcBBFrom3DPosition();
            }

            startEvaluating(colorfulNoise);
        }

    }

}

void GroundTruthScene::generate_bird_view() {
    // the bird view needs the range information of each object
    // Assuming the camera is mounted on the floor.
    /*
        for ( auto position_index = 0;  position_index <  MAX_ITERATION_RESULTS; position_index++ ) {

            cv::Mat birdview_frame(Dataset::getFrameSize(), CV_32FC1);
            for ( auto object_index= 0; object_index < get_ptr_customObjectMetaDataList().at(0).size(); object_index++ ) {
                birdview_frame.at(m_list_gt_objects.at(object_index).get_object_base_point_displacement().at(position_index).first.x, m_list_gt_objects.at(object_index).get_object_range().at(position_index)) = 100;
                cv::Mat roi_objects;
                roi_objects = birdview_frame.rowRange(m_list_gt_objects.at(object_index).get_object_range().at(position_index), m_list_gt_objects.at(object_index).get_object_dimension().at(position_index).z_offset )
                        .colRange(m_list_gt_objects.at(object_index).get_object_range().at(position_index), m_list_gt_objects.at(object_index).get_object_dimension().at(position_index).x_offset);

            }
            cv::imwrite("filename", birdview_frame);
            // time to collide with the object.

        }
        */
}

#define DEFAULT_RX_PORT_CAM_0               48182   /* for image port it should be 48192 */
#define DEFAULT_RX_PORT_PERFECT_0           48183   /* for image port it should be 48192 */
#define DEFAULT_RX_PORT_PERFECT_INERTIAL_0  48184   /* for image port it should be 48192 */

#define DEFAULT_RX_PORT_CAM_1               48185   /* for image port it should be 48192 */
#define DEFAULT_RX_PORT_PERFECT_1           48186   /* for image port it should be 48192 */
#define DEFAULT_RX_PORT_PERFECT_INERTIAL_1  48187   /* for image port it should be 48192 */

void GroundTruthSceneExternal::generate_gt_scene() {


    viresObjects.push_back(ViresObjects(0, m_generatepath));
    viresObjects.push_back(ViresObjects(1, m_generatepath));

    if (m_regenerate_yaml_file) { // call VIRES only at the time of generating the files

        configureSensor(0, 0x120a, DEFAULT_RX_PORT_CAM_0, DEFAULT_RX_PORT_PERFECT_0, DEFAULT_RX_PORT_PERFECT_INERTIAL_0, module_manager_libModuleSensor_CameraTemplate_left, module_manager_libModuleSensor_PerfectTemplate_left);
        configureSensor(1, 0x120b, DEFAULT_RX_PORT_CAM_1, DEFAULT_RX_PORT_PERFECT_1, DEFAULT_RX_PORT_PERFECT_INERTIAL_1, module_manager_libModuleSensor_CameraTemplate_right, module_manager_libModuleSensor_PerfectTemplate_right);

        char command[1024];

        std::string project = "Movement";

        std::vector<std::string> list_of_scenarios = {"carTypesComplete.xml", "crossing8Demo.xml",
                                                      "crossing8DualExt.xml",
                                                      "crossing8Static.xml", "HighwayPulk.xml", "invisibleCar.xml",
                                                      "ParkPerp.xml",
                                                      "RouteAndPathShapeSCP.xml",
                                                      "staticCar.xml", "TownActionsPath.xml", "TownPathLong.xml",
                                                      "traffic_demo2Ext.xml",
                                                      "trafficDemoClosePath.xml", "trafficDemoPath.xml",
                                                      "trafficDemoPed.xml", "traffic_demoReverse.xml",
                                                      "trafficDemoTrailer.xml", "trafficDemoUK.xml", "traffic_demo.xml"
                                                        "car.xml", "moving_car_near.xml", "moving_car.xml",
                                                      "moving_truck.xml", "moving.xml", "one.xml",
                                                      "truck.xml", "two.xml"};

        sleep(1); // Wait before starting vtd again.

        /*
        mWidth=1200
        mHeight=400
        mPosX=200
        mPosY=600
        mBorder=true
        mScreen=0
        mVisual=$7
        mViewportLeft=$8
        mViewportRight=$9
        mViewportBottom=$10
        mViewportTop=$11
        */


        //sprintf(command, "cd /local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.1/Data/Setups/Standard_test/Scripts/; bash configureDisplay.sh %d %d 200 600 true 0",
        //        Dataset::getFrameSize().width, Dataset::getFrameSize().height);
        std::cout << command << std::endl;
        system(command);
        std::cout << " configureDisplay for Image Generator - render surface, location, height, etc. Parameters will be fine tunedusing SCP commands" << std::endl;


        //if (m_environment == "blue_sky") {
            sprintf(command, "cd %s../../ ; bash vtdSendandReceive.sh %s", (m_datasetpath.string()).c_str(),
                    project.c_str());
            std::cout << command << std::endl;
            system(command);
            std::cout << " I am out of bash" << std::endl;
        //}
        //else {
        //    sendSCPMessage(m_scpSocket, apply.c_str());
        //}

        sleep(1); // Give some time before you send SCP commands.

        // std::string m_server;
        boost::filesystem::path m_ts_gt_out_dir;


        //fprintf(stderr, "ValidateArgs: key = 0x%x, checkMask = 0x%x, mForceBuffer = %d\n", viresObjects.at(0).getShmKey(), getCheckMask(),
        //        getForceBuffer());

        bool connected_trigger_port = false;
        bool connected_scp_port = false;
        bool connected_module_manager_port = false;

        std::string serverName = "127.0.0.1";
        setServer(serverName.c_str());
        m_scpSocket = openNetwork(SCP_DEFAULT_PORT);

        std::cout << "scp socket - " << m_scpSocket << std::endl;
        if (m_scpSocket != -1) { // this is blocking until the network has been opened
            connected_scp_port = true;
        }

        sleep(1); // Give some time before you send the next SCP command.


        // end of autoConfig ! autoconfig automatically sends Apply
        sleep(1);

        // start of autoStart ! autoStart calls tcAutoStart.sh implicitly. But my tcAutoStart is empty. I send scpCommands from vtdRunScp.sh

        sendSCPMessage(m_scpSocket, stop.c_str());
        sleep(1);

        sprintf(command, "cd %s../../ ; bash vtdRunScp.sh", (m_datasetpath.string()).c_str());
        std::cout << command << std::endl;
        system(command);

        sendSCPMessage(m_scpSocket, project_name.c_str());

        sleep(1);

        //sendSCPMessage(m_scpSocket, image_generator.c_str());
        //sleep(1);

        sendSCPMessage(m_scpSocket, rdbtrigger_portnumber.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, scenario_name.c_str());

        sleep(1);

        for ( ushort sensor_group_index = 0; sensor_group_index < m_generation_sensor_list.size(); sensor_group_index++ ) {

            for ( ushort j = 0; j < 3; j++ ) {
                std::cout << sensor_group.at(m_generation_sensor_list.at(sensor_group_index)).at(j).get<0>().c_str();
                sendSCPMessage(m_scpSocket, sensor_group.at(m_generation_sensor_list.at(sensor_group_index)).at(j).get<0>().c_str());
                sleep(1);
            }
        }

        //sendSCPMessage(m_scpSocket, view_parameters_sensorpoint_openglfrustum.c_str());
        //sleep(1);

        //sendSCPMessage(m_scpSocket, view_parameters_sensorpoint_intrinsicparams_left.c_str());
        //sleep(1);

        sendSCPMessage(m_scpSocket, view_parameters_sensorpoint_intrinsicparams_left.c_str());
        sleep(1);

        sendSCPMessage(m_scpSocket, view_parameters_sensorpoint_intrinsicparams_right.c_str());
        sleep(1);

        sendSCPMessage(m_scpSocket, display_parameters_left.c_str());

        sleep(1);

        //sendSCPMessage(m_scpSocket, display_parameters_right.c_str());
        //sleep(1);


        sendSCPMessage(m_scpSocket, m_environment_scp_message.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, message_scp.c_str());

        sleep(1);

        //sendSCPMessage(m_scpSocket, popup_scp.c_str());

        //sleep(1);

        sendSCPMessage(m_scpSocket, eyepoint.c_str());

        sleep(1);


        sendSCPMessage(m_scpSocket, elevation.c_str());

        sleep(1);

        //sendSCPMessage(m_scpSocket, bbox.c_str());

        sleep(1);

        //readScpNetwork(m_scpSocket);

        //sleep(1);

        // open the network connection to the taskControl (so triggers may be sent)
        fprintf(stderr, "creating network connection....\n");

        m_triggerSocket = openNetwork(DEFAULT_PORT);

        std::cout << "trigger socket - " << m_triggerSocket << std::endl;
        if (m_triggerSocket != -1) { // this is blocking until the network has been opened
            connected_trigger_port = true;
        }


        for (ushort sensor_group_index = 0; sensor_group_index < m_generation_sensor_list.size(); sensor_group_index++ ) {

            if ( m_environment == "blue_sky") {
                viresObjects.at(m_generation_sensor_list.at(sensor_group_index)).openAllFileHandles();
            }

            for (ushort j = 0 ; j < 3; j++ ) {

                int socket   = openNetwork(sensor_group.at(m_generation_sensor_list.at(sensor_group_index)).at(j).get<2>());

                std::cout << "mm socket - " << socket << std::endl;
                if (    socket != -1) {
                    connected_module_manager_port = true;
                    sensor_group.at(m_generation_sensor_list.at(sensor_group_index)).at(j).get<4>() = socket;
                }
                else {
                    connected_module_manager_port = false;
                    break;
                }
            }
        }


        if (connected_trigger_port && connected_module_manager_port && connected_scp_port) {


            for (ushort sensor_group_index = 0; sensor_group_index < m_generation_sensor_list.size(); sensor_group_index++ ) {
                // open the shared memory for IG image output (try to attach without creating a new segment)
                fprintf(stderr, "openCommunication: attaching to shared memory (IG image output) 0x%x....\n", sensor_group.at(sensor_group_index).at(0).get<3>());
                void *shmPtr = openShm(sensor_group.at(m_generation_sensor_list.at(sensor_group_index)).at(0).get<3>());
                sensor_group.at(m_generation_sensor_list.at(sensor_group_index)).at(0).get<5>() = shmPtr;
                usleep(1000);     // do not overload the CPU
            }

            // now check the SHM for the time being
            bool breaking = false;
            int count = 0;

            try {
                while (1) {

                    if (viresObjects.at(m_generation_sensor_list.at(0)).getBreaking()) {
                        break;
                    }


                    for (ushort sensor_group_index = 0; sensor_group_index < m_generation_sensor_list.size(); sensor_group_index++ ) {

                        bool getGroundTruthImages = true;
                        viresObjects.at(m_generation_sensor_list.at(sensor_group_index)).getGroundTruthInformation(sensor_group.at(m_generation_sensor_list.at(sensor_group_index)).at(0).get<5>(), ((sensor_group_index==0)?true:false), m_triggerSocket, (m_environment == "blue_sky"), getGroundTruthImages,
                        sensor_group.at(m_generation_sensor_list.at(sensor_group_index)).at(0).get<4>(), sensor_group.at(m_generation_sensor_list.at(sensor_group_index)).at(1).get<4>(), sensor_group.at(m_generation_sensor_list.at(sensor_group_index)).at(2).get<4>());
                    }

                    usleep(100000); // wait, 100 ms which is equivalent to 10 Hz. Normally VIRES runs with 60 Hz. So this number should not be a problem.
                    //std::cout << "getting data from VIRES\n";
                }
            }
            catch (...) {
                std::cerr << "Error in generation" << std::endl;
                stopSimulation();
                return;
            };

            stopSimulation();
            //configVires();
        }
        else {
            stopSimulation();
        }

        Noise noNoise;

        if (m_environment == "blue_sky") {

            try {
                for ( ushort sensor_group_index = 0 ; sensor_group_index < m_generation_sensor_list.size() ; sensor_group_index++) {

                    viresObjects.at(m_generation_sensor_list.at(sensor_group_index)).closeAllFileHandles();

                    viresObjects.at(m_generation_sensor_list.at(sensor_group_index)).readObjectStateFromBinaryFile("vires_");
                    viresObjects.at(m_generation_sensor_list.at(sensor_group_index)).readSensorObjectFromBinaryFile("vires_");
                    viresObjects.at(m_generation_sensor_list.at(sensor_group_index)).readSensorStateFromBinaryFile("vires_");

                    viresObjects.at(m_generation_sensor_list.at(sensor_group_index)).writePositionInYaml("vires_");
                    //system("diff ../position_vires_original_15_65.yml ../position_vires_0.yml");
                }

            }
            catch (...) {
                std::cerr << "VTD Generation complete, but error in generating images" << std::endl;
                stopSimulation();
            }

        }

    } else {

        std::unique_ptr<Noise> noNoise = std::make_unique<NoNoise>();

        if (m_environment == "blue_sky") {

            for ( ushort sensor_group_index = 0 ; sensor_group_index < m_evaluation_sensor_list.size() ; sensor_group_index++) {

                viresObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).readObjectStateFromBinaryFile("vires_");
                viresObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).readSensorObjectFromBinaryFile("vires_");
                viresObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).readSensorStateFromBinaryFile("vires_");

                viresObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).writePositionInYaml("vires_");

                viresObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).readPositionFromFile("vires_");

                viresObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).calcBBFrom3DPosition();
            }

            startEvaluating(noNoise);
        }
    }

}

double GroundTruthSceneExternal::getTime() {
    struct timeval tme;
    gettimeofday(&tme, 0);

    double now = tme.tv_sec + 1.0e-6 * tme.tv_usec;

    if (mStartTime < 0.0)
        mStartTime = now;

    return now;
}

void GroundTruthSceneExternal::calcStatistics() {
    double now = getTime();

    double dt = now - mStartTime;

    if (dt < 1.e-6)
        return;

    fprintf(stderr,
            "calcStatistics: received %d/%d images in %.3lf seconds (i.e. %.3lf/%.3lf images per second ), total number of errors = %d\n",
            mTotalNoImages, dt, mTotalNoImages / dt, mTotalErrorCount);
}

/**
 * @brief calcBBFrom3DPosition
 *
 * For the camera sensor the GT data from VTD is slightly shifted. Therefore we try to get better
 * bounding boxes by calculating the bounding box on the image from the 3D data delivered by the perfect
 * sensor.
 * @param screen_width      width of VTD display
 * @param screen_height     height of VTD display
 * @param cam_pos           position of camera relative to vehicle point of origin
 * @param fov_v             vertical fov of the VTD camera
 * @param pixSize           pixel size
 */

