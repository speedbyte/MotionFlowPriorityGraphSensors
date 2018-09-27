//
// Created by veikas on 27.01.18.
//

#ifndef MAIN_GENERATEGROUNDTRUTHSCENE_H
#define MAIN_GENERATEGROUNDTRUTHSCENE_H


#include <map>
#include <unistd.h>
#include "ObjectMetaData.h"
#include "Sensors.h"
#include "GenerateViresObjects.h"
#include "GroundTruthObjects.h"
#include "GenerateCppObjects.h"
#include <boost/tuple/tuple.hpp>

class GroundTruthScene  {

protected:

    boost::filesystem::path  m_groundtruthpath;

    boost::filesystem::path  m_datasetpath;

    boost::filesystem::path m_baseframepath;

    boost::filesystem::path  m_position_object_path;

    std::string m_scenario;

    std::string m_environment;

    cv::FileStorage m_write_fs;

    std::vector<ushort> m_generation_sensor_list;
    std::vector<ushort> m_evaluation_sensor_list;

/*
    std::vector<ObjectMetaData> objectMetaDataList;
    std::vector<std::vector<ObjectMetaData *> > m_ptr_customObjectMetaDataList;
    std::vector<std::vector<SensorMetaData *> > m_ptr_customSensorMetaDataList;
    std::vector<std::map<std::string, ObjectMetaData*> > m_mapObjectNameToObjectMetaData;
    std::vector<std::map<std::string, SensorMetaData*> > m_mapSensorNameToSensorMetaData;
    std::map<unsigned int, std::string> m_mapObjectIdToObjectName;
    std::map<unsigned int, std::string> m_mapSensorIdToSensorName;
    std::vector<SensorMetaData> sensorMetaDataList;

*/



public:

    static boost::filesystem::path  m_ground_truth_color_path;
    static boost::filesystem::path  m_ground_truth_edge_path;
    static boost::filesystem::path  m_ground_truth_flow_path;
    static boost::filesystem::path  m_ground_truth_plot_path;
    static boost::filesystem::path  m_ground_truth_depth_path;
    static boost::filesystem::path  m_ground_truth_framedifference_path;

    GroundTruthScene(std::vector<ushort> generation_sensor_list, std::vector<ushort> evaluation_sensor_list, std::string scenario, std::string environment):m_generation_sensor_list(generation_sensor_list), m_evaluation_sensor_list(evaluation_sensor_list), m_scenario(scenario), m_environment(environment){

        //m_ptr_customObjectMetaDataList = {};
        m_datasetpath = Dataset::m_dataset_basepath;
        m_groundtruthpath = Dataset::m_dataset_gtpath;

    };


    virtual void generate_gt_scene() {
        std::cout << "base implementation of generate_gt_scene()" << std::endl;
        throw;
    };


    virtual void convert_sensor_image_to_object_level(std::unique_ptr<Noise> &noise, std::vector<GroundTruthObjects> &list_of_gt_objects_base, std::vector<Sensors> &list_of_gt_sensors_base) {
        std::cout << "base implementation of convert_sensor_image_to_object_level()" << std::endl;
        throw;
    };

    void generate_bird_view();

    void prepare_scene_directories_others(ushort sensor_group_index);

    void prepare_scene_directories_blue_sky(ushort sensor_group_index);

    virtual void write_gt_scene_data() {throw;};

    virtual void read_gt_scene_data() {throw;};
};

class GroundTruthSceneInternal : public GroundTruthScene {

private:

    std::vector<CppObjects> cppObjects;



public:

    GroundTruthSceneInternal(std::vector<ushort> generation_sensor_lists, std::vector<ushort> evaluation_sensor_lists, std::string scenario, std::string environment) :
    GroundTruthScene(generation_sensor_lists, evaluation_sensor_lists, scenario, environment) {
    }

    void convert_sensor_image_to_object_level(std::unique_ptr<Noise> &noise, std::vector<GroundTruthObjects> &list_of_gt_objects_base, std::vector<Sensors> &list_of_gt_sensors_base) override;

    void generate_gt_scene() override;

    ~GroundTruthSceneInternal(){
        std::cout << "killing previous GroundTruthScene object\n" ;
    }

    void write_gt_scene_data() override;

    void read_gt_scene_data() override;


};

class GroundTruthSceneExternal : public GroundTruthScene, public Framework::ViresConfiguration {


    // <SimCtrl><Stop /></SimCtrl>
    // <Video><Stop /></Video>
    // <SimCtrl><Config /></SimCtrl> // Change to config mode

    // <SimCtrl><Apply /></SimCtrl>
    // übliche kramm...

    // <SimCtrl><Pause /></SimCtrl>
    // <SimCtrl><Step size="1" /></SimCtrl>

    //#define RDB_COORD_TYPE_INERTIAL         0  /**< inertial co-ordinate system              @version 0x0101 */
    //#define RDB_COORD_TYPE_PLAYER           2  /**< player co-ordinate system                @version 0x0100 */
    //#define RDB_COORD_TYPE_SENSOR           3  /**< sensor-specific co-ordinate system       @version 0x0100 */
    //#define RDB_COORD_TYPE_USK              4  /**< universal sensor co-ordinate system      @version 0x0100 */
    //#define RDB_COORD_TYPE_USER             5  /**< relative to a user co-ordinate system    @version 0x0100 */
    //#define RDB_COORD_TYPE_WINDOW           6  /**< window co-ordinates [pixel]              @version 0x0100 */
    //#define RDB_COORD_TYPE_TEXTURE          7  /**< texture co-ordinates [normalized]        @version 0x010C */
    //#define RDB_COORD_TYPE_RELATIVE_START   8  /**< co-ordinate relative to start pos.       @version 0x0110 */
    //#define RDB_COORD_TYPE_GEO              9  /**< geographic co-ordinate                   @version 0x0118 */
    //#define RDB_COORD_TYPE_TRACK           10  /**< track co-ordinate (x=s, y=t )            @version 0x0119 */

private:

    std::vector<std::pair<std::string, cv::Point2f> > mposition_points;

    std::string apply = "<SimCtrl><Apply/></SimCtrl>";

    std::string stop = "<SimCtrl> <Stop/> <LoadScenario filename=\"traffic_demo.xml\" /> <Init mode=\"operation\"/> </SimCtrl>";

    std::string config = "<SimCtrl> <Config/> </SimCtrl>";

    std::string start = "<SimCtrl> <Start/> </SimCtrl>";

    std::string project_name = "<SimCtrl><Project name=\"Movement\" path=\"/local/git/MotionFlowPriorityGrapSensors/VIRES/VTD.2.1/Data/Projects/Current\" parameters=\"RDBTrigger\"/></SimCtrl>";

    std::string scenario_name = "<SimCtrl><UnloadSensors /><LoadScenario filename=\"/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.1/Data/Projects/Current/Scenarios/traffic_demo.xml\" /><Start mode=\"operation\" /></SimCtrl>";

    std::string image_generator = "<ImageGenerator> <Window width=\"" + std::to_string(Dataset::m_frame_size.width) + "\" height=\"" + std::to_string(Dataset::m_frame_size.height) + "\" x=\"" + std::to_string(0) + "\" y=\"" + std::to_string(0) + "\" screen=\"0\" border=\"true\"/></ImageGenerator>";

    std::string image_generator_ = "<ImageGenerator ignore=\"true\"></ImageGenerator>";

    std::string module_manager_libModuleSensor_CameraTemplate_left =
            "<Sensor name=\"Sensor_MM\" type=\"video\" > "
                    "   <Config cameraId=\"1\" verbose=\"true\"/> "
                    "   <Load lib=\"libModuleCameraSensor.so\" path=\"\" persistent=\"true\" /> "
                    "   <Player name=\"MovingCar\"/> "
                    "   <Frustum near=\"1.000000\" far=\"40.000000\" left=\"30.000000\" right=\"30.000000\" bottom=\"20.000000\" top=\"20.000000\" /> "
                    "   <Position dx=\"2.000000\" dy=\"1.000000\" dz=\"1.500000\" dhDeg=\"0.000000\" dpDeg=\"0.000000\" drDeg=\"0.000000\" /> "
                    "   <Origin type=\"usk\" /> "
                    "   <Cull maxObjects=\"10\" enable=\"true\" /> "
                    "   <Port name=\"RDBout\" number=\"65535\" type=\"TCP\" sendEgo=\"false\" /> "
//                    "   <Filter objectType=\"none\" />"
                    "   <Filter objectType=\"pedestrian\" /> "
                    "   <Filter objectType=\"vehicle\" /> "
                    "   <Debug enable=\"false\" detection=\"false\" road=\"false\" position=\"false\" dimensions=\"false\" camera=\"false\" packages=\"false\" culling=\"false\" /> "
                    "</Sensor>";

    std::string module_manager_libModuleSensor_CameraTemplate_right =
            "<Sensor name=\"Sensor_MM\" type=\"video\" > "
                    "   <Config cameraId=\"2\" verbose=\"true\"/> "
                    "   <Load lib=\"libModuleCameraSensor.so\" path=\"\" persistent=\"true\" /> "
                    "   <Player name=\"MovingCar\"/> "
                    "   <Frustum near=\"1.000000\" far=\"40.000000\" left=\"30.000000\" right=\"30.000000\" bottom=\"20.000000\" top=\"20.000000\" /> "
                    "   <Position dx=\"2.000000\" dy=\"-1.000000\" dz=\"1.500000\" dhDeg=\"0.000000\" dpDeg=\"0.000000\" drDeg=\"0.000000\" /> "
                    "   <Origin type=\"usk\" /> "
                    "   <Cull maxObjects=\"10\" enable=\"true\" /> "
                    "   <Port name=\"RDBout\" number=\"65535\" type=\"TCP\" sendEgo=\"false\" /> "
//                    "   <Filter objectType=\"none\" />"
                    "   <Filter objectType=\"pedestrian\" /> "
                    "   <Filter objectType=\"vehicle\" /> "
                    "   <Debug enable=\"false\" detection=\"false\" road=\"false\" position=\"false\" dimensions=\"false\" camera=\"false\" packages=\"false\" culling=\"false\" /> "
                    "</Sensor>";

    std::string module_manager_libModuleSensor_PerfectTemplate_left =
            "<Sensor name=\"Sensor_MM\" type=\"video\" > "
                    "   <Load lib=\"libModulePerfectSensor.so\" path=\"\" persistent=\"true\" /> "
                    "   <Player name=\"MovingCar\"/> "
                    "   <Frustum near=\"1.000000\" far=\"40.000000\" left=\"30.000000\" right=\"30.000000\" bottom=\"20.000000\" top=\"20.000000\" /> "
                    "   <Position dx=\"2.000000\" dy=\"1.000000\" dz=\"1.500000\" dhDeg=\"0.000000\" dpDeg=\"0.000000\" drDeg=\"0.000000\" /> "
                    "   <Origin type=\"usk\" /> "
                    "   <Cull maxObjects=\"10\" enable=\"true\" /> "
                    "   <Port name=\"RDBout\" number=\"65535\" type=\"TCP\" sendEgo=\"false\" /> "
//                    "   <Filter objectType=\"none\" />"
                    "   <Filter objectType=\"pedestrian\" /> "
                    "   <Filter objectType=\"vehicle\" /> "
                    "   <Debug enable=\"false\" detection=\"false\" road=\"false\" position=\"false\" dimensions=\"false\" camera=\"false\" packages=\"false\" culling=\"false\" /> "
                    "</Sensor>";


    std::string module_manager_libModuleSensor_PerfectTemplate_right=
            "<Sensor name=\"Sensor_MM\" type=\"video\" > "
                    "   <Load lib=\"libModulePerfectSensor.so\" path=\"\" persistent=\"true\" /> "
                    "   <Player name=\"MovingCar\"/> "
                    "   <Frustum near=\"1.000000\" far=\"40.000000\" left=\"30.000000\" right=\"30.000000\" bottom=\"20.000000\" top=\"20.000000\" /> "
                    "   <Position dx=\"2.000000\" dy=\"-1.000000\" dz=\"1.500000\" dhDeg=\"0.000000\" dpDeg=\"0.000000\" drDeg=\"0.000000\" /> "
                    "   <Origin type=\"usk\" /> "
                    "   <Cull maxObjects=\"10\" enable=\"true\" /> "
                    "   <Port name=\"RDBout\" number=\"65535\" type=\"TCP\" sendEgo=\"false\" /> "
//                    "   <Filter objectType=\"none\" />"
                    "   <Filter objectType=\"pedestrian\" /> "
                    "   <Filter objectType=\"vehicle\" /> "
                    "   <Debug enable=\"false\" detection=\"false\" road=\"false\" position=\"false\" dimensions=\"false\" camera=\"false\" packages=\"false\" culling=\"false\" /> "
                    "</Sensor>";

    std::string module_manager_libModuleSingleRaySensor =
            "<Sensor name=\"simpleSensor\" type=\"radar\">\n"
                    "   <Load     lib=\"libModuleSingleRaySensor.so\" path=\"\" persistent=\"true\" />\n"
                    "   <Frustum  near=\"0.0\" far=\"50.0\" left=\"1.0\" right=\"1.0\" bottom=\"1.0\" top=\"1.0\" />\n"
                    "   <Cull     maxObjects=\"5\" enable=\"true\" />\n"
                    "   <Port     name=\"RDBout\" number=\"48195\" type=\"UDP\" sendEgo=\"true\" />\n"
                    "   <Player   default=\"true\" />\n"
                    "   <Position dx=\"3.5\" dy=\"0.0\" dz=\"0.5\" dhDeg=\"0.0\" dpDeg=\"0.0\" drDeg=\"0.0\" />\n"
                    "   <Filter   objectType=\"pedestrian\"/>\n"
                    "   <Filter   objectType=\"vehicle\"/>\n"
                    "   <Filter   objectType=\"trafficSign\"/>\n"
                    "   <Debug    enable=\"false\"/>\n"
                    "</Sensor>";

    // Precipitation intensity needs to be > 0 for snow and rain.
    const std::string environment_parameters_dry = "<Environment> <Friction value=\"1.000000\" /> <TimeOfDay value=\"39600\" headlights=\"off\" /> <Sky cloudState=\"0/8\" visibility=\"100000.000000\" /><Precipitation type=\"none\" intensity=\"0.000000\" /><Road state=\"dry\" effectScale=\"0.500000\" /></Environment>";
    //Rain
    const std::string environment_parameters_wet = "<Environment> <Friction value=\"1.000000\" /> <TimeOfDay value=\"39600\" headlights=\"off\" /> <Sky cloudState=\"4/8\" visibility=\"100000.000000\" /><Precipitation type=\"rain\" intensity=\"0.500000\" /><Road state=\"dry\" effectScale=\"0.600000\" /></Environment>";

    std::string view_parameters_tethered_openglfrustum = "<Camera name=\"VIEW_CAMERA\" showOwner=\"false\"> <Frustum near=\"0.100000\" far=\"1501.000000\" fovHor=\"60.000000\" fovVert=\"40.000000\" offsetHor=\"0.000000\" offsetVert=\"0.000000\" /> <PosTether player=\"MovingCar\" distance=\"6.000000\" azimuth=\"0.000000\" elevation=\"0.261799\" slew=\"1\" /> <ViewRelative dh=\"0.000000\" dp=\"0.000000\" dr=\"0.000000\" /><Set /> </Camera>";

    std::string view_parameters_eyepoint_openglfrustum = "<Camera name=\"VIEW_CAMERA\" showOwner=\"false\"> <Frustum near=\"0.100000\" far=\"1501.000000\" fovHor=\"60.000000\" fovVert=\"40.000000\" offsetHor=\"0.000000\" offsetVert=\"0.000000\" /> "
            "<PosEyepoint player=\"MovingCar\" distance=\"6.000000\" azimuth=\"0.000000\" elevation=\"0.261799\" slew=\"1\" /> <ViewRelative dh=\"0.000000\" dp=\"0.000000\" dr=\"0.000000\" /><Set /> </Camera>";

    // I tested and the name of the camera does not matter. What matters is the channel number.
    std::string view_parameters_playerrelative_intrinsicparams_left = "<Camera name=\"cam1\" showOwner=\"false\"><Projection far=\"1501.000000\" focalX=\"" + std::to_string(FOCAL_X) + "\" focalY=\"" + std::to_string(FOCAL_Y) + "\" height=\"" + std::to_string(Dataset::m_frame_size.height) + "\" near=\"0.100000\" principalX=\"" + std::to_string(Dataset::m_frame_size.width/2) + "\" principalY=\"" + std::to_string(Dataset::m_frame_size.height/2) + "\" width=\"" + std::to_string(Dataset::m_frame_size.width) + "\" /><PosRelative player=\"MovingCar\" dx=\"2\" dy=\" 1\" dz=\"1.2\"/><Set/></Camera>";

    std::string view_parameters_eyepoint_intrinsicparams_right   = "<Camera name=\"cam2\" showOwner=\"false\"><Projection far=\"1501.000000\" focalX=\"" + std::to_string(FOCAL_X) + "\" focalY=\"" + std::to_string(FOCAL_Y) + "\" height=\"" + std::to_string(Dataset::m_frame_size.height) + "\" near=\"0.100000\" principalX=\"" + std::to_string(Dataset::m_frame_size.width/2) + "\" principalY=\"" + std::to_string(Dataset::m_frame_size.height/2) + "\" width=\"" + std::to_string(Dataset::m_frame_size.width) + "\" /><PosRelative player=\"MovingCar\" dx=\"2\" dy=\"-1\" dz=\"1.2\"/><Set channel=\"0x2\"/></Camera>";

    std::string view_parameters_sensorpoint_intrinsicparams_left_pl = "<Camera name=\"cam1\" showOwner=\"true\"><Projection far=\"1501.000000\" focalX=\"" + std::to_string(FOCAL_X) + "\" focalY=\"" + std::to_string(FOCAL_Y) + "\" height=\"" + std::to_string(Dataset::m_frame_size.height) + "\" near=\"0.100000\" principalX=\"" + std::to_string(Dataset::m_frame_size.width/2) + "\" principalY=\"" + std::to_string(Dataset::m_frame_size.height/2) + "\" width=\"" + std::to_string(Dataset::m_frame_size.width) + "\" /><PosRelative player=\"MovingCar\" dx=\"2\" dy=\"1\" dz=\"1.2\"/><ViewRelative dh=\"0.000000\" dp=\"0.000000\" dr=\"0.000000\" /><Set /></Camera>";
    std::string view_parameters_sensorpoint_intrinsicparams_right_pl = "<Camera name=\"cam2\" showOwner=\"true\"><Projection far=\"1501.000000\" focalX=\"" + std::to_string(FOCAL_X) + "\" focalY=\"" + std::to_string(FOCAL_Y) + "\" height=\"" + std::to_string(Dataset::m_frame_size.height) + "\" near=\"0.100000\" principalX=\"" + std::to_string(Dataset::m_frame_size.width/2) + "\" principalY=\"" + std::to_string(Dataset::m_frame_size.height/2) + "\" width=\"" + std::to_string(Dataset::m_frame_size.width) + "\" /><PosRelative player=\"MovingCar\" dx=\"2\" dy=\"-1\" dz=\"1.2\"/><ViewRelative dh=\"0.000000\" dp=\"0.000000\" dr=\"0.000000\" /></Camera>";

    std::string view_parameters_sensorpoint_intrinsicparams_left = "<Camera name=\"cam1\" showOwner=\"true\"><Projection far=\"1501.000000\" focalX=\"" + std::to_string(FOCAL_X) + "\" focalY=\"" + std::to_string(FOCAL_Y) + "\" height=\"" + std::to_string(Dataset::m_frame_size.height) + "\" near=\"0.100000\" principalX=\"" + std::to_string(Dataset::m_frame_size.width/2) + "\" principalY=\"" + std::to_string(Dataset::m_frame_size.height/2) + "\" width=\"" + std::to_string(Dataset::m_frame_size.width) + "\" /><PosSensor sensor=\"Sensor_MM_0\" useCamFrustum=\"false\" /><ViewRelative dh=\"0.000000\" dp=\"0.000000\" dr=\"0.000000\" /><Set channel=\"0x1\"/></Camera>";
    std::string view_parameters_sensorpoint_intrinsicparams_right = "<Camera name=\"cam2\" showOwner=\"true\"><Projection far=\"1501.000000\" focalX=\"" + std::to_string(FOCAL_X) + "\" focalY=\"" + std::to_string(FOCAL_Y) + "\" height=\"" + std::to_string(Dataset::m_frame_size.height) + "\" near=\"0.100000\" principalX=\"" + std::to_string(Dataset::m_frame_size.width/2) + "\" principalY=\"" + std::to_string(Dataset::m_frame_size.height/2) + "\" width=\"" + std::to_string(Dataset::m_frame_size.width) + "\" /><PosSensor sensor=\"Sensor_MM_1\" useCamFrustum=\"false\" /><ViewRelative dh=\"0.000000\" dp=\"0.000000\" dr=\"0.000000\" /><Set channel=\"0x2\"/></Camera>";

    std::string view_parameters_sensorpoint_openglfrustum = "<Camera name=\"VIEW_CAMERA\" showOwner=\"false\"> <Frustum near=\"0.100000\" far=\"1501.000000\" fovHor=\"60.000000\" fovVert=\"40.000000\" offsetHor=\"0.000000\" offsetVert=\"0.000000\" /> "
            "<PosSensor sensor=\"Sensor_MM\" useCamFrustum=\"true\" /> <ViewRelative dh=\"0.000000\" dp=\"0.000000\" dr=\"0.000000\" /><Set /> </Camera>";

    std::string display_parameters_left = "<Display>  <SensorSymbols enable=\"false\" sensor=\"Sensor_MM_0\" showCone=\"false\" /> <Database enable=\"true\" streetLamps=\"false\"/> <VistaOverlay enable=\"false\" /> </Display>";

    std::string display_parameters_right = "<Display>  <SensorSymbols enable=\"false\" sensor=\"Sensor_MM_1\" showCone=\"false\" /> <Database enable=\"true\" streetLamps=\"false\"/> <VistaOverlay enable=\"false\" /> </Display>";

    std::string elevation = "<VIL><Imu dbElevation=\"true\" /></VIL>";

    std::string eyepoint = "<VIL><EyepointOffset hDeg=\"0.000000\" pDeg=\"0.000000\" rDeg=\"0.000000\" x=\"0.000000\"y=\"0.000000\" z=\"0.000000\" /></VIL>";

    std::string pause = "<SimCtrl> <Pause/> </SimCtrl>";

    std::string rdbtrigger_portnumber = "<TaskControl><RDB client=\"false\" enable=\"true\" interface=\"eth0\" portRx=\"48190\" portTx=\"48190\" portType=\"TCP\" /></TaskControl>";

    std::string  message_scp = "<Symbol name=\"expl01\" > <Text data=\"Time for snow\" colorRGB=\"0xffff00\" size=\"50.0\" /> <PosScreen x=\"0.01\" y=\"0.05\" /></Symbol>";

    std::string popup_scp = "<Info level=\"info\"> <Message popup=\"true\" text=\"snow!!!!\"/> </Info>";

    std::string bbox = "<Symbol name=\"BBCar\"> <BoundingBox player=\"SpyingCar\" /> </Symbol>";

    /*

   0 "<SimCtrl> <Stop/> <LoadScenario filename="traffic_demo.xml" /> <Init mode="operation"/> </SimCtrl>"$
$
wait "<SimCtrl> <InitDone place="checkInitConfirmation"/> </SimCtrl>"$
$
  +1 "<SimCtrl> <Start/> </SimCtrl>"$
 +2s "<Symbol name="expl01" > <Text data="Auto-start script has been running for 2s now" colorRGB="0xffff00" size="50.0" /> <PosScreen x="0.01" y="0.05" /></Symbol>"$
 +2s "<Symbol name="expl01" > <Text data="Auto-start script has been running for 4s now" colorRGB="0xffff00" size="50.0" /> <PosScreen x="0.01" y="0.05" /></Symbol>"$
 +2s "<Symbol name="expl01" > <Text data="Auto-start script has been running for 6s now" colorRGB="0xffff00" size="50.0" /> <PosScreen x="0.01" y="0.05" /></Symbol>"$
 +2s "<Symbol name="expl01" > <Text data="Auto-start script has been running for 8s now" colorRGB="0xffff00" size="50.0" /> <PosScreen x="0.01" y="0.05" /></Symbol>"$
 +2s "<Symbol name="expl01" duration="1.0" > <Text data="Auto-start script is terminating" colorRGB="0xff0000" size="50.0" /> <PosScreen x="0.01" y="0.05" /></Symbol>"$

     */


    unsigned int mFirstIgnoredFrame = 65535;
    int          mLastNetworkFrame = -1;

    // some stuff for performance measurement
    double       mStartTime;


    int          mTotalNoImages;

    // total number of errors
    unsigned int mTotalErrorCount;

    // image skip factor
    static const unsigned short mImageSkipFactor;


    std::string m_environment_scp_message;

    int m_triggerSocket;

    int m_scpSocket;

    std::vector<std::vector<boost::tuple<std::string, std::string, ushort, ushort, ushort, void*> > > sensor_group;

    std::vector<ViresObjects> viresObjects;



public:

    void convert_sensor_image_to_object_level(std::unique_ptr<Noise> &noise, std::vector<GroundTruthObjects> &list_of_gt_objects_base, std::vector<Sensors> &list_of_gt_sensors_base) override;

    void configVires() {

        sleep(1);

        sendSCPMessage(m_scpSocket, config.c_str());

        close(m_scpSocket);
        close(m_triggerSocket);
        for (ushort i = 0 ; i < m_generation_sensor_list.size() ; i++ ) {
            closeAllSockets(/* send a list of socket numbers*/);
        }

    }

    void stopSimulation() {

        close(m_scpSocket);
        close(m_triggerSocket);
        for (ushort i = 0 ; i < m_generation_sensor_list.size() ; i++ ) {
            closeAllSockets(/* send a list of socket numbers*/);
        }

        char command[1024];
        sprintf(command, "cd %s../../ ; bash vtdStop.sh", (m_datasetpath.string()).c_str());
        std::cout << command << std::endl;
        system(command);
        std::cout << "Stopping simulation" << std::endl;
    }


    void analyzeImage( RDB_IMAGE_t* img, const unsigned int & simFrame, unsigned int index );

    void calcStatistics();

    double getTime();

    GroundTruthSceneExternal(std::vector<ushort> generation_sensor_list, std::vector<ushort> evaluation_sensor_list, std::string scenario, std::string environment) : sensor_group(MAX_ALLOWED_SENSORS), GroundTruthScene(generation_sensor_list, evaluation_sensor_list, scenario, environment) {


        std::string to_replace = "traffic_demo";
        std::string::size_type position = scenario_name.find(to_replace);

        if ( position != std::string::npos ) {
            scenario_name.replace(position, to_replace.length(), std::string(m_scenario));
        }

        std::string::size_type position2 = stop.find(to_replace);
        if ( position2 != std::string::npos ) {
            stop.replace(position2, to_replace.length(), std::string(m_scenario));
        }

        if ( environment == "blue_sky") {
            m_environment_scp_message = environment_parameters_dry;
        }

        else if ( environment == "light_snow"  ) {

            m_environment_scp_message = environment_parameters_wet;
            std::string to_replace = "rain";
            position = m_environment_scp_message.find(to_replace);
            if ( position != std::string::npos) {
                m_environment_scp_message.replace(position, to_replace.length(), "snow");
            }
            to_replace = "4/8";
            position = m_environment_scp_message.find(to_replace);
            if ( position != std::string::npos) {
                m_environment_scp_message.replace(position, to_replace.length(), "4/8");
            }
            to_replace = "100000.000000";
            position = m_environment_scp_message.find(to_replace);
            if ( position != std::string::npos) {
                m_environment_scp_message.replace(position, to_replace.length(), "70.000000");
            }
            to_replace = "0.500000";
            position = m_environment_scp_message.find(to_replace);
            if ( position != std::string::npos) {
                m_environment_scp_message.replace(position, to_replace.length(), "0.600000");
            }
        }

        else if ( environment == "mild_snow"  ) {

            m_environment_scp_message = environment_parameters_wet;
            std::string to_replace = "rain";
            position = m_environment_scp_message.find(to_replace);
            if ( position != std::string::npos) {
                m_environment_scp_message.replace(position, to_replace.length(), "snow");
            }
            to_replace = "4/8";
            position = m_environment_scp_message.find(to_replace);
            if ( position != std::string::npos) {
                m_environment_scp_message.replace(position, to_replace.length(), "4/8");
            }
            to_replace = "100000.000000";
            position = m_environment_scp_message.find(to_replace);
            if ( position != std::string::npos) {
                m_environment_scp_message.replace(position, to_replace.length(), "70.000000");
            }
            to_replace = "0.500000";
            position = m_environment_scp_message.find(to_replace);
            if ( position != std::string::npos) {
                m_environment_scp_message.replace(position, to_replace.length(), "0.800000");
            }
        }

        else if ( environment == "heavy_snow"  ) {

            m_environment_scp_message = environment_parameters_wet;
            std::string to_replace = "rain";
            position = m_environment_scp_message.find(to_replace);
            if ( position != std::string::npos) {
                m_environment_scp_message.replace(position, to_replace.length(), "snow");
            }
            to_replace = "4/8";
            position = m_environment_scp_message.find(to_replace);
            if ( position != std::string::npos) {
                m_environment_scp_message.replace(position, to_replace.length(), "0/8");
            }
            to_replace = "100000.000000";
            position = m_environment_scp_message.find(to_replace);
            if ( position != std::string::npos) {
                m_environment_scp_message.replace(position, to_replace.length(), "100000.000000");
            }
            to_replace = "0.500000";
            position = m_environment_scp_message.find(to_replace);
            if ( position != std::string::npos) {
                m_environment_scp_message.replace(position, to_replace.length(), "1.000000");
            }
        }

        else if ( environment == "night" ) {
            m_environment_scp_message = environment_parameters_dry;
            std::string to_replace = "39600";
            position = m_environment_scp_message.find(to_replace);
            if ( position != std::string::npos) {
                m_environment_scp_message.replace(position, to_replace.length(), "79200");
            }
        }

        // some stuff for performance measurement
        mStartTime = -1.0;

        mTotalNoImages= 0;

        // total number of errors
        mTotalErrorCount = 0;


    }

    void generate_gt_scene() override;

    void closeAllSockets() {

        for (ushort i = 0; i < m_generation_sensor_list.size(); i++) {

            for (ushort j = 0; j < 3; j++) {
                close(sensor_group.at(m_generation_sensor_list.at(i)).at(j).get<4>());
            }
        }
    }


    void configureSensor(ushort sensor_group_index, ushort shmKey, const int port_number_camera_sensor_data, const int port_number_usk_sensor_data, const int port_number_inertial_sensor_data,
            std::string module_manager_libModuleSensor_CameraTemplate, std::string module_manager_libModuleSensor_PerfectTemplate) {


        char command[50];
        sprintf(command, "ipcrm -M 0x%05X", shmKey);
        system(command);

        std::string module_manager_libModuleCameraSensor;
        std::string module_manager_libModulePerfectSensor;
        std::string module_manager_libModulePerfectSensorInertial;

        std::string to_replace, with_replace;

        std::string::size_type position;

        ///Start sensor
        ///--------------------------
        module_manager_libModuleCameraSensor = module_manager_libModuleSensor_CameraTemplate;

        //sensor name
        to_replace = "Sensor_MM";
        with_replace = "Sensor_MM_" + std::to_string(sensor_group_index);
        position = module_manager_libModuleCameraSensor.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModuleCameraSensor.replace(position, to_replace.length(), with_replace);
        }

        to_replace = std::to_string(65535);
        position = module_manager_libModuleCameraSensor.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModuleCameraSensor.replace(position, to_replace.length(), std::to_string(port_number_camera_sensor_data));
        }

        sensor_group.at(sensor_group_index).push_back(boost::make_tuple(module_manager_libModuleCameraSensor, "suffix", port_number_camera_sensor_data, shmKey, 0, (void *)0x0));

        ///--------------------------

        module_manager_libModulePerfectSensor = module_manager_libModuleSensor_PerfectTemplate;
        //port number
        to_replace = std::to_string(65535);
        position = module_manager_libModulePerfectSensor.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModulePerfectSensor.replace(position, to_replace.length(), std::to_string(port_number_usk_sensor_data));
        }

        //sensor name
        to_replace = "Sensor_MM";
        with_replace = "Sensor_MM_Perfect_" + std::to_string(sensor_group_index);
        position = module_manager_libModulePerfectSensor.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModulePerfectSensor.replace(position, to_replace.length(), with_replace);
        }

        sensor_group.at(sensor_group_index).push_back(boost::make_tuple(module_manager_libModulePerfectSensor, "suffix", port_number_usk_sensor_data, shmKey, 0, (void *)NULL));

        ///--------------------------

        module_manager_libModulePerfectSensorInertial = module_manager_libModuleSensor_PerfectTemplate;
        //port number
        to_replace = std::to_string(65535);
        position = module_manager_libModulePerfectSensorInertial.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModulePerfectSensorInertial.replace(position, to_replace.length(), std::to_string(port_number_inertial_sensor_data));
        }

        //sensor name
        to_replace = "Sensor_MM";
        with_replace = "Sensor_MM_PerfectInertial_" + std::to_string(sensor_group_index);
        position = module_manager_libModulePerfectSensorInertial.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModulePerfectSensorInertial.replace(position, to_replace.length(), with_replace);
        }

        //sensor coordinate
        to_replace = "usk";
        position = module_manager_libModulePerfectSensorInertial.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModulePerfectSensorInertial.replace(position, to_replace.length(), "inertial");
        }

        sensor_group.at(sensor_group_index).push_back(boost::make_tuple(module_manager_libModulePerfectSensorInertial, "suffix", port_number_inertial_sensor_data, shmKey, 0, (void *)NULL));

        ///End sensor

    }


    ~GroundTruthSceneExternal(){
        std::cout << "killing previous GroundTruthScene object\n" ;
    }

    void write_gt_scene_data() override;

    void read_gt_scene_data() override;

    void generate_frame_difference_images_noise();

};






#endif //MAIN_GENERATEGROUNDTRUTHSCENE_H