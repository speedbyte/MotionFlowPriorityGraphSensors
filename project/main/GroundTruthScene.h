//
// Created by veikas on 27.01.18.
//

#ifndef MAIN_GROUNDTRUTHSCENE_H
#define MAIN_GROUNDTRUTHSCENE_H


#include <vires-interface/Common/viRDBIcd.h>
#include <vires-interface/vires_common.h>
#include <iostream>
#include "Dataset.h"
#include "ObjectTrajectory.h"
#include "GroundTruthObjects.h"
#include "Canvas.h"
#include <map>


class GroundTruthScene  {

protected:
    std::vector<GroundTruthObjects> &m_list_objects;

    boost::filesystem::path  m_groundtruthpath;

    boost::filesystem::path  m_datasetpath;

    boost::filesystem::path  m_generatepath;

    boost::filesystem::path  m_trajectory_obj_path;

    std::string m_scenario;

    std::string m_environment;

    Canvas m_canvas;

    cv::FileStorage m_write_fs;


public:

    GroundTruthScene(std::string scenario, std::string environment, std::vector<GroundTruthObjects> &list_objects):m_scenario(scenario), m_environment(environment),
    m_list_objects(list_objects) {

        m_datasetpath = Dataset::getDatasetPath();
        m_groundtruthpath = Dataset::getGroundTruthPath();

    };

    std::vector<cv::Point2f> readTrajectoryFromFile(std::string trajectoryFileName);

    void writeTrajectoryInYaml();

    virtual void generate_gt_scene() {};

    void generate_bird_view();

    void prepare_directories();


};

class GroundTruthSceneInternal : public GroundTruthScene {

private:


public:

    GroundTruthSceneInternal(std::string scenario, std::string environment, std::vector<GroundTruthObjects> &list_objects) :
    GroundTruthScene(scenario, environment, list_objects) {
    }

    void generate_gt_scene() override;

    cv::Mat getObjectShape(int index) {
        return m_list_objects.at(index).getImageShapeAndData().get();
    }

    ~GroundTruthSceneInternal(){
        std::cout << "killing previous GroundTruthScene object\n" ;
    }


};

class GroundTruthSceneExternal : public GroundTruthScene, protected Framework::ViresInterface {

private:

    MyTrajectory trajectory;

    std::vector<std::pair<std::string, cv::Point2f> > mtrajectory_points;

    const std::string to_replace = "traffic_demo";

    std::string apply = "<SimCtrl><Apply/></SimCtrl>";

    std::string project_name = "<SimCtrl><Project name=\"Movement\"path=\"/local/git/MotionFlowPriorityGrapSensors/VIRES/VTD.2.0/Data/Projects/Current\" /></SimCtrl>";

    std::string scenario_name = "<SimCtrl><UnloadSensors /><LoadScenario filename=\"/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.0/Data/Projects/Current/Scenarios/traffic_demo.xml\" /><Start mode=\"operation\" /></SimCtrl>";

    std::string image_generator = "<ImageGenerator> <Window width=\"800\" height=\"600\" x=\"0\" y=\"0\" screen=\"0\" border=\"true\"/></ImageGenerator>";

    std::string module_manager = "<Sensor name=\"Sensor_MM\" type=\"video\" > <Load lib=\"libModuleCameraSensor.so\" path=\"/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.0/Data/Projects/../Distros/Distro/Plugins/ModuleManager\" /> <Player name=\"MovingCar\"/> <Frustum near=\"1.000000\" far=\"40.000000\" left=\"30.000000\" right=\"12.500000\" bottom=\"15.000000\" top=\"15.000000\" /> <Position dx=\"0.000000\" dy=\"0.000000\" dz=\"0.000000\" dhDeg=\"0.000000\" dpDeg=\"0.000000\" drDeg=\"0.000000\" /> <Origin type=\"usk\" /> <Cull maxObjects=\"10\" enable=\"true\" /> <Port name=\"RDBout\" number=\"48185\" type=\"TCP\" sendEgo=\"true\" /> <Filter objectType=\"none\" /><Filter objectType=\"pedestrian\" /> <Debug enable=\"false\" detection=\"false\" road=\"false\" position=\"false\" dimensions=\"false\" camera=\"false\" packages=\"false\" culling=\"false\" /> </Sensor>";

    // Precipitation intensity needs to be > 0 for snow and rain.
    const std::string environment_parameters_dry = "<Environment> <Friction value=\"1.000000\" /> <TimeOfDay value=\"39600\" headlights=\"auto\" /> <Sky cloudState=\"4/8\" visibility=\"100000.000000\" /><Precipitation type=\"none\" intensity=\"0.000000\" /><Road state=\"dry\" effectScale=\"0.500000\" /></Environment>";
    //Rain
    const std::string environment_parameters_wet = "<Environment> <Friction value=\"1.000000\" /> <TimeOfDay value=\"39600\" headlights=\"auto\" /> <Sky cloudState=\"4/8\" visibility=\"100000.000000\" /><Precipitation type=\"rain\" intensity=\"0.500000\" /><Road state=\"dry\" effectScale=\"0.500000\" /></Environment>";

    std::string camera_parameters = "<Camera name=\"VIEW_CAMERA\" showOwner=\"false\"> <Frustum near=\"0.100000\" far=\"1501.000000\" fovHor=\"60.000000\" fovVert=\"40.000000\" offsetHor=\"0.000000\" offsetVert=\"0.000000\" /> <PosTether player=\"MovingCar\" distance=\"6.000000\" azimuth=\"0.000000\" elevation=\"0.261799\" slew=\"1\" /> <ViewRelative dh=\"0.000000\" dp=\"0.000000\" dr=\"0.000000\" /><Set /> </Camera>";

    std::string display_parameters = "<Display>  <SensorSymbols enable=\"false\" sensor=\"Sensor_MM\" showCone=\"false\" /> <SensorSymbols enable=\"false\" sensor=\"Sensor_MM\" showCone=\"true\" /> <Database enable=\"true\" streetLamps=\"false\"/> <VistaOverlay enable=\"false\" /> </Display>";

    std::string elevation = "<VIL><Imu dbElevation=\"true\" /></VIL>";

    std::string eyepoint = "<VIL><EyepointOffset hDeg=\"0.000000\" pDeg=\"0.000000\" rDeg=\"0.000000\" x=\"0.000000\"y=\"0.000000\" z=\"0.000000\" /></VIL>";

    std::string pause = "<SimCtrl> <Pause/> </SimCtrl>";

    std::string rdbtrigger_portnumber = "<TaskControl><RDB client=\"false\" enable=\"true\" interface=\"eth0\" portRx=\"48190\" portTx=\"48190\" portType=\"TCP\" /></TaskControl>";

    std::string message_scp = "<Symbol name=\"expl01\" > <Text data=\"Time for snow\" colorRGB=\"0xffff00\" size=\"50.0\" /> <PosScreen x=\"0.01\" y=\"0.05\" /></Symbol>";

    std::string popup_scp = "<Info level=\"info\"> <Message popup=\"true\" text=\"snow!!!!\"/> </Info>";

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


    int          mLastNetworkFrame;

    int          mLastIGTriggerFrame ;

    unsigned int mSimFrame;                                 // simulation frame counter
    double       mSimTime;                               // simulation time

    double       mDeltaTime;                              // simulation step width

    int          mHaveImage ;                                 // is an image available?

    // some stuff for performance measurement
    double       mStartTime;


    unsigned int mFrameNo;

    unsigned int mFrameTime;

    bool         mHaveFirstImage;
    bool         mHaveFirstFrame;
    bool         mCheckForImage;

    int          mTotalNoImages;

    // total number of errors
    unsigned int mTotalErrorCount;

    int          mLastImageId;

    ushort mImageCount;

    unsigned int mShmKey;      // key of the SHM segment

    std::vector<MyTrajectory> myTrajectoryVector;

    // image skip factor
    static const unsigned short mImageSkipFactor;


    std::string m_environment_scp_message;


public:


    void analyzeImage( RDB_IMAGE_t* img, const unsigned int & simFrame, unsigned int index );

    void calcStatistics();

    double getTime();

    GroundTruthSceneExternal(std::string scenario, std::string environment, std::vector<GroundTruthObjects> &list_objects) :
    GroundTruthScene(scenario, environment, list_objects) {



        std::string::size_type position = scenario_name.find(to_replace);
        if ( position != std::string::npos ) {
            scenario_name.replace(position, to_replace.length(), std::string(m_scenario));
        }


        if ( environment == "none") {
            m_environment_scp_message = environment_parameters_dry;
        }

        else if ( environment == "snow" || environment == "rain" ) {

            m_environment_scp_message = environment_parameters_wet;
            std::string to_replace = "rain";
            position = m_environment_scp_message.find(to_replace);
            if ( position != std::string::npos) {
                m_environment_scp_message.replace(position, to_replace.length(), environment );
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

        mLastNetworkFrame = -1;

        mLastIGTriggerFrame = -1;

        mSimFrame     = 0;                                 // simulation frame counter
        mSimTime      = 0.0;                               // simulation time

        mDeltaTime    = 0.01;                              // simulation step width

        mHaveImage    = 0;                                 // is an image available?

        // some stuff for performance measurement
        mStartTime = -1.0;


        mFrameNo           = 0;

        mFrameTime         = mDeltaTime;

        mHaveFirstImage = false;
        mHaveFirstFrame    = false;
        mCheckForImage  = false;

        mTotalNoImages= 0;

        // total number of errors
        mTotalErrorCount = 0;

        mLastImageId    = 0;

        mImageCount = 0;

        mShmKey       = RDB_SHM_ID_IMG_GENERATOR_OUT;      // key of the SHM segment

        MyTrajectory character_trajectory;
        MyTrajectory character01_trajectory;
        myTrajectoryVector.push_back(character_trajectory);
        myTrajectoryVector.push_back(character01_trajectory);

    }

    void generate_gt_scene() override;

    void parseStartOfFrame(const double &simTime, const unsigned int &simFrame);

    void parseEndOfFrame( const double & simTime, const unsigned int & simFrame );

    void parseEntry( RDB_IMAGE_t *data, const double & simTime, const unsigned int & simFrame, const
    unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int &totalElem );

    void parseEntry( RDB_OBJECT_STATE_t *data, const double & simTime, const unsigned int & simFrame, const
    unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int &
    totalElem );

    void parseEntry( RDB_OBJECT_CFG_t *data, const double & simTime, const unsigned int & simFrame, const
    unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem );

    ~GroundTruthSceneExternal(){
        std::cout << "killing previous GroundTruthScene object\n" ;
    }




};





#endif //MAIN_GROUNDTRUTHSCENE_H
