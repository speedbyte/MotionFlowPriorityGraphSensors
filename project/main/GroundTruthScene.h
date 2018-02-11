//
// Created by veikas on 27.01.18.
//

#ifndef MAIN_GROUNDTRUTHSCENE_H
#define MAIN_GROUNDTRUTHSCENE_H


#include <vires/vires/viRDBIcd.h>
#include <vires/vires_common.h>
#include <iostream>
#include "Dataset.h"
#include "ObjectTrajectory.h"
#include "GroundTruthObjects.h"
#include "Canvas.h"
#include <map>


class GroundTruthScene  {

protected:
    std::vector<GroundTruthObjects> &m_list_objects;

    boost::filesystem::path  m_basepath;

    boost::filesystem::path  m_datasetpath;

    boost::filesystem::path  m_generatepath;

    boost::filesystem::path  m_trajectory_obj_path;

    std::string m_scenario;

public:

    GroundTruthScene(std::string scenario, std::vector<GroundTruthObjects> &list_objects):m_scenario
                                                                                                  (scenario),
    m_list_objects(list_objects) {

        m_datasetpath = Dataset::getBasePath();
        m_basepath = Dataset::getGroundTruthPath();

    };

    GroundTruthScene(std::vector<GroundTruthObjects> &list_objects):m_list_objects(list_objects) {

        m_basepath = Dataset::getGroundTruthPath();

    };

    virtual void generate_gt_scene() {};

    void prepare_directories();

};

class GroundTruthSceneInternal : public GroundTruthScene {

private:

    Canvas &m_canvas;

public:

    GroundTruthSceneInternal(Canvas &canvas, std::vector<GroundTruthObjects> &list_objects) :
            m_canvas(canvas), GroundTruthScene(list_objects) {}

    void generate_gt_scene();

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

    std::vector<std::pair<std::string, cv::Point2f> >trajectory_points;

    const std::string to_replace = "traffic_demo";

    const char *project_name = "<SimCtrl><Project name=\"Movement\" "
            "path=\"/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.0/Data/Projects/Current\" "
            "/></SimCtrl>";

    std::string scenario_name = "<SimCtrl><UnloadSensors /><LoadScenario "
            "filename=\"/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2"
            ".0/Data/Projects/Current/Scenarios/traffic_demo"
            ".xml\" /><Start mode=\"operation\" /></SimCtrl>";

    const char *module_manager = "<Sensor name=\"Sensor_MM\" type=\"video\"><Load lib=\"libModuleCameraSensor.so\" "
            "path=\"/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.0/Data/Projects/../Distros/Distro/Plugins/ModuleManager\" /><Player name=\"New Player\" /><Frustum bottom=\"15.000000\" far=\"40.000000\" left=\"20.000000\" near=\"1.000000\" right=\"20.000000\" top=\"15.000000\" /><Position dhDeg=\"0.000000\" dpDeg=\"0.000000\" drDeg=\"0.000000\" dx=\"0.000000\" dy=\"0.000000\" dz=\"0.000000\" /><Origin type=\"usk\" /><Cull enable=\"true\" maxObjects=\"10\" /><Port name=\"RDBout\" number=\"48185\" sendEgo=\"true\" type=\"TCP\" /><Filter objectType=\"none\" /><Filter objectType=\"pedestrian\" /><Debug camera=\"false\" culling=\"false\" detection=\"false\" dimensions=\"false\" enable=\"false\" packages=\"false\" position=\"false\" road=\"false\" /></Sensor>";

    std::string camera_parameters = "<Camera name=\"VIEW_CAMERA\" showOwner=\"false\"><Frustum far=\"1500.000000\" "
    "fovHor=\"40.000000\" fovVert=\"30.000000\" near=\"1.000000\" offsetHor=\"0.000000\" offsetVert=\"0"
            ".000000\"/><PosEyepoint /><ViewRelative dh=\"0.000000\" dp=\"0.000000\" dr=\"0.000000\" /><Set/></Camera>";

    std::string display_parameters = "<Display><SensorSymbols enable=\"false\" sensor=\"Sensor_MM\" "
            "showCone=\"false\" /><Database enable=\"true\" streetLamps=\"false\" /><VistaOverlay enable=\"false\" "
            "/></Display>";

    // Precipitation intensity needs to be > 0 for snow and rain.
    std::string environment_parameters = "<Environment><Friction value=\"1.000000\" /><TimeOfDay "
            "headlights=\"false\" value=\"39600\" /><Sky cloudState=\"4/8\" visibility=\"100000.000000\" "
            "/><Precipitation intensity=\"4.000000\" type=\"snow\" /><Road effectScale=\"0.500000\" state=\"wet\" "
            "/></Environment>";

    std::string rdbtrigger_portnumber = "<TaskControl><RDB client=\"false\" enable=\"true\" interface=\"eth0\" portRx=\"48190\" portTx=\"48190\" portType=\"TCP\" /></TaskControl>";

    std::string message_scp = "<Symbol name=\"expl01\" > <Text data=\"Time for snow\" colorRGB=\"0xffff00\" size=\"50"
            ".0\" /> <PosScreen x=\"0.01\" y=\"0.05\" /></Symbol>";

    std::string popup_scp = "<Info level=\"info\"> <Message popup=\"true\" text=\"snow!!!!\"/> </Info>";

    std::string eyepoint = "<VIL><EyepointOffset hDeg=\"30.000000\" pDeg=\"0.000000\" rDeg=\"0.000000\" x=\"0.000000\""
            " y=\"0.000000\" z=\"0.000000\" /></VIL>";

    std::string elevation = "<VIL><Imu dbElevation=\"true\" /></VIL>";

public:

    GroundTruthSceneExternal(std::string scenario, std::vector<GroundTruthObjects> &list_objects) :
    GroundTruthScene(scenario, list_objects)  {

        std::string::size_type position = scenario_name.find(to_replace);
        if ( position != std::string::npos ) {
            scenario_name.replace(position, to_replace.length(), std::string(m_scenario));
        }

    }

    void generate_gt_scene();

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
