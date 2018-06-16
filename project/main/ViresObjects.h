//
// Created by veikas on 10.02.18.
//

#ifndef MAIN_VIRESOBJECTS_H
#define MAIN_VIRESOBJECTS_H

#include <iostream>
#include <unistd.h>
#include <boost/tuple/tuple.hpp>
#include <vires-interface/vires_configuration.h>
#include <vires-interface/vires_common.h>
#include <map>
#include "ObjectMetaData.h"
#include "SensorMetaData.h"


#define MAX_DUMPS 10


class ViresObjects: protected Framework::ViresInterface {

private:
    std::vector<boost::tuple<std::string, std::string, ushort > > sensor_group;

    unsigned current_frame_index;

    unsigned int mShmKey;      // key of the SHM segment

    void *mShmPtr;

    int m_moduleManagerSocket_Camera;

    int m_moduleManagerSocket_Perfect;

    int m_moduleManagerSocket_PerfectInertial;

    bool m_dumpInitialFrames;

    bool         mCheckForImage;

    int          mHaveImage ;                                 // is an image available?

    bool         mHaveFirstFrame;

    bool         mHaveFirstImage;

    int mImageCount;

    bool m_breaking;

    boost::filesystem::path  m_generatepath;

    std::vector<ObjectMetaData> objectMetaDataList;
    std::vector<std::vector<ObjectMetaData *> > m_ptr_customObjectMetaDataList;
    std::vector<std::vector<SensorMetaData *> > m_ptr_customSensorMetaDataList;
    std::vector<std::map<std::string, ObjectMetaData*> > m_mapObjectNameToObjectMetaData;
    std::vector<std::map<std::string, SensorMetaData*> > m_mapSensorNameToSensorMetaData;
    std::map<unsigned int, std::string> m_mapObjectIdToObjectName;
    std::map<unsigned int, std::string> m_mapSensorIdToSensorName;
    std::vector<SensorMetaData> sensorMetaDataList;
    ushort m_objectCount;
    ushort m_sensorCount;

    static ushort m_sensorGroupTotalCount;
    ushort m_sensorGroupCount;



public:

    ViresObjects(unsigned int shmKey, boost::filesystem::path  generatepath): mShmKey(shmKey), m_generatepath(generatepath),
            m_ptr_customObjectMetaDataList(MAX_ALLOWED_SENSOR_GROUPS), m_mapObjectNameToObjectMetaData(MAX_ALLOWED_SENSOR_GROUPS), m_ptr_customSensorMetaDataList(MAX_ALLOWED_SENSOR_GROUPS), m_mapSensorNameToSensorMetaData(MAX_ALLOWED_SENSOR_GROUPS) {

        mCheckForImage  = false;
        mHaveImage    = 0;                                 // is an image available?
        mHaveFirstFrame    = false;
        mHaveFirstImage    = false;
        mImageCount = 0;

        m_sensorGroupCount = m_sensorGroupTotalCount;
        m_sensorGroupTotalCount++;

        m_breaking = false;
        m_dumpInitialFrames = true;

        m_objectCount = 0;
        m_sensorCount = 0;

        for (int i = 0; i < MAX_ALLOWED_OBJECTS; ++i) {
            ObjectMetaData objMetaData;
            objectMetaDataList.push_back(objMetaData);
        }
        for (int i = 0; i < MAX_ALLOWED_SENSORS; ++i) {
            SensorMetaData senMetaData;
            sensorMetaDataList.push_back(senMetaData);
        }

    }

    ViresObjects() {}

    void writePositionInYaml(std::string suffix);

    void configureSensor(const int port_number_camera_sensor_data, const int port_number_usk_sensor_data, const int port_number_inertial_sensor_data,
    std::string module_manager_libModuleSensor_CameraTemplate, std::string module_manager_libModuleSensor_PerfectTemplate, std::string module_manager_libModuleSensor_PerfectInertialTemplate) {


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
        with_replace = "Sensor_MM_" + std::to_string(m_sensorGroupCount);
        position = module_manager_libModuleCameraSensor.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModuleCameraSensor.replace(position, to_replace.length(), with_replace);
        }

        to_replace = std::to_string(65535);
        position = module_manager_libModuleCameraSensor.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModuleCameraSensor.replace(position, to_replace.length(), std::to_string(port_number_camera_sensor_data));
        }

        sensor_group.push_back(boost::make_tuple(module_manager_libModuleCameraSensor, "suffix", port_number_camera_sensor_data));

        ///--------------------------

        module_manager_libModulePerfectSensor = module_manager_libModuleSensor_PerfectTemplate;
        //port number
        to_replace = std::to_string(65535);
        position = module_manager_libModulePerfectSensor.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModulePerfectSensor.replace(position, to_replace.length(), std::to_string(port_number_usk_sensor_data));
        }

        //sensorlibrary
        to_replace = "libModuleCameraSensor";
        position = module_manager_libModulePerfectSensor.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModulePerfectSensor.replace(position, to_replace.length(), "libModulePerfectSensor");
        }

        //sensor name
        to_replace = "Sensor_MM";
        with_replace = "Sensor_MM_Perfect_" + std::to_string(m_sensorGroupCount);
        position = module_manager_libModulePerfectSensor.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModulePerfectSensor.replace(position, to_replace.length(), with_replace);
        }

        sensor_group.push_back(boost::make_tuple(module_manager_libModulePerfectSensor, "suffix", port_number_usk_sensor_data));

        ///--------------------------

        module_manager_libModulePerfectSensorInertial = module_manager_libModuleSensor_PerfectInertialTemplate;
        //port number
        to_replace = std::to_string(65535);
        position = module_manager_libModulePerfectSensorInertial.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModulePerfectSensorInertial.replace(position, to_replace.length(), std::to_string(port_number_inertial_sensor_data));
        }

        //sensorlibrary
        to_replace = "libModuleCameraSensor";
        position = module_manager_libModulePerfectSensorInertial.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModulePerfectSensorInertial.replace(position, to_replace.length(), "libModulePerfectSensor");
        }

        //sensor name
        to_replace = "Sensor_MM";
        with_replace = "Sensor_MM_PerfectInertial_" + std::to_string(m_sensorGroupCount);
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

        sensor_group.push_back(boost::make_tuple(module_manager_libModulePerfectSensorInertial, "suffix", port_number_inertial_sensor_data));

        ///End sensor

    }

    const std::vector<boost::tuple< std::string, std::string, ushort > >  &getSensorConfiguration() {
        return sensor_group;
    }

    const unsigned int &getShmKey() {
        return mShmKey;
    }

    void closeAllSockets() {

        close(m_moduleManagerSocket_Camera);
        close(m_moduleManagerSocket_Perfect);
        close(m_moduleManagerSocket_PerfectInertial);
    }

    void openNetworkSockets() {

        // initalize the server variable
        std::string serverName = "127.0.0.1";

        setServer(serverName.c_str());

        m_moduleManagerSocket_Camera = openNetwork(sensor_group.at(0).get<2>());
        m_moduleManagerSocket_Perfect = openNetwork(sensor_group.at(1).get<2>());
        m_moduleManagerSocket_PerfectInertial = openNetwork(sensor_group.at(2).get<2>());

    }

    int getCameraSocketHandler() {
        return m_moduleManagerSocket_Camera;
    }

    int getPerfectSocketHandler() {
        return m_moduleManagerSocket_Perfect;
    }

    int getPerfectInertialSocketHandler() {
        return m_moduleManagerSocket_PerfectInertial;
    }

    void getGroundTruthInformation(bool withTrigger, int triggerSocket, bool getGroundTruthData);


    void openShmWrapper() {
        mShmPtr = openShm(mShmKey);
    }

    bool getBreaking() {
        return m_breaking;
    }


    void parseStartOfFrame(const double &simTime, const unsigned int &simFrame);

    void parseEndOfFrame( const double & simTime, const unsigned int & simFrame );

    void parseEntry( RDB_IMAGE_t *data, const double & simTime, const unsigned int & simFrame, const
    unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int &totalElem );

    void parseEntry( RDB_OBJECT_STATE_t *data, const double & simTime, const unsigned int & simFrame, const
    unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int &
    totalElem );

    void parseEntry( RDB_OBJECT_CFG_t *data, const double & simTime, const unsigned int & simFrame, const
    unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem );

    void parseEntry( RDB_DRIVER_CTRL_t *data, const double & simTime, const unsigned int & simFrame, const
    unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int &totalElem );

    void parseEntry( RDB_TRIGGER_t *data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId,
                     const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem );

    void parseEntry( RDB_SENSOR_STATE_t *data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId,
                     const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem );

    void parseEntry(RDB_SENSOR_OBJECT_t *data, const double &simTime, const unsigned int &
    simFrame, const unsigned short &pkgId, const unsigned short &flags, const unsigned int &elemId,
                    const unsigned int &totalElem);


};


#endif //MAIN_VIRESOBJECTS_H
