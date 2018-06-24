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

class BasicObjects {

    std::vector<ObjectMetaData *>  m_ptr_customObjectMetaDataList;
    std::vector<SensorMetaData *>  m_ptr_customSensorMetaDataList;

    std::vector<ObjectMetaData> objectMetaDataList;
    std::vector<SensorMetaData> sensorMetaDataList;

    ushort m_sensorGroupCount;
    boost::filesystem::path  m_generatepath;



public:

    BasicObjects(ushort current_sensor_group_index, boost::filesystem::path  generatepath): m_sensorGroupCount(current_sensor_group_index), m_generatepath(generatepath) {


        for (int i = 0; i < MAX_ALLOWED_OBJECTS; ++i) {
            ObjectMetaData objMetaData;
            objectMetaDataList.push_back(objMetaData);
        }
        for (int i = 0; i < MAX_ALLOWED_SENSORS; ++i) {
            SensorMetaData senMetaData;
            sensorMetaDataList.push_back(senMetaData);
        }


    }
    void calcBBFrom3DPosition();


};

class CppObjects: public BasicObjects {

};


class ViresObjects: protected Framework::ViresInterface, public BasicObjects {

private:

    unsigned current_frame_index;


    bool m_dumpInitialFrames;

    bool         mCheckForImage;

    int          mHaveImage ;                                 // is an image available?

    bool         mHaveFirstFrame;

    bool         mHaveFirstImage;

    int mImageCount;

    bool m_breaking;

    std::map<std::string, ObjectMetaData*>  m_mapObjectNameToObjectMetaData;
    std::map<std::string, SensorMetaData*>  m_mapSensorNameToSensorMetaData;
    std::map<unsigned int, std::string> m_mapObjectIdToObjectName;
    std::map<unsigned int, std::string> m_mapSensorIdToSensorName;

    ushort m_objectCount;
    ushort m_sensorCount;


    std::ofstream fstream_output_object_state;
    std::ofstream fstream_output_sensor_state;
    std::ofstream fstream_output_sensor_object;




public:

    ViresObjects(ushort current_sensor_group_index, boost::filesystem::path  generatepath): BasicObjects(current_sensor_group_index, generatepath) {

        mCheckForImage  = false;
        mHaveImage    = 0;                                 // is an image available?
        mHaveFirstFrame    = false;
        mHaveFirstImage    = false;
        mImageCount = 0;

        m_breaking = false;
        m_dumpInitialFrames = true;

        m_objectCount = 0;
        m_sensorCount = 0;


    }

    void openAllFileHandles() {
        fstream_output_object_state = std::ofstream("../object_state_" + std::to_string(m_sensorGroupCount) + ".bin", std::ios::out | std::ios::binary);
        fstream_output_sensor_state = std::ofstream("../sensor_state_" + std::to_string(m_sensorGroupCount) + ".bin", std::ios::out | std::ios::binary);
        fstream_output_sensor_object = std::ofstream("../sensor_object_" + std::to_string(m_sensorGroupCount) + ".bin", std::ios::out | std::ios::binary);
    }

    ViresObjects() {}

    void writePositionInYaml(std::string suffix);

    void readObjectStateFromBinaryFile(std::string suffix);
    void readSensorObjectFromBinaryFile(std::string suffix);
    void readSensorStateFromBinaryFile(std::string suffix);

    void closeAllFileHandles() {
        fstream_output_object_state.close();
        fstream_output_sensor_object.close();
        fstream_output_sensor_state.close();
    }
    void getGroundTruthInformation(void* shmPtr, bool withTrigger, int triggerSocket, bool getGroundTruthData, bool getGroundTruthImages,
            ushort m_moduleManagerSocket_Camera, ushort m_moduleManagerSocket_Perfect, ushort m_moduleManagerSocket_PerfectInertial);


    bool getBreaking() {
        return m_breaking;
    }

    void readPositionFromFile(std::string positionFileName);


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
