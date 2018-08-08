//
// Created by veikas on 10.02.18.
//

#ifndef MAIN_VIRESOBJECTS_H
#define MAIN_VIRESOBJECTS_H

#include <iostream>
#include <unistd.h>
#include <boost/tuple/tuple.hpp>
#include <vires-interface/ViresConfiguration.h>
#include <vires-interface/ViresInterface.h>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include "ObjectMetaData.h"
#include "SensorMetaData.h"
#include "BasicObjects.h"


#define MAX_DUMPS 10




class ViresObjects: public Framework::ViresInterface, public BasicObjects {

private:

    unsigned current_frame_index;


    bool m_dumpInitialFrames;

    bool         mCheckForImage;

    int          mHaveImage ;                                 // is an image available?

    bool         mHaveFirstFrame;

    bool         mHaveFirstImage;

    bool m_breaking;

    std::map<unsigned int, std::string> m_mapObjectIdToObjectName;
    std::map<unsigned int, std::string> m_mapSensorIdToSensorName;

    std::ofstream fstream_output_object_state;
    std::ofstream fstream_output_sensor_state;
    std::ofstream fstream_output_sensor_object;

    void readObjectStateFromBinaryFile();
    void readSensorObjectFromBinaryFile();
    void readSensorStateFromBinaryFile();


public:

    ViresObjects(ushort current_sensor_group_index, boost::filesystem::path  generatepath): BasicObjects(current_sensor_group_index, generatepath) {

        mCheckForImage  = false;
        mHaveImage    = 0;                                 // is an image available?
        mHaveFirstFrame    = false;
        mHaveFirstImage    = false;

        m_breaking = false;
        m_dumpInitialFrames = true;

        m_objectCount = 0;
        m_sensorCount = 0;


    }

    void openAllFileHandles() {

        boost::filesystem::remove("../object_state_" + std::to_string(m_sensorGroupCount) + ".bin");
        fstream_output_object_state = std::ofstream("../object_state_" + std::to_string(m_sensorGroupCount) + ".bin", std::ios::out | std::ios::binary);

        boost::filesystem::remove("../sensor_state_" + std::to_string(m_sensorGroupCount) + ".bin");
        fstream_output_sensor_state = std::ofstream("../sensor_state_" + std::to_string(m_sensorGroupCount) + ".bin", std::ios::out | std::ios::binary);

        boost::filesystem::remove("../sensor_object_" + std::to_string(m_sensorGroupCount) + ".bin");
        fstream_output_sensor_object = std::ofstream("../sensor_object_" + std::to_string(m_sensorGroupCount) + ".bin", std::ios::out | std::ios::binary);
    }

    ViresObjects() {}


    void readObjectData();
    void readSensorData();


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

    void configureShm(void *shmPtr, unsigned int shmTotalSize);

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
