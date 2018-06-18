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

    unsigned current_frame_index;


    bool m_dumpInitialFrames;

    bool         mCheckForImage;

    int          mHaveImage ;                                 // is an image available?

    bool         mHaveFirstFrame;

    bool         mHaveFirstImage;

    int mImageCount;

    bool m_breaking;

    boost::filesystem::path  m_generatepath;

    std::vector<ObjectMetaData> objectMetaDataList;
    std::vector<ObjectMetaData *>  m_ptr_customObjectMetaDataList;
    std::vector<SensorMetaData *>  m_ptr_customSensorMetaDataList;
    std::map<std::string, ObjectMetaData*>  m_mapObjectNameToObjectMetaData;
    std::map<std::string, SensorMetaData*>  m_mapSensorNameToSensorMetaData;
    std::map<unsigned int, std::string> m_mapObjectIdToObjectName;
    std::map<unsigned int, std::string> m_mapSensorIdToSensorName;
    std::vector<SensorMetaData> sensorMetaDataList;
    ushort m_objectCount;
    ushort m_sensorCount;

    ushort m_sensorGroupCount;




public:

    ViresObjects(ushort current_sensor_group_index, boost::filesystem::path  generatepath): m_sensorGroupCount(current_sensor_group_index), m_generatepath(generatepath) {

        mCheckForImage  = false;
        mHaveImage    = 0;                                 // is an image available?
        mHaveFirstFrame    = false;
        mHaveFirstImage    = false;
        mImageCount = 0;

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



    void getGroundTruthInformation(void* shmPtr, bool withTrigger, int triggerSocket, bool getGroundTruthData, bool getGroundTruthImages,
            ushort m_moduleManagerSocket_Camera, ushort m_moduleManagerSocket_Perfect, ushort m_moduleManagerSocket_PerfectInertial);


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
