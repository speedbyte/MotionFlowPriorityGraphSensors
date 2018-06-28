//
// Created by veikas on 26.06.18.
//

#ifndef MAIN_BASICOBJECTS_H
#define MAIN_BASICOBJECTS_H


#include <map>
#include "SensorMetaData.h"
#include "ObjectMetaData.h"


class BasicObjects {

protected:

    std::map<std::string, ObjectMetaData*>  m_mapObjectNameToObjectMetaData;
    std::map<std::string, SensorMetaData*>  m_mapSensorNameToSensorMetaData;

    ushort m_objectCount;
    ushort m_sensorCount;

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

    BasicObjects() {}

    void calcBBFrom3DPosition();

    const std::vector<ObjectMetaData *>  get_ptr_customObjectMetaDataList() {
        return m_ptr_customObjectMetaDataList;
    }

    const std::vector<SensorMetaData *>  get_ptr_customSensorMetaDataList() {
        return m_ptr_customSensorMetaDataList;
    }

    void writePositionInYaml(std::string suffix);

    void readPositionFromFile(std::string positionFileName);

};

#endif //MAIN_BASICOBJECTS_H
