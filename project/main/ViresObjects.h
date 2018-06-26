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
#include <opencv2/imgcodecs.hpp>
#include "ObjectMetaData.h"
#include "SensorMetaData.h"
#include "Canvas.h"


#define MAX_DUMPS 10

class BasicObjects {

protected:
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

};

class CppObjects: public BasicObjects {


public:

    CppObjects(ushort current_sensor_group_index, boost::filesystem::path  generatepath): BasicObjects(current_sensor_group_index, generatepath) {}

    CppObjects() {}

    void process(cv::Mat tempGroundTruthImageBase) {

        Rectangle rectangle(Dataset::getFrameSize().width, Dataset::getFrameSize().height); // width, height

        Achterbahn achterbahn;

        achterbahn = Achterbahn(rectangle, "rectangle_long", 60);
        achterbahn.process(Dataset::getFrameSize());
        objectMetaDataList.at(0) = achterbahn;
        m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(0));

        m_ptr_customObjectMetaDataList.at(0)->setObjectShape(rectangle);

        achterbahn = Achterbahn(rectangle, "random_object", 120);
        achterbahn.process(Dataset::getFrameSize());
        objectMetaDataList.at(1) = achterbahn;
        m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(1));

        m_ptr_customObjectMetaDataList.at(1)->setObjectShape(rectangle);

        cv::Mat tempGroundTruthPosition;
        tempGroundTruthPosition.create(Dataset::getFrameSize(), CV_32FC3);
        assert(tempGroundTruthPosition.channels() == 3);
        tempGroundTruthPosition = cv::Scalar::all(255);

        cv::Mat tempGroundTruthPosition_2;
        tempGroundTruthPosition_2.create(Dataset::getFrameSize(), CV_32FC3);
        assert(tempGroundTruthPosition_2.channels() == 3);
        tempGroundTruthPosition_2 = cv::Scalar::all(255);


        std::map<std::string, double> time_map = {{"generate_single_scene_image", 0},
                {"generate_all_scene_image",    0}};

        char file_name_image[50];

        cv::Mat image_data_and_shape;
        cv::Mat positionShape;

        //draw new ground truth image.
        cv::Mat tempGroundTruthImage;
        tempGroundTruthImage.create(Dataset::getFrameSize(), CV_32FC3);
        assert(tempGroundTruthImage.channels() == 3);

        for (ushort current_frame_index = 0; current_frame_index < MAX_ITERATION_RESULTS; current_frame_index++) {

            sprintf(file_name_image, "000%03d_10.png", current_frame_index);
            std::string input_image_file_with_path = m_generatepath.string() + "_" + std::to_string(m_sensorGroupCount) + "/" + file_name_image; //+ file_name_image;

            tempGroundTruthImage = tempGroundTruthImageBase.clone();

            for (unsigned obj_index = 0; obj_index < m_ptr_customObjectMetaDataList.size(); obj_index++) {

                std::string position_image_file_with_path = m_generatepath.string() + '_' + std::to_string(m_sensorGroupCount)+ "/" + file_name_image;

                image_data_and_shape = m_ptr_customObjectMetaDataList.at(obj_index)->getObjectShape().get().clone();
                image_data_and_shape = image_data_and_shape.rowRange(0, cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        current_frame_index).m_object_dimension_camera_px.height_px)).colRange(0, cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        current_frame_index).m_object_dimension_camera_px.width_px));

                positionShape = m_ptr_customObjectMetaDataList.at(obj_index)->getObjectShape().get().clone();
                positionShape = positionShape.rowRange(0, cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        current_frame_index).m_object_dimension_camera_px.height_px)).colRange(0, cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                        current_frame_index).m_object_dimension_camera_px.width_px));

                if ((!m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).occluded )) {

                    image_data_and_shape.copyTo(tempGroundTruthImage(
                            cv::Rect(cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                    current_frame_index).m_object_location_camera_px.cog_px.x),
                                    cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                            current_frame_index).m_object_location_camera_px.cog_px.y),
                                    cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_dimension_camera_px.width_px),
                                    cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_dimension_camera_px.height_px))));

                }
            }

            cv::imwrite(input_image_file_with_path, tempGroundTruthImage);

        }


    }



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
