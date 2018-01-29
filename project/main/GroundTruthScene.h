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
#include "ObjectProperties.h"

class GroundTruthScene : protected Framework::ViresInterface {

protected:
    Dataset &m_dataset;

public:

    GroundTruthScene(Dataset &dataset):m_dataset(dataset) {}

    virtual void generate_gt_scene() {};

    virtual ObjectShape getObjectShape() {};

    virtual ObjectTrajectory getObjectTrajectory() {};

protected:
    void prepare_directories();

};

class GroundTruthSceneInternal : public GroundTruthScene {

private:

    std::vector<ObjectProperties> &m_list_objects;

public:

    GroundTruthSceneInternal(Dataset &dataset, std::vector<ObjectProperties> &list_objects) :
            GroundTruthScene(dataset), m_list_objects(list_objects) {}

    void generate_gt_scene();

    cv::Mat getObjectShape(int index) {
        return m_list_objects.at(index).getShape().get();
    }

    std::vector<cv::Point2i> getObjectTrajectory(int index) {
        return m_list_objects.at(index).getTrajectoryPoints().get();
    };

    ~GroundTruthSceneInternal(){
        std::cout << "killing previous GroundTruthScene object\n" ;
    }

};

class GroundTruthSceneExternal : public GroundTruthScene {

private:

    std::vector<ObjectProperties> m_list_of_objects;
    std::string m_scenario;

public:

    GroundTruthSceneExternal(Dataset &dataset, std::string scenario) : GroundTruthScene(dataset), m_scenario
    (scenario) {}

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

    std::vector<ObjectProperties> getListOfObjects() {
        return m_list_of_objects;
    }

    ~GroundTruthSceneExternal(){
        std::cout << "killing previous GroundTruthScene object\n" ;
    }

};





#endif //MAIN_GROUNDTRUTHSCENE_H
