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
#include "Objects.h"
#include "Canvas.h"


class GroundTruthScene  {

protected:
    Dataset &m_dataset;

public:

    GroundTruthScene(Dataset &dataset, Canvas canvas): m_dataset(dataset) {}

    GroundTruthScene(Dataset &dataset): m_dataset(dataset) {}

    virtual void generate_gt_scene() {};

protected:
    void prepare_directories();

};

class GroundTruthSceneInternal : public GroundTruthScene {

private:

    std::vector<Objects> &m_list_objects;
    Canvas &m_canvas;

public:

    GroundTruthSceneInternal(Dataset &dataset, Canvas &canvas, std::vector<Objects> &list_objects) :
            GroundTruthScene(dataset), m_canvas(canvas), m_list_objects(list_objects) {}

    void generate_gt_scene();

    cv::Mat getObjectShape(int index) {
        return m_list_objects.at(index).getData();
    }

    std::vector<cv::Point2i> getObjectTrajectory(int index) {
        return m_list_objects.at(index).getTrajectoryPoints().get();
    };

    ~GroundTruthSceneInternal(){
        std::cout << "killing previous GroundTruthScene object\n" ;
    }

};

class GroundTruthSceneExternal : public GroundTruthScene, protected Framework::ViresInterface {

private:

    std::vector<Objects> m_list_of_objects;
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

    std::vector<Objects> getListOfObjects() {
        return m_list_of_objects;
    }

    ~GroundTruthSceneExternal(){
        std::cout << "killing previous GroundTruthScene object\n" ;
    }

};





#endif //MAIN_GROUNDTRUTHSCENE_H
