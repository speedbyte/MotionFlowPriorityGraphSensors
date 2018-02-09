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


class GroundTruthScene  {

protected:
    std::vector<GroundTruthObjects> &m_list_objects;

public:

    GroundTruthScene(std::vector<GroundTruthObjects> &list_objects):m_list_objects(list_objects) {};

    virtual void generate_gt_scene() {};

protected:
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

    std::string m_scenario;

public:

    GroundTruthSceneExternal(std::string scenario, std::vector<GroundTruthObjects> &list_objects) :
    m_scenario(scenario), GroundTruthScene(list_objects)  {}

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
