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

public:

    GroundTruthSceneExternal(std::string scenario, std::vector<GroundTruthObjects> &list_objects) :
    GroundTruthScene(scenario, list_objects)  {}

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
