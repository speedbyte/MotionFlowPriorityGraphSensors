//
// Created by veikas on 27.01.18.
//

#ifndef MAIN_GROUNDTRUTHSCENE_H
#define MAIN_GROUNDTRUTHSCENE_H


#include <vires/vires/viRDBIcd.h>
#include <vires/vires_common.h>
#include "Dataset.h"
#include "ObjectTrajectory.h"

class GroundTruthScene : protected Framework::ViresInterface {
protected:
    Dataset m_dataset;
public:
    GroundTruthScene() {}
    GroundTruthScene(Dataset dataset):m_dataset(dataset) {}
    void prepare_directories();
    virtual void generate_gt_scene() {};

};

class GroundTruthSceneInternal : public GroundTruthScene {

private:

    std::vector<ObjectShape> &m_shapes;
    std::vector<ObjectTrajectory> &m_trajectories;

public:

    GroundTruthSceneInternal(Dataset dataset, std::vector<ObjectShape> &shapes, std::vector<ObjectTrajectory>
    &trajectories) :
    GroundTruthScene(dataset), m_shapes(shapes), m_trajectories(trajectories) {}

    void generate_gt_scene();

    std::vector<ObjectShape> getListObjectShapes() {
        return m_shapes;
    }

    std::vector<ObjectTrajectory> getListObjectTrajectories() {
        return m_trajectories;
    };

    ~GroundTruthSceneInternal(){
        std::cout << "killing previous GroundTruthScene object\n" ;
    }

};

class GroundTruthSceneExternal : GroundTruthScene {

private:

    std::vector<ObjectShape> &m_shapes;
    std::vector<ObjectTrajectory> &m_trajectories;

public:

    GroundTruthSceneExternal(Dataset dataset, std::vector<ObjectShape> &shapes, std::vector<ObjectTrajectory>
    &trajectories) :
    GroundTruthScene(dataset), m_shapes(shapes), m_trajectories(trajectories) {}

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
