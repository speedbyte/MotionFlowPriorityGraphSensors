//
// Created by veikas on 28.01.18.
//

#ifndef MAIN_OBJECTPROPERTIES_H
#define MAIN_OBJECTPROPERTIES_H


// GroundTruthObjects are either CameraSensor or RadarSensor

#include "Dataset.h"
#include "ObjectMetaData.h"
#include "Dataset.h"
#include "Objects.h"
#include "SensorImage.h"
#include "ObjectImageShapeData.h"

class GroundTruthObjects : public Objects {

public:

    static unsigned groundTruthObjectTotalCount; // assingn object id

    void generate_object_base_point_displacement(ObjectMetaData gt_data);

    void generate_combined_sensor_data();

private:

    ushort m_startPoint;

    ObjectImageShapeData m_image_data_and_shape;

public:

    GroundTruthObjects() {}
    GroundTruthObjects( ObjectImageShapeData image_data_and_shape, ushort startPoint, std::unique_ptr<Noise> &noise, std::string objectName) : m_image_data_and_shape(image_data_and_shape), m_startPoint(startPoint), Objects(objectName) {

        m_objectId = groundTruthObjectTotalCount ;
        image_data_and_shape.process();
        image_data_and_shape.applyNoise(noise);
        groundTruthObjectTotalCount += 1;

    }

    void beginGroundTruthGeneration(ushort sensor_group_index, ObjectMetaData gt_data) {

        if ( m_objectName != "ObjectImageBackgroundShapeData") {

            printf("generating ground truth basic displacement for sensor %d name %s with object id %u\n", sensor_group_index, getObjectName().c_str(), getObjectId());

            generate_object_base_point_displacement(gt_data);

        }
    }

    void setDisjointSpecialRegionOfInterest(std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > all_object_combination_sensor_disjoint_special_region_of_interest) override {
        std::cout << "in derived" << std::endl;
        m_object_sroi_disjoint = all_object_combination_sensor_disjoint_special_region_of_interest;
    }



    std::vector<std::pair<cv::Point2f, cv::Point2f> >  get_object_base_point_displacement()
    const  {
        return m_object_base_point_displacement;
    }

    ObjectImageShapeData getImageShapeAndData() const {
        return m_image_data_and_shape;
    }

};



#endif //MAIN_OBJECTPROPERTIES_H
