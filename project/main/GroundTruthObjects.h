//
// Created by veikas on 28.01.18.
//

#ifndef MAIN_OBJECTPROPERTIES_H
#define MAIN_OBJECTPROPERTIES_H


// GroundTruthObjects are either CameraSensor or RadarSensor

#include "Dataset.h"
#include "ObjectMetaData.h"
#include "datasets.h"
#include "Objects.h"
#include "SensorImage.h"

class GroundTruthObjects : public Objects {

public:
    static unsigned objectCurrentCount; // assingn object id

    void generate_obj_base_pixel_position_pixel_displacement(ObjectMetaData gt_data);

private:

    const ushort m_startPoint;

    ObjectImageShapeData m_image_data_and_shape;

    void generate_obj_extrapolated_pixel_position_pixel_displacement();

    void generate_obj_extrapolated_shape_pixel_point_pixel_displacement_pixel_visibility();

    void generate_obj_extrapolated_stencil_pixel_point_pixel_displacement(std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_stencil_movement  ) override;

public:

    GroundTruthObjects( ObjectImageShapeData image_data_and_shape, ushort startPoint, Noise noise, std::string objectName) : m_image_data_and_shape(image_data_and_shape), m_startPoint(startPoint), Objects(objectName)

    {
        m_objectId = objectCurrentCount ;
        image_data_and_shape.process();
        image_data_and_shape.applyNoise(&noise);
        objectCurrentCount += 1;

    }

    void beginGroundTruthGeneration(ObjectMetaData gt_data) {

        if ( m_objectName != "BackgroundCanvas") {

            printf("generating ground truth basic displacement for name %s with object id %u\n", getObjectName().c_str
                    (), getObjectId());

            generate_obj_base_pixel_position_pixel_displacement(gt_data);

            generate_obj_extrapolated_pixel_position_pixel_displacement();

            generate_obj_extrapolated_shape_pixel_point_pixel_displacement_pixel_visibility();
        }
    }

    std::vector<std::pair<cv::Point2f, cv::Point2f> >  get_obj_base_pixel_position_pixel_displacement()
    const  {
        return m_obj_base_pixel_position_pixel_displacement;
    }

    ObjectImageShapeData getImageShapeAndData() const {
        return m_image_data_and_shape;
    }

};



#endif //MAIN_OBJECTPROPERTIES_H
