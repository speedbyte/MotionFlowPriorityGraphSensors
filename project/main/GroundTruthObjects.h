//
// Created by veikas on 28.01.18.
//

#ifndef MAIN_OBJECTPROPERTIES_H
#define MAIN_OBJECTPROPERTIES_H


// GroundTruthObjects are either CameraSensorImage or RadarSensorImage

#include "Dataset.h"
#include "ObjectTrajectory.h"
#include "datasets.h"
#include "CameraSensorImage.h"
#include "Objects.h"

class GroundTruthObjects : public CameraSensorImage, public Objects {

public:
    static unsigned objectCurrentCount; // assingn object id

private:

    const ushort m_startPoint;

    const ObjectTrajectory m_obj_trajectory;

    const ObjectDimensions m_obj_dimensions;

    std::vector<std::pair<cv::Point2f, cv::Point2f> > m_obj_base_pixel_point_pixel_displacement;

    std::vector<cv::Point2f> m_obj_base_shape_dimensions;

    void generate_obj_base_pixel_point_pixel_displacement();

    void generate_obj_base_shape_dimensions();

    void generate_obj_extrapolated_pixel_point_pixel_displacement(const unsigned &max_skips);

    void generate_obj_extrapolated_shape_pixel_point_pixel_displacement_pixel_visibility(const unsigned &max_skips);

    void generate_obj_extrapolated_stencil_pixel_point_pixel_displacement(std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_stencil_movement  ) override;

public:

    GroundTruthObjects( ObjectImageShapeData image_data_and_shape, const ObjectTrajectory trajectory, ushort
    startPoint, Noise &noise, std::string objectName) : m_obj_trajectory(trajectory), m_startPoint(startPoint),CameraSensorImage(image_data_and_shape, noise),
                                                        Objects(objectName)

    {
        assert(m_obj_trajectory.getTrajectory().size() >= MAX_ITERATION_GT_SCENE_GENERATION_VECTOR);
        m_objectId = objectCurrentCount ;
        image_data_and_shape.process();
        objectCurrentCount += 1;

        m_ObjectWidth = image_data_and_shape.get().cols;
        m_ObjectHeight = image_data_and_shape.get().rows;
        if ( m_objectName.compare("BackgroundCanvas")) {

            printf("generating ground truth basic displacement for name %s with object id %u\n", getObjectName().c_str
                    (), getObjectId());

            generate_obj_base_pixel_point_pixel_displacement();

            generate_obj_base_shape_dimensions();

            beginGroundTruthGeneration();

        }
    }

    GroundTruthObjects( ObjectImageShapeData image_data_and_shape, const ObjectDimensions dimension, const ObjectTrajectory trajectory, ushort
    startPoint, Noise &noise, std::string objectName) : m_obj_dimensions(dimension), m_obj_trajectory(trajectory), m_startPoint(startPoint),CameraSensorImage(image_data_and_shape, noise),
                                                        Objects(objectName)

    {
        assert(m_obj_trajectory.getTrajectory().size() >= MAX_ITERATION_GT_SCENE_GENERATION_VECTOR);
        m_objectId = objectCurrentCount ;
        image_data_and_shape.process();
        objectCurrentCount += 1;

        m_ObjectWidth = image_data_and_shape.get().cols;
        m_ObjectHeight = image_data_and_shape.get().rows;
        if ( m_objectName.compare("BackgroundCanvas")) {

            printf("generating ground truth basic displacement for name %s with object id %u\n", getObjectName().c_str
                    (), getObjectId());

            generate_obj_base_pixel_point_pixel_displacement();

            generate_obj_base_shape_dimensions();

            beginGroundTruthGeneration();

        }
    }

    void beginGroundTruthGeneration() {

        if ( m_objectName.compare("BackgroundCanvas")) {

            printf("generating ground truth basic displacement for name %s with object id %u\n", getObjectName().c_str
                    (), getObjectId());

            generate_obj_extrapolated_pixel_point_pixel_displacement( MAX_SKIPS );

            generate_obj_extrapolated_shape_pixel_point_pixel_displacement_pixel_visibility(MAX_SKIPS);

            // m_obj_extrapolated_shape_pixel_point_pixel_displacement
            generate_obj_extrapolated_mean_pixel_centroid_pixel_displacement( MAX_SKIPS , m_obj_extrapolated_shape_pixel_point_pixel_displacement, m_obj_extrapolated_shape_visibility);

            generate_obj_line_parameters(MAX_SKIPS);
        }
    }

    ObjectTrajectory getTrajectoryPoints() const {
        return m_obj_trajectory;
    }

    std::vector<std::pair<cv::Point2f, cv::Point2f> >  get_obj_base_pixel_point_pixel_displacement()
    const  {
        return m_obj_base_pixel_point_pixel_displacement;
    }

    std::vector<cv::Point2f> get_obj_base_shape_dimensions()
    const  {
        return m_obj_base_shape_dimensions;
    }

    ushort getStartPoint() const {
        return m_startPoint;
    }

};



#endif //MAIN_OBJECTPROPERTIES_H
