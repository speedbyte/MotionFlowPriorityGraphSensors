//
// Created by veikas on 28.01.18.
//

#ifndef MAIN_OBJECTPROPERTIES_H
#define MAIN_OBJECTPROPERTIES_H


// Objects are either CameraSensorImage or RadarSensorImage

#include "Dataset.h"
#include "ObjectTrajectory.h"
#include "datasets.h"
#include "CameraSensorImage.h"

class Objects : public CameraSensorImage {

private:

    
    const ushort m_startPoint;

    int m_ObjectWidth;

    int m_ObjectHeight;

    static unsigned objectCurrentCount; // assingn object id

    const ObjectTrajectory &m_obj_trajectory; // TODO use a own copy instead of reference.

    const unsigned m_objectId;

    const std::string m_objectName;

    std::vector<std::pair<cv::Point2f, cv::Point2f> > m_obj_base_pixel_point_pixel_displacement;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > m_obj_extrapolated_pixel_point_pixel_displacement;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            m_obj_extrapolated_shape_pixel_point_pixel_displacement;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            m_obj_extrapolated_pixel_centroid_pixel_displacement_mean;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            m_obj_line_parameters;

    std::vector<bool> m_obj_base_visibility;

    std::vector<std::vector<bool> >  m_obj_extrapolated_visibility;

    std::vector<std::vector<std::vector<bool> > > m_obj_extrapolated_shape_visibility;

    std::vector<std::vector<bool> > m_obj_extrapolated_mean_visibility;

    std::vector<std::pair<cv::Point2f, cv::Point2f> >  getBasePixelpoint_pixelDisplacement() const {
        return m_obj_base_pixel_point_pixel_displacement;
    }

    void generate_obj_base_pixel_point_pixel_displacement();

    void generate_obj_extrapolated_pixel_point_pixel_displacement(const unsigned &max_skips);

    void generate_obj_extrapolated_pixel_centroid_pixel_displacement_mean(const unsigned &max_skips);

    void generate_obj_extrapolated_shape_pixel_point_pixel_displacement(const unsigned &max_skips);

public:

    Objects( ObjectImageShapeData &image_data_and_shape, const ObjectTrajectory &trajectory, ushort startPoint, Noise
    &noise, std::string objectName) : CameraSensorImage(image_data_and_shape, noise), m_obj_trajectory
            (trajectory), m_startPoint(startPoint) , m_objectName(objectName), m_objectId (objectCurrentCount)
    {


        image_data_and_shape.process();
        objectCurrentCount += 1;

        m_ObjectWidth = image_data_and_shape.get().cols;
        m_ObjectHeight = image_data_and_shape.get().rows;

        if ( objectName.compare("BackgroundCanvas")) {

            printf("generating ground truth basic displacement for name %s with object id %u\n", getObjectName().c_str
                    (), getObjectId());

            generate_obj_base_pixel_point_pixel_displacement();

            printf("generating ground truth frame displacement for name %s with object id %u\n", getObjectName().c_str
                    (), getObjectId());

            generate_obj_extrapolated_pixel_point_pixel_displacement( MAX_SKIPS );

            generate_obj_extrapolated_shape_pixel_point_pixel_displacement(MAX_SKIPS);

            generate_obj_extrapolated_pixel_centroid_pixel_displacement_mean( MAX_SKIPS );
        }
    }


    int getWidth() const {
        return m_ObjectWidth;
    }

    int getHeight() const {
        return m_ObjectHeight;
    }

    ObjectTrajectory getTrajectoryPoints() const {
        return m_obj_trajectory;
    }

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >  get_obj_extrapolated_pixel_point_pixel_displacement()
    const {
        return m_obj_extrapolated_pixel_point_pixel_displacement;
    }

    std::vector<std::vector<bool> >  get_obj_extrapolated_visibility()
    const {
        return m_obj_extrapolated_visibility;
    }


    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >  get_obj_extrapolated_pixel_centroid_pixel_displacement_mean()
    const {
        return m_obj_extrapolated_pixel_centroid_pixel_displacement_mean;
    }

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >  get_line_parameters() const {
        return m_obj_line_parameters;
    }

    ushort getStartPoint() const {
        return m_startPoint;
    }

    unsigned getObjectId() const {
        return m_objectId;
    }

    std::string getObjectName() const {
        return m_objectName;
    }
};



#endif //MAIN_OBJECTPROPERTIES_H
