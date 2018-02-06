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

    static unsigned objectCurrentCount; // assingn object id

    ObjectTrajectory &m_obj_trajectory;

    const unsigned m_objectId;

    const std::string m_objectName;

    std::vector<std::pair<cv::Point2f, cv::Point2f> > m_obj_base_pixel_point_pixel_displacement;
    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > m_obj_extrapolated_pixel_point_pixel_displacement;
    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >m_obj_extrapolated_shape_pixel_point_pixel_displacement;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            m_obj_extrapolated_pixel_centroid_pixel_displacement_mean;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            m_obj_line_parameters;



public:

    Objects( ObjectImageShapeData &image_data_and_shape, ObjectTrajectory &trajectory, ushort startPoint, Noise
    &noise, std::string objectName ) : CameraSensorImage(image_data_and_shape, noise), m_obj_trajectory
            (trajectory), m_startPoint(startPoint) , m_objectName ( objectName ), m_objectId
                                               (objectCurrentCount) {

        image_data_and_shape.process();
        trajectory.process(Dataset::getFrameSize());
        objectCurrentCount += 1;

        if ( objectName.compare("BackgroundCanvas")) {
            printf("generating ground truth basic displacement for name %s with object id %u\n", getObjectName().c_str
                    (), getObjectId());

            generate_obj_base_pixel_point_pixel_displacement( m_startPoint, m_obj_trajectory.get());

            printf("generating ground truth frame displacement for name %s with object id %u\n", getObjectName().c_str
                    (), getObjectId());

            generate_obj_extrapolated_pixel_point_pixel_displacement( MAX_SKIPS );

            generate_obj_extrapolated_shape_pixel_point_pixel_displacement(MAX_SKIPS);

            generate_obj_extrapolated_pixel_centroid_pixel_displacement_mean( MAX_SKIPS );
        }
    }



    void generate_obj_base_pixel_point_pixel_displacement(const ushort &start_point, const std::vector<cv::Point2f>
    &trajectory_points);

    void generate_obj_extrapolated_pixel_point_pixel_displacement(const unsigned &max_skips);

    void generate_obj_extrapolated_pixel_centroid_pixel_displacement_mean(const unsigned &max_skips);

    void generate_obj_extrapolated_shape_pixel_point_pixel_displacement(const unsigned &max_skips);

    ObjectTrajectory getTrajectoryPoints() {
        return m_obj_trajectory;
    }

    std::vector<std::pair<cv::Point2f, cv::Point2f> >  getBasePixelpoint_pixelDisplacement() {
        return m_obj_base_pixel_point_pixel_displacement;
    }

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >  getExtrapolatedPixelpoint_pixelDisplacement() {
        return m_obj_extrapolated_pixel_point_pixel_displacement;
    }

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >  getExtrapolatedPixelCentroid_DisplacementMean() {
        return m_obj_extrapolated_pixel_centroid_pixel_displacement_mean;
    }

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >  getLineParameters() {
        return m_obj_line_parameters;
    }

    ushort getStartPoint() {
        return m_startPoint;
    }

    unsigned getObjectId() {
        return m_objectId;
    }

    std::string getObjectName() {
        return m_objectName;
    }
};



#endif //MAIN_OBJECTPROPERTIES_H
