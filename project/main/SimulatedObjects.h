//
// Created by veikas on 07.02.18.
//

#ifndef MAIN_SIMULATEDOBJECTS_H
#define MAIN_SIMULATEDOBJECTS_H


#include <opencv2/core/types.hpp>

class SimulatedObjects {
private:

    static unsigned SimulatedobjectCurrentCount; // assingn object id

    const unsigned m_objectId;

    const std::string m_objectName;

    int m_simulatedObjectWidth;

    int m_simulatedObjectHeight;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > m_algo_frame_pixel_point_pixel_displacement;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            m_simulated_obj_extrapolated_shape_pixel_point_pixel_displacement;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            m_simulated_obj_extrapolated_pixel_centroid_pixel_displacement_mean;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            m_simulated_obj_line_parameters;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > outer_base_movement;

public:

    SimulatedObjects(unsigned objectId, std::string objectName, int width, int height) :
            m_objectId(objectId), m_objectName(objectName), m_simulatedObjectWidth(width), m_simulatedObjectHeight
            (height) {
        SimulatedobjectCurrentCount += 1;
    }

    void generate_simulated_obj_extrapolated_pixel_centroid_pixel_displacement_mean(const unsigned &max_skips);

    const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f>>>> &
    get_simulated_obj_extrapolated_shape_pixel_point_pixel_displacement() const {
        return m_simulated_obj_extrapolated_shape_pixel_point_pixel_displacement;
    }

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >  getLineParameters() const {
        return m_simulated_obj_line_parameters;
    }

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
    getSimulatedExtrapolatedPixelCentroid_DisplacementMean()
    const {
        return m_simulated_obj_extrapolated_pixel_centroid_pixel_displacement_mean;
    }

    unsigned getObjectId() const {
        return m_objectId;
    }

    std::string getObjectName() const {
        return m_objectName;
    }

    void set_outer_base_movement(std::vector<std::pair<cv::Point2f, cv::Point2f> > base_movement) {
        outer_base_movement.push_back(base_movement);
    }

    void trigger_m_simulated_obj_extrapolated_shape_pixel_point_pixel_displacement() {
        m_simulated_obj_extrapolated_shape_pixel_point_pixel_displacement.push_back(outer_base_movement);
        outer_base_movement.clear();
    }

    int getWidth() const {
        return m_simulatedObjectWidth;
    }

    int getHeight() const {
        return m_simulatedObjectHeight;
    }


};


#endif //MAIN_SIMULATEDOBJECTS_H
