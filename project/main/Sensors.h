//
// Created by veikas on 08.02.18.
//

#ifndef MAIN_SENSORS_H
#define MAIN_SENSORS_H


#include <opencv2/core/types.hpp>
#include "SensorMetaData.h"

class Sensors {

private:

protected:

    std::vector<STRUCT_GT_SENSORS_ALL> m_sen_base_all;

    std::vector<std::vector<STRUCT_GT_SENSORS_ALL> > m_sen_extrapolated_all;

    int m_SensorInertialWidth;

    int m_SensorInertialHeight;

    unsigned m_sensorId;

    std::string m_sensorName;

    ushort m_startPoint;

    std::vector<bool> m_sen_base_visibility;

    std::vector<std::vector<bool> >  m_sen_extrapolated_visibility;


public:

    Sensors (const SensorMetaData gt_data, ushort startPoint, Noise *noise, const std::string sensorName) : m_startPoint(startPoint), m_sensorName(sensorName) {

        generate_sen_base_pixel_position_pixel_displacement(gt_data);

        generate_sen_extrapolated_pixel_position_pixel_displacement(MAX_SKIPS);

        //image_data_and_shape.process();

    }

    const std::string &getSensorName() const {
        return m_sensorName;
    }

    const int &getInertialWidth() const {
        return m_SensorInertialWidth;
    }

    const int &getInertialHeight() const {
        return m_SensorInertialHeight;
    }

    unsigned getSensorId() const {
        return m_sensorId;
    }

    const std::vector<std::vector<STRUCT_GT_SENSORS_ALL> > &getExtrapolatedGroundTruthDetails() const {
        return m_sen_extrapolated_all;
    }

    ushort& getSensorStartPoint() {
        return m_startPoint;
    }

    void setSensorName(std::string sensorName) {
        m_sensorName = sensorName;
    }

    void generate_sen_base_pixel_position_pixel_displacement(SensorMetaData gt_data);

    void generate_sen_extrapolated_pixel_position_pixel_displacement(const unsigned &max_skips);

    const std::vector<bool>  &get_sen_base_visibility()    const {
        return m_sen_base_visibility;
    }

    
};


#endif //MAIN_SENSORS_H
