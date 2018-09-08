//
// Created by veikas on 08.02.18.
//

#ifndef MAIN_SENSORS_H
#define MAIN_SENSORS_H


#include <opencv2/core/types.hpp>
#include "SensorMetaData.h"
#include "Noise.h"

class Sensors {

private:

    void generate_sen_base_point_displacement(SensorMetaData gt_data);

protected:

    std::vector<STRUCT_GT_SENSORS_ALL> m_sen_base_all;

    std::vector<std::vector<STRUCT_GT_SENSORS_ALL> > m_sen__all;

    unsigned m_sensorId;

    std::string m_sensorName;

    ushort m_startPoint;

    std::vector<bool> m_sen_base_visibility;

    std::vector<std::vector<bool> >  m_sen__visibility;

public:

    Sensors (ushort startPoint, std::unique_ptr<Noise> &noise, const std::string sensorName) : m_startPoint(startPoint), m_sensorName(sensorName) {

        //image_data_and_shape.process();

    }

    void beginGroundTruthGeneration(SensorMetaData &gt_data) {

        generate_sen_base_point_displacement(gt_data);

        //generate_sen__pixel_position_pixel_displacement();
    }

    const std::string &getSensorName() const {
        return m_sensorName;
    }

    unsigned getSensorId() const {
        return m_sensorId;
    }

    const std::vector<std::vector<STRUCT_GT_SENSORS_ALL> > &getExtrapolatedGroundTruthDetails() const {
        return m_sen__all;
    }

    ushort& getSensorStartPoint() {
        return m_startPoint;
    }

    void setSensorName(std::string sensorName) {
        m_sensorName = sensorName;
    }

    void generate_sen__pixel_position_pixel_displacement();

    const std::vector<bool>  &get_sen_base_visibility()    const {
        return m_sen_base_visibility;
    }

    
};


#endif //MAIN_SENSORS_H
