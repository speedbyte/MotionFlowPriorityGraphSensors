//
// Created by veikas on 19.05.18.
//

#ifndef MAIN_DATAPROCESSINGALGORITHM_H
#define MAIN_DATAPROCESSINGALGORITHM_H

#include "Objects.h"

class DataProcessingAlgorithm {

protected:

    std::string m_algoName;
    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            m_sensor_multiframe_centroid_displacement;

public:

    DataProcessingAlgorithm(std::string algoName): m_algoName(algoName) {

    }
    void common(Objects *objects);

    virtual cv::Scalar execute(Objects *object, ushort sensor_index, ushort frame_count, unsigned CLUSTER_SIZE) {
        std::cout << "not implemented" << std::endl;
        throw;
    }

    const std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > &getObjectMeanCentroidDisplacement() const {
        return m_sensor_multiframe_centroid_displacement;
    }

};


class RankedMean : public DataProcessingAlgorithm {

private:

public:

    RankedMean() : DataProcessingAlgorithm("ranked mean") {}

    cv::Scalar execute(Objects *object, ushort sensor_index, ushort frame_count, unsigned CLUSTER_SIZE) override;
};


class VotedMean : public DataProcessingAlgorithm {

private:

public:

    VotedMean() : DataProcessingAlgorithm("voted mean") {}

    cv::Scalar execute(Objects *object, ushort sensor_index, ushort frame_count, unsigned CLUSTER_SIZE) override;
};


class SimpleAverage : public DataProcessingAlgorithm {

public:

    SimpleAverage() : DataProcessingAlgorithm("simple average") {}

    cv::Scalar execute(Objects *object, ushort sensor_index, ushort frame_count, unsigned CLUSTER_SIZE) override;
};


class MovingAverage : public DataProcessingAlgorithm {

public:

    MovingAverage() : DataProcessingAlgorithm("moving average") {}

    cv::Scalar execute(Objects *object, ushort sensor_index, ushort frame_count, unsigned CLUSTER_SIZE) override;
};


class SensorFusion : public DataProcessingAlgorithm {

public:

    SensorFusion() : DataProcessingAlgorithm("sensor fusion") {}

    cv::Scalar execute(Objects *object, ushort sensor_index, ushort frame_count, unsigned CLUSTER_SIZE) override;
};


/*
 * const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &obj__blob_point_displacement
*/

#endif //MAIN_DATAPROCESSINGALGORITHM_H
