//
// Created by veikas on 19.05.18.
//

#ifndef MAIN_DATAPROCESSINGALGORITHM_H
#define MAIN_DATAPROCESSINGALGORITHM_H

#include "Objects.h"

class DataProcessingAlgorithm {

protected:

    std::string m_algoName;
    std::vector<std::vector<OBJECTS_MEAN_STDDEV > > m_sensor_multiframe_dataprocessing_centroid_displacement;
    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > m_sensor_multiframe_dataprocessing_stencil_point_displacement;

public:

    DataProcessingAlgorithm(std::string algoName): m_algoName(algoName) {

    }
    void common(Objects *objects, std::string post_processing_algorithm);

    virtual void execute(Objects *object, ushort sensor_index, ushort current_frame_index, unsigned CLUSTER_SIZE, cv::Scalar &mean, cv::Scalar &stddev, std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement, cv::Mat_<cv::Vec4f> &samples) {
        std::cout << "not implemented" << std::endl;
        throw;
    }

    const std::vector<std::vector<OBJECTS_MEAN_STDDEV > > &get_object_dataprocessing_mean_centroid_displacement() const {
        return m_sensor_multiframe_dataprocessing_centroid_displacement;
    }

    const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f>> > > &get_object_dataprocessing_stencil_point_displacement() const {
        return m_sensor_multiframe_dataprocessing_stencil_point_displacement;
    }

};


class RankedMean : public DataProcessingAlgorithm {

private:

public:

    RankedMean() : DataProcessingAlgorithm("ranked mean") {}

    void execute(Objects *object, ushort sensor_index, ushort current_frame_index, unsigned CLUSTER_SIZE,
                       cv::Scalar &mean, cv::Scalar &stddev, std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement, cv::Mat_<cv::Vec4f> &samples) override;
};


class VotedMean : public DataProcessingAlgorithm {

private:

public:

    VotedMean() : DataProcessingAlgorithm("voted mean") {}

    void execute(Objects *object, ushort sensor_index, ushort current_frame_index, unsigned CLUSTER_SIZE,
                       cv::Scalar &mean, cv::Scalar &stddev, std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement, cv::Mat_<cv::Vec4f> &samples) override;
};


class SimpleAverage : public DataProcessingAlgorithm {

public:

    SimpleAverage() : DataProcessingAlgorithm("simple average") {}

    void execute(Objects *object, ushort sensor_index, ushort current_frame_index, unsigned CLUSTER_SIZE,
                       cv::Scalar &mean, cv::Scalar &stddev, std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement, cv::Mat_<cv::Vec4f> &samples) override;
};


class MovingAverage : public DataProcessingAlgorithm {

public:

    MovingAverage() : DataProcessingAlgorithm("moving average") {}

    void execute(Objects *object, ushort sensor_index, ushort current_frame_index, unsigned CLUSTER_SIZE,
                       cv::Scalar &mean, cv::Scalar &stddev, std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement, cv::Mat_<cv::Vec4f> &samples) override;
};


class NoAlgorithm: public DataProcessingAlgorithm {

public:

    NoAlgorithm() : DataProcessingAlgorithm("no_algorithm") {}

    void execute(Objects *object, ushort sensor_index, ushort current_frame_index, unsigned CLUSTER_SIZE,
                 cv::Scalar &mean, cv::Scalar &stddev, std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement, cv::Mat_<cv::Vec4f> &samples) override;
};

class SensorFusion : public DataProcessingAlgorithm {

public:

    SensorFusion() : DataProcessingAlgorithm("sensor fusion") {}

    void execute(Objects *object, ushort sensor_index, ushort current_frame_index, unsigned CLUSTER_SIZE,
                       cv::Scalar &mean, cv::Scalar &stddev, std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement, cv::Mat_<cv::Vec4f> &samples) override;
};


/*
 * const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &obj__blob_point_displacement
*/

#endif //MAIN_DATAPROCESSINGALGORITHM_H
