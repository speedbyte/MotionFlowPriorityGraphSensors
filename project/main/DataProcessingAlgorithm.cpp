//
// Created by veikas on 19.05.18.
//

#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <cassert>
#include "DataProcessingAlgorithm.h"
#include "Utils.h"


void DataProcessingAlgorithm::common(Objects *object) {

    for (ushort sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

        std::vector<std::pair<cv::Point2f, cv::Point2f> >
                multiframe_centroid_displacement;

        std::cout << "generate_object_mean_centroid_displacement for sensor_index "
                  << sensor_index
                  << " for object name " << object->getObjectName() << " for algorithm " << m_algoName << std::endl;

        unsigned long FRAME_COUNT = object->get_object_stencil_point_displacement().at(sensor_index)
                .size();

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
            // gt_displacement

            std::cout << "frame_count " << frame_count << std::endl;

            bool visibility = object->get_object_extrapolated_visibility().at(sensor_index).at(frame_count);

            if (visibility) {

                cv::Mat_<float> covar_new, mean_new, corr;
                cv::Scalar mean, stddev;

                const unsigned CLUSTER_SIZE = (unsigned) object->get_object_stencil_point_displacement().at
                        (sensor_index).at(frame_count).size();

                

                /*-----------------------------------------------------------------------------*/

                mean = execute(object, sensor_index, frame_count, CLUSTER_SIZE);

                /* ------------------------------------------------------------------------------ */

                std::cout << "mean_displacement " + m_algoName
                          << cv::Point2f(mean(2), mean(3)) << std::endl;

                if (frame_count > 0) {
                    assert(std::abs(mean(2))> 0);
                    assert(std::abs(mean(3))> 0);
                }

                multiframe_centroid_displacement.push_back(std::make_pair(cv::Point2f(mean(0), mean(1)),
                                                                          cv::Point2f(mean(2), mean(3))
                ));
            }

            else {

                std::cout << "not visible" << std::endl;

                multiframe_centroid_displacement.push_back(
                        std::make_pair(cv::Point2f(0, 0),cv::Point2f(0,0)
                        ));
            }
        }

        m_sensor_multiframe_centroid_displacement.push_back(multiframe_centroid_displacement);

    }
}


cv::Scalar RankedMean::execute(Objects *object, ushort sensor_index, ushort frame_count, unsigned CLUSTER_SIZE) {

    cv::Scalar mean;
    cv::Mat_<cv::Vec4f> samples(1, CLUSTER_SIZE, CV_32FC4);
    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

        cv::Point2f pts = object->get_object_stencil_point_displacement().at(sensor_index)
                .at(frame_count).at(cluster_index).first;
        cv::Point2f gt_displacement = object->get_object_stencil_point_displacement().at(
                sensor_index).at(frame_count).at(cluster_index).second;

        //samples.at<cv::Vec4f>(0, cluster_index) = {pts.x, pts.y, gt_displacement.x, gt_displacement.y};

        mean(0) += pts.x;
        mean(1) += pts.y;
        mean(2) += gt_displacement.x;
        mean(3) += gt_displacement.y;

    }

    const unsigned CLUSTER_EDGE_SIZE = (unsigned) object->get_object_edge_point_displacement().at
            (sensor_index).at(frame_count).size();

    ushort WEIGHT = 100;

    for (unsigned cluster_edge_index = 0; cluster_edge_index < CLUSTER_EDGE_SIZE; cluster_edge_index++) {

        cv::Point2f pts_edge = object->get_object_edge_point_displacement().at(
                        sensor_index)
                .at(frame_count).at(cluster_edge_index).first;
        cv::Point2f gt_displacement = object->get_object_edge_point_displacement().at(
                        sensor_index)
                .at(frame_count).at(cluster_edge_index).second;

        mean(0) += WEIGHT * pts_edge.x;
        mean(1) += WEIGHT * pts_edge.y;
        mean(2) += WEIGHT * gt_displacement.x;
        mean(3) += WEIGHT * gt_displacement.y;

    }


    mean(0) /= CLUSTER_SIZE + WEIGHT*CLUSTER_EDGE_SIZE;
    mean(1) /= CLUSTER_SIZE + WEIGHT*CLUSTER_EDGE_SIZE;
    mean(2) /= CLUSTER_SIZE + WEIGHT*CLUSTER_EDGE_SIZE;
    mean(3) /= CLUSTER_SIZE + WEIGHT*CLUSTER_EDGE_SIZE;

    return mean;

}


cv::Scalar MovingAverage::execute(Objects *object, ushort sensor_index, ushort frame_count, unsigned CLUSTER_SIZE) {

    cv::Scalar mean;
    cv::Mat_<cv::Vec4f> samples(1, CLUSTER_SIZE, CV_32FC4);

    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

        cv::Point2f pts = object->get_object_stencil_point_displacement().at(sensor_index)
                .at(frame_count).at(cluster_index).first;
        cv::Point2f gt_displacement = object->get_object_stencil_point_displacement().at(
                sensor_index).at(frame_count).at(cluster_index).second;

        samples.at<cv::Vec4f>(0, cluster_index) = {pts.x, pts.y, gt_displacement.x, gt_displacement.y};

        if (cluster_index > 0) {
            ( mean(0) += pts.x ) /= 2 ;
            ( mean(1) += pts.y )  /= 2;
            ( mean(2) += gt_displacement.x )  /= 2;
            ( mean(3) += gt_displacement.x )  /= 2;
        } else {
            mean(0) = pts.x;
            mean(1) = pts.y;
            mean(2) = gt_displacement.x;
            mean(3) = gt_displacement.y;
        }

    }

    return mean;

}

cv::Scalar VotedMean::execute(Objects *object, ushort sensor_index, ushort frame_count, unsigned CLUSTER_SIZE) {

    cv::Scalar mean, stddev;
    cv::Mat_<cv::Vec4f> samples(1, CLUSTER_SIZE, CV_32FC4);

    std::vector<float> data_x, data_y;
    std::vector<float> data_x_pts, data_y_pts;

    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

        cv::Point2f pts = object->get_object_stencil_point_displacement().at(sensor_index)
                .at(frame_count).at(cluster_index).first;
        cv::Point2f gt_displacement = object->get_object_stencil_point_displacement().at(
                sensor_index).at(frame_count).at(cluster_index).second;

        samples.at<cv::Vec4f>(0, cluster_index) = {pts.x, pts.y, gt_displacement.x, gt_displacement.y};

        // get samples from template
        data_x.push_back(std::round(gt_displacement.x * 1000) / 1000);
        data_y.push_back(std::round(gt_displacement.y * 1000) / 1000);

        data_x_pts.push_back(pts.x);
        data_y_pts.push_back(pts.y);

    }

    cv::meanStdDev(samples, mean, stddev);

    std::vector<cv::Mat> histogram_x, histogram_y;

    if (frame_count > 0) {
        Utils::drawHistogram(data_x, histogram_x, false);
        mean(2) = Utils::getHistogramPeak(histogram_x);
        Utils::drawHistogram(data_y, histogram_y, false);
        mean(3) = Utils::getHistogramPeak(histogram_y);
    } else {
        mean(2) = 0;
        mean(3) = 0;
    }

    //std::cout << "max_voted_displacement_x " << max_voted_displacement_x << " max_voted_displacement_y " << max_voted_displacement_y << std::endl;

    return mean;

}


cv::Scalar SimpleAverage::execute(Objects *object, ushort sensor_index, ushort frame_count, unsigned CLUSTER_SIZE) {

    cv::Scalar mean,stddev;
    cv::Mat_<cv::Vec4f> samples(1, CLUSTER_SIZE, CV_32FC4);
    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {
        cv::Point2f pts = object->get_object_stencil_point_displacement().at(sensor_index)
                .at(frame_count).at(cluster_index).first;
        cv::Point2f gt_displacement = object->get_object_stencil_point_displacement().at(
                sensor_index).at(frame_count).at(cluster_index).second;

        samples.at<cv::Vec4f>(0, cluster_index) = {pts.x, pts.y, gt_displacement.x, gt_displacement.y};
    }
    cv::meanStdDev(samples, mean, stddev);
    //cv::calcCovarMatrix(samples, covar, mean, cv::COVAR_NORMAL | cv::COVAR_COLS | cv::COVAR_SCALE, CV_32FC1 );
    return mean;

}


cv::Scalar SensorFusion::execute(Objects *object, ushort sensor_index, ushort frame_count, unsigned CLUSTER_SIZE) {

    cv::Scalar mean, stddev;
    cv::Mat_<cv::Vec4f> samples(1, CLUSTER_SIZE, CV_32FC4);
    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {
        cv::Point2f pts = object->get_object_stencil_point_displacement().at(sensor_index)
                .at(frame_count).at(cluster_index).first;
        cv::Point2f gt_displacement = object->get_object_stencil_point_displacement().at(
                sensor_index).at(frame_count).at(cluster_index).second;

        samples.at<cv::Vec4f>(0, cluster_index) = {pts.x, pts.y, gt_displacement.x, gt_displacement.y};
    }
    cv::meanStdDev(samples, mean, stddev);
    //cv::calcCovarMatrix(samples, covar, mean, cv::COVAR_NORMAL | cv::COVAR_COLS | cv::COVAR_SCALE, CV_32FC1 );
    return mean;

}
