//
// Created by veikas on 19.05.18.
//

#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <cassert>
#include "DataProcessingAlgorithm.h"
#include "Utils.h"
#include "Objects.h"


void DataProcessingAlgorithm::common(Objects *object) {

    for (ushort sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

        std::vector<OBJECTS_MEAN_STDDEV> multiframe_centroid_displacement;
        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > multiframe_dataprocessing_displacement;

        std::cout << "generate_object_mean_centroid_displacement for sensor_index "
                  << sensor_index << " for object name " << object->getObjectName() << " for algorithm " << m_algoName << std::endl;

        unsigned long FRAME_COUNT = object->get_object_stencil_point_displacement().at(sensor_index)
                .size();

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
            // gt_displacement

            std::vector<std::pair<cv::Point2f, cv::Point2f>> frame_dataprocessing_displacement;

            bool visibility = object->get_object_extrapolated_visibility().at(sensor_index).at(frame_count);

            std::cout << "frame_count " << frame_count << std::endl;

            if (visibility) {

                cv::Mat_<float> covar_new, mean_new, corr;
                cv::Scalar mean, stddev;

                const unsigned CLUSTER_SIZE = (unsigned) object->get_object_stencil_point_displacement().at
                        (sensor_index).at(frame_count).size();

                /*-----------------------------------------------------------------------------*/

                execute(object, sensor_index, frame_count, CLUSTER_SIZE, mean, stddev, frame_dataprocessing_displacement);

                /* ------------------------------------------------------------------------------ */

                multiframe_centroid_displacement.push_back({cv::Point2f(mean(0), mean(1)), cv::Point2f(mean(2), mean(3)),cv::Point2f(stddev(0), stddev(1)), cv::Point2f(stddev(2), stddev(3))});
                multiframe_dataprocessing_displacement.push_back(frame_dataprocessing_displacement);

                //std::cout << "mean_displacement and mean stddev " + m_algoName << multiframe_centroid_displacement.at(frame_count).mean_displacement << multiframe_centroid_displacement.at(frame_count).stddev_displacement<< std::endl;

                if (frame_count > 0) {
                    assert(std::abs(multiframe_centroid_displacement.at(frame_count).mean_displacement.x)> 0);
                    assert(std::abs(multiframe_centroid_displacement.at(frame_count).mean_displacement.y)> 0);
                }
            }

            else {

                std::cout << "not visible" << std::endl;
                multiframe_centroid_displacement.push_back({cv::Point2f(0, 0),cv::Point2f(0,0), cv::Point2f(0, 0), cv::Point2f(0, 0)});
                multiframe_dataprocessing_displacement.push_back({std::make_pair(cv::Point2f(0, 0),cv::Point2f(0,0))});
            }

        }

        m_sensor_multiframe_dataprocessing_centroid_displacement.push_back(multiframe_centroid_displacement);
        m_sensor_multiframe_dataprocessing_stencil_point_displacement.push_back(multiframe_dataprocessing_displacement);

    }
}


void NoAlgorithm::execute(Objects *object, ushort sensor_index, ushort frame_count, unsigned CLUSTER_SIZE,
                          cv::Scalar &mean, cv::Scalar &stddev,  std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement) {

    cv::Mat_<cv::Vec4f> samples(1, CLUSTER_SIZE, CV_32FC4);

    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

        cv::Point2f pts = object->get_object_stencil_point_displacement().at(sensor_index)
                .at(frame_count).at(cluster_index).first;
        cv::Point2f gt_displacement = object->get_object_stencil_point_displacement().at(
                sensor_index).at(frame_count).at(cluster_index).second;

        samples.at<cv::Vec4f>(0, cluster_index) = {pts.x, pts.y, gt_displacement.x, gt_displacement.y};

        frame_dataprocessing_displacement.push_back(std::make_pair(pts,gt_displacement));
    }

    cv::meanStdDev(samples, mean, stddev);

    cv::Mat covar_pts, covar_displacement;
    // Covariance matrix of the displacement dataset
    Utils::getCovarMatrix(samples, covar_pts, covar_displacement, mean);

    // the execute function returns a new cluster size. This could be lesser or more than the cluster size depending on the underlying algorithm

}

void SimpleAverage::execute(Objects *object, ushort sensor_index, ushort frame_count, unsigned CLUSTER_SIZE,
                            cv::Scalar &mean, cv::Scalar &stddev,  std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement) {


    cv::Mat_<cv::Vec4f> samples(1, CLUSTER_SIZE, CV_32FC4);



    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

        cv::Point2f pts = object->get_object_stencil_point_displacement().at(sensor_index)
                .at(frame_count).at(cluster_index).first;
        cv::Point2f gt_displacement = object->get_object_stencil_point_displacement().at(
                sensor_index).at(frame_count).at(cluster_index).second;

        samples.at<cv::Vec4f>(0, cluster_index) = {pts.x, pts.y, gt_displacement.x, gt_displacement.y};

        frame_dataprocessing_displacement.push_back(std::make_pair(pts,gt_displacement));
    }

    cv::meanStdDev(samples, mean, stddev);

    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {
        frame_dataprocessing_displacement.at(cluster_index).second = cv::Point2f(mean(2), mean(3));
    }

}


void MovingAverage::execute(Objects *object, ushort sensor_index, ushort frame_count, unsigned CLUSTER_SIZE,
                                  cv::Scalar &mean, cv::Scalar &stddev,  std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement) {


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
            ( mean(3) += gt_displacement.y )  /= 2;
        } else {
            mean(0) = pts.x;
            mean(1) = pts.y;
            mean(2) = gt_displacement.x;
            mean(3) = gt_displacement.y;
        }

        frame_dataprocessing_displacement.push_back(std::make_pair(pts,cv::Point2f(mean(2), mean(3))));
    }

}

void VotedMean::execute(Objects *object, ushort sensor_index, ushort frame_count, unsigned CLUSTER_SIZE, cv::Scalar &mean, cv::Scalar &stddev,  std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement) {



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

        frame_dataprocessing_displacement.push_back(std::make_pair(pts,gt_displacement));

    }

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

    //std::cout << "max_voted_x " << mean(2) << " max_voted_y " << mean(3) << std::endl;

    //std::cout << "max_voted_displacement_x " << max_voted_displacement_x << " max_voted_displacement_y " << max_voted_displacement_y << std::endl;

    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {
        frame_dataprocessing_displacement.at(cluster_index).second = cv::Point2f(mean(2), mean(3));
    }


}

void RankedMean::execute(Objects *object, ushort sensor_index, ushort frame_count, unsigned CLUSTER_SIZE, cv::Scalar &mean, cv::Scalar &stddev,  std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement) {


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

        frame_dataprocessing_displacement.push_back(std::make_pair(pts, gt_displacement));

    }

    const unsigned CLUSTER_EDGE_SIZE = (unsigned) object->get_object_edge_point_displacement().at
            (sensor_index).at(frame_count).size();

    ushort WEIGHT = 100;

    for (unsigned cluster_edge_index = 0; cluster_edge_index < CLUSTER_EDGE_SIZE; cluster_edge_index++) {

        cv::Point2f pts_edge = object->get_object_edge_point_displacement().at(
                sensor_index).at(frame_count).at(cluster_edge_index).first;
        cv::Point2f gt_displacement = object->get_object_edge_point_displacement().at(
                sensor_index).at(frame_count).at(cluster_edge_index).second;

        mean(0) += WEIGHT * pts_edge.x;
        mean(1) += WEIGHT * pts_edge.y;
        mean(2) += WEIGHT * gt_displacement.x;
        mean(3) += WEIGHT * gt_displacement.y;

    }

    mean(0) /= CLUSTER_SIZE + WEIGHT*CLUSTER_EDGE_SIZE;
    mean(1) /= CLUSTER_SIZE + WEIGHT*CLUSTER_EDGE_SIZE;
    mean(2) /= CLUSTER_SIZE + WEIGHT*CLUSTER_EDGE_SIZE;
    mean(3) /= CLUSTER_SIZE + WEIGHT*CLUSTER_EDGE_SIZE;

    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {
        frame_dataprocessing_displacement.at(cluster_index).second = cv::Point2f(mean(2), mean(3));
    }


}


void SensorFusion::execute(Objects *object, ushort sensor_index, ushort frame_count, unsigned CLUSTER_SIZE,
                                 cv::Scalar &mean, cv::Scalar &stddev,  std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement) {


    cv::Mat_<cv::Vec4f> samples(1, CLUSTER_SIZE, CV_32FC4);



    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {
        cv::Point2f pts = object->get_object_stencil_point_displacement().at(sensor_index)
                .at(frame_count).at(cluster_index).first;
        cv::Point2f gt_displacement = object->get_object_stencil_point_displacement().at(
                sensor_index).at(frame_count).at(cluster_index).second;

        samples.at<cv::Vec4f>(0, cluster_index) = {pts.x, pts.y, gt_displacement.x, gt_displacement.y};

        frame_dataprocessing_displacement.push_back(std::make_pair(pts,cv::Point2f(mean(2), mean(3))));
    }

    cv::meanStdDev(samples, mean, stddev);

    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {
        frame_dataprocessing_displacement.at(cluster_index).second = cv::Point2f(mean(2), mean(3));
    }



}
