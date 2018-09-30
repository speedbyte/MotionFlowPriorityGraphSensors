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


void DataProcessingAlgorithm::common(Objects *object, std::string post_processing_algorithm) {

    for (ushort sensor_index = 0; sensor_index < Dataset::SENSOR_COUNT; sensor_index++) {

        std::vector<OBJECTS_MEAN_STDDEV> multiframe_object_results;
        std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > multiframe_dataprocessing_displacement;

        std::cout << "generate_object_mean_centroid_displacement for sensor_index "
                  << sensor_index << " for object name " << object->getObjectName() << " for algorithm " << m_algoName << std::endl;

        unsigned long FRAME_COUNT = object->get_object_stencil_point_displacement().at(sensor_index)
                .size();

        for (ushort current_frame_index = 0; current_frame_index < FRAME_COUNT; current_frame_index++) {
            // gt_displacement

            std::vector<std::pair<cv::Point2f, cv::Point2f>> frame_dataprocessing_displacement;

            bool visibility = object->get_object_extrapolated_visibility().at(sensor_index).at(current_frame_index);

            std::cout << "current_frame_index " << current_frame_index << std::endl;

            if (visibility) {

                cv::Mat_<float>  corr;
                cv::Scalar mean, stddev;
                cv::Mat covar_pts(2,2, CV_32FC1), covar_displacement(2,2, CV_32FC1);
                cv::Vec4f regression_line_displacement;

                mean = cv::Scalar::all(65535);
                stddev = cv::Scalar::all(65535);
                covar_pts = cv::Scalar::all(65535.0d);
                covar_displacement = cv::Scalar::all(65535.0d);

                const unsigned CLUSTER_SIZE = (unsigned) object->get_object_stencil_point_displacement().at
                        (sensor_index).at(current_frame_index).size();

                cv::Mat_<cv::Vec4f> samples(1, CLUSTER_SIZE, CV_32FC4);
                OBJECTS_MEAN_STDDEV final_values_for_this_object;
                /*-----------------------------------------------------------------------------*/

                execute(object, sensor_index, current_frame_index, CLUSTER_SIZE, mean, stddev, frame_dataprocessing_displacement, samples);

                /* ------------------------------------------------------------------------------ */

                final_values_for_this_object.mean_pts = cv::Point2f(mean(0), mean(1));
                final_values_for_this_object.mean_displacement = cv::Point2f(mean(2), mean(3));
                final_values_for_this_object.stddev_pts = cv::Point2f(stddev(0), stddev(1));
                final_values_for_this_object.stddev_displacement = cv::Point2f(stddev(2), stddev(3));

                // Covariance matrix of the displacement dataset
                if ( CLUSTER_SIZE != 0 ) {
                    Utils::getCovarMatrix(samples, covar_pts, covar_displacement, regression_line_displacement);
                }

                cv::Mat_<float> ellipse(3,1);
                // this is a structure of all sample semantics

                final_values_for_this_object.covar_pts = covar_pts;
                final_values_for_this_object.covar_displacement = covar_displacement;
                final_values_for_this_object.regression_line = regression_line_displacement;

                double chisquare_val = 2.4477;
                cv::Mat_<float> eigenvectors(2, 2);
                cv::Mat_<float> eigenvalues(1, 2);

                cv::eigen(covar_displacement, eigenvalues, eigenvectors);

                std::cout << "eigen " << eigenvectors << "\n" << eigenvalues << std::endl ;

                if (eigenvectors.data != NULL) {
                    //Calculate the angle between the largest eigenvector and the x-axis
                    double angle = atan2(eigenvectors(0, 1), eigenvectors(0, 0));

                    //Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
                    if (angle < 0)
                        angle += 6.28318530718;

                    //Conver to degrees instead of radians
                    angle = 180 * angle / 3.14159265359;

                    //Calculate the size of the minor and major axes
                    double halfmajoraxissize = chisquare_val * sqrt(eigenvalues(0));
                    double halfminoraxissize = chisquare_val * sqrt(eigenvalues(1));

                    //Return the oriented ellipse_shape_from_eigendata
                    //The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
                    ellipse << halfmajoraxissize, halfminoraxissize, angle;

                    std::cout << "ellipse" << ellipse;

                }

                final_values_for_this_object.ellipse = ellipse.clone();

                multiframe_object_results.push_back(final_values_for_this_object);
                multiframe_dataprocessing_displacement.push_back(frame_dataprocessing_displacement);



                //std::cout << "mean_displacement and mean stddev " + m_algoName << multiframe_object_results.at(current_frame_index).mean_displacement << multiframe_object_results.at(current_frame_index).stddev_displacement<< std::endl;

                if (current_frame_index > 0 && CLUSTER_SIZE != 0) {
                    assert(std::abs(multiframe_object_results.at(current_frame_index).mean_displacement.x)> 0);
                    //assert(std::abs(multiframe_object_results.at(current_frame_index).mean_displacement.y)> 0);
                }
            }

            else {

                std::cout << "not visible" << std::endl;
                //TODO change 0,0 to 65535,65535
                multiframe_object_results.push_back({cv::Point2f(0, 0),cv::Point2f(0,0), cv::Point2f(0, 0), cv::Point2f(0, 0)});
                multiframe_dataprocessing_displacement.push_back({std::make_pair(cv::Point2f(0, 0),cv::Point2f(0,0))});
            }

        }

        m_sensor_multiframe_dataprocessing_object_results.push_back(multiframe_object_results);
        m_sensor_multiframe_dataprocessing_stencil_point_displacement.push_back(multiframe_dataprocessing_displacement);

    }
}


void NoAlgorithm::execute(Objects *object, ushort sensor_index, ushort current_frame_index, unsigned CLUSTER_SIZE,
                          cv::Scalar &mean, cv::Scalar &stddev,  std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement, cv::Mat_<cv::Vec4f> &samples) {


    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

        cv::Point2f pts = object->get_object_stencil_point_displacement().at(sensor_index)
                .at(current_frame_index).at(cluster_index).first;
        cv::Point2f gt_displacement = object->get_object_stencil_point_displacement().at(
                sensor_index).at(current_frame_index).at(cluster_index).second;

        samples.at<cv::Vec4f>(0, cluster_index) = {pts.x, pts.y, gt_displacement.x, gt_displacement.y};

        frame_dataprocessing_displacement.push_back(std::make_pair(pts,gt_displacement));
    }

    cv::meanStdDev(samples, mean, stddev);


}

void SimpleAverage::execute(Objects *object, ushort sensor_index, ushort current_frame_index, unsigned CLUSTER_SIZE,
                            cv::Scalar &mean, cv::Scalar &stddev,  std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement, cv::Mat_<cv::Vec4f> &samples) {

    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

        cv::Point2f pts = object->get_object_stencil_point_displacement().at(sensor_index)
                .at(current_frame_index).at(cluster_index).first;
        cv::Point2f gt_displacement = object->get_object_stencil_point_displacement().at(
                sensor_index).at(current_frame_index).at(cluster_index).second;

        samples.at<cv::Vec4f>(0, cluster_index) = {pts.x, pts.y, gt_displacement.x, gt_displacement.y};

        frame_dataprocessing_displacement.push_back(std::make_pair(pts,gt_displacement));
    }

    cv::meanStdDev(samples, mean, stddev);

    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {
        frame_dataprocessing_displacement.at(cluster_index).second = cv::Point2f(mean(2), mean(3));
    }

}


void MovingAverage::execute(Objects *object, ushort sensor_index, ushort current_frame_index, unsigned CLUSTER_SIZE,
                                  cv::Scalar &mean, cv::Scalar &stddev,  std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement, cv::Mat_<cv::Vec4f> &samples) {

    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

        cv::Point2f pts = object->get_object_stencil_point_displacement().at(sensor_index)
                .at(current_frame_index).at(cluster_index).first;
        cv::Point2f gt_displacement = object->get_object_stencil_point_displacement().at(
                sensor_index).at(current_frame_index).at(cluster_index).second;

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

void VotedMean::execute(Objects *object, ushort sensor_index, ushort current_frame_index, unsigned CLUSTER_SIZE, cv::Scalar &mean, cv::Scalar &stddev,  std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement, cv::Mat_<cv::Vec4f> &samples) {



    

    std::vector<float> data_x, data_y;
    std::vector<float> data_x_pts, data_y_pts;

    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

        cv::Point2f pts = object->get_object_stencil_point_displacement().at(sensor_index)
                .at(current_frame_index).at(cluster_index).first;
        cv::Point2f gt_displacement = object->get_object_stencil_point_displacement().at(
                sensor_index).at(current_frame_index).at(cluster_index).second;

        samples.at<cv::Vec4f>(0, cluster_index) = {pts.x, pts.y, gt_displacement.x, gt_displacement.y};

        // get samples from template
        data_x.push_back(std::round(gt_displacement.x * 1000) / 1000);
        data_y.push_back(std::round(gt_displacement.y * 1000) / 1000);

        data_x_pts.push_back(pts.x);
        data_y_pts.push_back(pts.y);

        frame_dataprocessing_displacement.push_back(std::make_pair(pts,gt_displacement));

    }

    std::vector<cv::Mat> histogram_x, histogram_y;

    if (current_frame_index > 0 && CLUSTER_SIZE != 0 ) {
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

void RankedMean::execute(Objects *object, ushort sensor_index, ushort current_frame_index, unsigned CLUSTER_SIZE, cv::Scalar &mean, cv::Scalar &stddev,  std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement, cv::Mat_<cv::Vec4f> &samples) {


    



    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {

        cv::Point2f pts = object->get_object_stencil_point_displacement().at(sensor_index)
                .at(current_frame_index).at(cluster_index).first;
        cv::Point2f gt_displacement = object->get_object_stencil_point_displacement().at(
                sensor_index).at(current_frame_index).at(cluster_index).second;

        //samples.at<cv::Vec4f>(0, cluster_index) = {pts.x, pts.y, gt_displacement.x, gt_displacement.y};

        mean(0) += pts.x;
        mean(1) += pts.y;
        mean(2) += gt_displacement.x;
        mean(3) += gt_displacement.y;

        frame_dataprocessing_displacement.push_back(std::make_pair(pts, gt_displacement));

    }

    const unsigned CLUSTER_EDGE_SIZE = (unsigned) object->get_object_edge_point_displacement().at
            (sensor_index).at(current_frame_index).size();

    ushort WEIGHT = 100;

    for (unsigned cluster_edge_index = 0; cluster_edge_index < CLUSTER_EDGE_SIZE; cluster_edge_index++) {

        cv::Point2f pts_edge = object->get_object_edge_point_displacement().at(
                sensor_index).at(current_frame_index).at(cluster_edge_index).first;
        cv::Point2f gt_displacement = object->get_object_edge_point_displacement().at(
                sensor_index).at(current_frame_index).at(cluster_edge_index).second;

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


void SensorFusion::execute(Objects *object, ushort sensor_index, ushort current_frame_index, unsigned CLUSTER_SIZE,
                                 cv::Scalar &mean, cv::Scalar &stddev,  std::vector<std::pair<cv::Point2f, cv::Point2f> > &frame_dataprocessing_displacement, cv::Mat_<cv::Vec4f> &samples) {


    



    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {
        cv::Point2f pts = object->get_object_stencil_point_displacement().at(sensor_index)
                .at(current_frame_index).at(cluster_index).first;
        cv::Point2f gt_displacement = object->get_object_stencil_point_displacement().at(
                sensor_index).at(current_frame_index).at(cluster_index).second;

        samples.at<cv::Vec4f>(0, cluster_index) = {pts.x, pts.y, gt_displacement.x, gt_displacement.y};

        frame_dataprocessing_displacement.push_back(std::make_pair(pts,cv::Point2f(mean(2), mean(3))));
    }

    cv::meanStdDev(samples, mean, stddev);

    for (unsigned cluster_index = 0; cluster_index < CLUSTER_SIZE; cluster_index++) {
        frame_dataprocessing_displacement.at(cluster_index).second = cv::Point2f(mean(2), mean(3));
    }



}
