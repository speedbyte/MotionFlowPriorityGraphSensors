//
// Created by veikas on 02.02.18.
//

#include <opencv/cv.hpp>
#include <iostream>
#include <map>
#include "RobustnessIndex.h"
#include "Dataset.h"
#include "datasets.h"
#include "OpticalFlow.h"
#include "Utils.h"


void PixelRobustness::generatePixelRobustness(const OpticalFlow &opticalFlow, const OpticalFlow &opticalFlow_base_algo) {

    auto position = opticalFlow.getResultOrdner().find('/');
    std::string suffix = opticalFlow.getResultOrdner().replace(position, 1, "_");
    unsigned COUNT;
    if ( suffix == "_ground_truth") {
        COUNT = 1;
    }
    else {
        COUNT = DATAFILTER_COUNT;
    }

    // shape of algorithhm, with shape of ground truth
    for ( unsigned datafilter_index = 0; datafilter_index < COUNT; datafilter_index++ ) {

        for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

            std::cout << "generating pixel robustness in RobustnessIndex.cpp for " << suffix << " " << sensor_index
                    << " for datafilter " << datafilter_index << std::endl;

            std::vector<cv::Point2f>  xsamples, ysamples;
            std::vector<cv::Point2f>  xsamples_dimension, ysamples_displacement;

            unsigned long FRAME_COUNT = opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).size();

            // Send to plotter

            if ( suffix == "_ground_truth") {
                m_fs << (std::string("pixel_density")  + suffix + std::string("_sensor_index_") + std::to_string(sensor_index)) << "[";
            }
            else {
                m_fs << (std::string("pixel_density") +
                         std::string("_datafilter_") + std::to_string(datafilter_index) + suffix + std::string("sensor_index_") + std::to_string(sensor_index)) << "[";
            }

            for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

                unsigned long POINTS = opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(frame_count).size();

                for (unsigned points = 0; points < POINTS; points++) {

                    std::pair<cv::Point2f, cv::Point2f> displacementPoints = std::make_pair(
                    opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(frame_count).at(points).object_dimension, opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(frame_count).at(points).mean_object_displacement);

                    //Utils::getMahalanobisDistance()

                    m_fs << "{:" << "frame_count" <<
                         opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(frame_count).at(points).frame_count
                            << "obj_index" <<
                         opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(frame_count).at(points).obj_index
                            << "good_pixels" <<
                         opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(frame_count).at(points).goodPixels
                            << "visible_pixels" <<
                         opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(frame_count).at(points).visiblePixels
                            << "total_pixels" <<
                         opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(frame_count).at(points).object_dimension.x * opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(frame_count).at(points).object_dimension.y
                            << "stddev" <<
                         (opticalFlow.get_sensor_multiframe_evaluation_data().at(datafilter_index).at(sensor_index).at(frame_count).at(points).stddev_object_displacement)

                         << "}";
                    //xsamples.push_back(shapepoints.first);
                    //ysamples.push_back(shapepoints.second);
                    xsamples_dimension.push_back(displacementPoints.first);
                    ysamples_displacement.push_back(displacementPoints.second);

                }

            }

            m_fs << "]";

            // Send to plotter
            if ( datafilter_index < 0 ) {

                if (suffix == "_ground_truth") {
                    m_fs << (std::string("obj_displacement") + suffix) + std::string("_sensor_index_") +
                            std::to_string(sensor_index) << "[";
                } else {
                    m_fs << (std::string("obj_displacement") +
                             std::string("_datafilter_") + std::to_string(datafilter_index) + suffix +
                             std::string("sensor_index_") + std::to_string(sensor_index)) << "[";
                }

                for (unsigned i = 0; i < xsamples_dimension.size(); i++) {
                    m_fs << "{:" << "objDim" << xsamples_dimension[i] << "objDisp" << ysamples_displacement[i] << "}";
                }
                m_fs << "]";


                std::map<std::pair<float, float>, int> scenario_displacement_occurence;

                scenario_displacement_occurence = opticalFlow.getScenarioDisplacementOccurence().at(sensor_index);

                m_fs << (std::string("scenario_displacement_occurence") + std::string("sensor_index_") +
                         std::to_string(sensor_index) +
                         std::string("_datafilter_") + std::to_string(datafilter_index) + suffix) << "[";

                for (auto it = scenario_displacement_occurence.begin();
                     it != scenario_displacement_occurence.end(); it++) {

                    if (it->second > 1) {
                        std::cout << cv::Point2f(it->first.first, it->first.second) << " " << it->second << std::endl;
                        m_fs << "{:" << "x" << it->first.first << "y" << it->first.second << "occurence" << it->second
                             << "}";
                    }
                }

                m_fs << "]";

            }
        }
    }
}


void VectorRobustness::generateVectorRobustness(const OpticalFlow &opticalFlow, const OpticalFlow &opticalFlow_base_algo) {

    auto position = opticalFlow.getResultOrdner().find('/');
    std::string suffix = opticalFlow.getResultOrdner().replace(position, 1, "_");
    unsigned COUNT;
    if ( suffix == "_ground_truth") {
        COUNT = 1;
    }
    else {
        COUNT = DATAFILTER_COUNT;
    }

    for ( unsigned datafilter_index = 0; datafilter_index < COUNT; datafilter_index++ ) {

        for (unsigned sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {

            std::cout << "generating vector robustness in RobustnessIndex.cpp for " << suffix << " " << sensor_index
                    << " for datafilter " << datafilter_index << std::endl;

            std::vector<cv::Point2f>  xsamples, ysamples;
            std::vector<float> xsamples_line,ysamples_line;


            unsigned long FRAME_COUNT = opticalFlow.getCollisionPoints().at(datafilter_index).at(sensor_index).size();

            for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

                unsigned long POINTS = opticalFlow.getCollisionPoints().at(datafilter_index).at(sensor_index).at(frame_count).size();

                for ( unsigned points = 0 ; points < POINTS; points++ ) {

                    std::pair<cv::Point2i, cv::Point2f> collisionpoints = opticalFlow.getCollisionPoints().at(datafilter_index).at(sensor_index).at(frame_count).at(points);

                    xsamples.push_back(collisionpoints.first);
                    ysamples.push_back(collisionpoints.second);

                    /*
                    cv::Point2f lineangles = opticalFlow.getLineAngles().at(datafilter_index).at(sensor_index).at(frame_count).at(points);

                    xsamples_line.push_back(std::tanh(lineangles.x)*180/CV_PI);
                    ysamples_line.push_back(std::tanh(lineangles.y)*180/CV_PI);
                     */

                }
            }

            // Send to plotter

            std::string plot_least_square_line_list;
            std::vector<std::pair<double, double>> xypoints_lineangles;

            if ( suffix == "_ground_truth") {
                m_fs << (std::string("collision_points") + suffix + std::string("_sensor_index_") + std::to_string(sensor_index)) << "[";

            } else {
                m_fs << (std::string("collision_points") +  std::string("_datafilter_") + std::to_string(datafilter_index) + suffix + std::string("sensor_index_") + std::to_string(sensor_index)) << "[";

            }

            for (unsigned i = 0; i < xsamples.size(); i++) {
                m_fs << "{:" << "frame_count" << xsamples[i] << "collision_points" << ysamples[i] << "}";

            }
            m_fs << "]";


/*
            if ( suffix == "_ground_truth") {
                m_fs << (std::string("line_angles") + suffix + std::string("_sensor_index_") + std::to_string(sensor_index)) << "[";

            } else {
                m_fs << (std::string("line_angles") +  std::string("_datafilter_") + std::to_string(datafilter_index) + suffix + std::string("sensor_index_") + std::to_string(sensor_index)) << "[";

            }

            for (unsigned i = 0; i < xsamples_line.size(); i++) {
                xypoints_lineangles.push_back(std::make_pair(xsamples_line.at(i), ysamples_line.at(i)));
                m_fs << "{:" << "obj1" <<  xypoints_lineangles.at(i).first << "obj2" << xypoints_lineangles.at(i).second << "}";
            }
            m_fs << "]";
*/
            cv::Mat_<float> samples_xy_collision(2, xsamples.size());


            for ( auto i = 0; i < xsamples.size(); i++) {
                samples_xy_collision(0,i) = ysamples.at(i).x;
                samples_xy_collision(1,i) = ysamples.at(i).y;
            }


            // Linear least square
            fitLineForCollisionPoints(samples_xy_collision, plot_least_square_line_list);

        }
    }
}


void VectorRobustness::fitLineForCollisionPoints(const cv::Mat_<float> &samples_xy, std::string &plot_least_square_line_list) {

    float m, c;
    std::string coord1;
    std::string coord2;
    std::string gp_line;
    // XY, 2XY and 2X2Y all gives the same correlation
    cv::Mat_<float> covar, mean, corr;
    cv::Scalar mean_x, mean_y, stddev_x, stddev_y;

    cv::Vec4f line;
    cv::Mat mat_samples(1, samples_xy.cols, CV_32FC(2));

    std::cout << "\nsamples_xy\n" << samples_xy;

    if ( !samples_xy.empty() ) {

        cv::calcCovarMatrix(samples_xy, covar, mean, cv::COVAR_NORMAL | cv::COVAR_COLS | cv::COVAR_SCALE, CV_32FC1);

        cv::meanStdDev(samples_xy.row(0), mean_x, stddev_x);
        cv::meanStdDev(samples_xy.row(1), mean_y, stddev_y);

        //assert(std::floor(mean(0) * 100) == std::floor(mean_x(0) * 100));
        //assert(std::floor(mean(1) * 100) == std::floor(mean_y(0) * 100));

        cv::Mat_<float> stddev(2, 2);
        stddev << stddev_x[0] * stddev_x[0], stddev_x[0] * stddev_y[0], stddev_x[0] * stddev_y[0], stddev_y[0] *
                                                                                                   stddev_y[0];
        corr = covar / stddev;

        std::cout << "\nMean\n" << mean << "\nCovar\n" << covar <<
                  "\nstddev_x\n" << stddev_x << "\nstddev_y\n" << stddev_y <<
                  "\ncorr\n" << corr << std::endl;


        for (unsigned i = 0; i < samples_xy.cols; i++) {
            mat_samples.at<cv::Vec<float, 2>>(0, i)[0] = samples_xy[0][i];
            mat_samples.at<cv::Vec<float, 2>>(0, i)[1] = samples_xy[1][i];
        }

        cv::fitLine(mat_samples, line, CV_DIST_L2, 0, 0.01, 0.01); // radius and angle from the origin - a kind of
        // constraint
        m = line[1] / line[0];
        c = line[3] - line[2] * m;
        coord1 = "0," + std::to_string(c);
        coord2 = std::to_string((2000 - c ) / m) + ",375";
        gp_line = "set arrow from " + coord1 + " to " + coord2 + " nohead lc rgb \'red\'\n";
        plot_least_square_line_list = gp_line;

    }
}
