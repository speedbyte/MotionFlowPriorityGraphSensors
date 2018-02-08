//
// Created by veikas on 02.02.18.
//

#include <opencv/cv.hpp>
#include <iostream>
#include "RobustnessIndex.h"
#include "Dataset.h"
#include "datasets.h"
#include "OpticalFlow.h"
#include <gnuplot-iostream/gnuplot-iostream.h>


void PixelRobustness::generatePixelRobustness(const std::string &resultOrdner) {


    cv::Mat temp_image;
    boost::filesystem::directory_iterator end_iter;

    boost::filesystem::path dir_path = Dataset::getResultTrajectoryPath();
    dir_path += "/";
    dir_path += resultOrdner;
    dir_path += "/flow_occ_01";

    std::cout << dir_path.string() << std::endl;
    assert(boost::filesystem::exists(dir_path) != 0);

    std::string file_name, path;
    boost::filesystem::path temp;



    for (boost::filesystem::directory_iterator dir_iter(dir_path); dir_iter != end_iter; ++dir_iter) {
        if (boost::filesystem::is_regular_file(dir_iter->status())) {

            std::string extension = boost::filesystem::extension(*dir_iter);
            if (extension == ".png") {
                std::cout << *dir_iter << std::endl;
                temp = *dir_iter;
                temp_image = cv::imread(temp.string(), cv::IMREAD_COLOR);
                cv::imshow("video", temp_image);
                cv::waitKey(1000);

            } else {

                std::cout << "ignoring extension : " << extension << " path " << *dir_iter << std::endl;

            }
        }
        cv::destroyAllWindows();
    }
    //calcCovarMatrix();
}

void VectorRobustness::generateVectorRobustness(const OpticalFlow &opticalFlow) {

    calcCovarMatrix(opticalFlow);

}


/**
 * Given any number of vectors, the function will compute
 * 1. The mean of the gaussian approximaiton to the distribution of the sample points.
 * 2. The covariance for the Guassian approximation to the distribution of the sample points.
 *
 */
void VectorRobustness::calcCovarMatrix(const OpticalFlow &opticalFlow) {


    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {
        ushort m_valid_collision_points = 0;
        ushort m_invalid_collision_points = 0;
        std::vector<float> xsamples,ysamples;
        unsigned long FRAME_COUNT = opticalFlow.getCollisionPoints().at(frame_skip - 1).size();

        for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

            for ( unsigned points = 0 ; points < opticalFlow.getCollisionPoints().at(frame_skip-1).at(frame_count)
                                                         .size();
                  points++ ) {

                cv::Point2f collisionpoints = opticalFlow.getCollisionPoints().at(frame_skip-1).at(frame_count).at
                        (points);
                if ( ( collisionpoints.x ) > 0 &&
                     ( collisionpoints.y ) > 0 &&
                     ( collisionpoints.x ) < Dataset::getFrameSize().width  &&
                     ( collisionpoints.y ) < Dataset::getFrameSize().height
                        ) {
                    xsamples.push_back(collisionpoints.x);
                    ysamples.push_back(collisionpoints.y);
                    m_valid_collision_points++;
                }
                else {
                    m_invalid_collision_points++;
                }
            }

            if (m_valid_collision_points == 0 ) {
                std::cout << "number of invalid collision points in frame " << frame_count << " are " <<
                        m_invalid_collision_points << std::endl;
                xsamples.push_back(0);
                ysamples.push_back(0);

            }
        }


        std::vector<std::string> list_gp_lines;
        std::vector<std::pair<double, double>> xypoints_1, xypoints_2, xypoints_3, xypoints_collision;

        ushort size_collision = xsamples.size();
        cv::Mat_<float> samples_xy_collision(2, size_collision);


        for ( auto i = 0; i < size_collision; i++) {
            samples_xy_collision(0,i) = xsamples.at(i);
            samples_xy_collision(1,i) = ysamples.at(i);
        }

/*    for ( auto t : ysamples ) {
        samples_xy_collision.push_back(t);
    }
*/
        fitLineForCollisionPoints(samples_xy_collision, list_gp_lines);
        for (unsigned i = 0; i < samples_xy_collision.cols; i++) {
            xypoints_collision.push_back(std::make_pair(samples_xy_collision[0][i], samples_xy_collision[1][i]));
        }

        //Plot
        Gnuplot gp;
        gp << "set xlabel 'x'\nset ylabel 'y'\n";
        gp << "set xrange[0:1242]\n" << "set yrange[0:375]\n";
        std::cout << list_gp_lines[0];
        gp << list_gp_lines.at(0);
        gp << "plot '-' with points title " + std::string("'collision ") + std::to_string(m_valid_collision_points)+
                "'\n";
        gp.send1d(xypoints_collision);
    }
}



void VectorRobustness::fitLineForCollisionPoints(cv::Mat_<float> &samples_xy, std::vector<std::string> &list_gp_lines) {

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
    coord2 = std::to_string((375 - c ) / m) + ",375";
    //gp_line = "set arrow from 0,0 to $x1,$y2 nohead lc rgb \'red\'\n";
    gp_line = "set arrow from " + coord1 + " to " + coord2 + " nohead lc rgb \'red\'\n";
    list_gp_lines.push_back(gp_line);
}
