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



void RobustnessIndex::compareFlowData(const std::string &resultOrdner) {


    cv::Mat temp_image;
    boost::filesystem::directory_iterator end_iter;

    boost::filesystem::path dir_path = Dataset::getResultPath();
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

void PixelRobustness::generatePixelRobustness(const std::string &resultOrdner) {

}

void VectorRobustness::generateVectorRobustness(const OpticalFlow &opticalFlow) {

    generateFrameVectorSignature(opticalFlow);


}


/**
 * Given any number of vectors, the function will compute
 * 1. The mean of the gaussian approximaiton to the distribution of the sample points.
 * 2. The covariance for the Guassian approximation to the distribution of the sample points.
 *
 */
void VectorRobustness::generateFrameVectorSignature(const OpticalFlow &opticalFlow) {


    m_fs << "APPLICATION" << opticalFlow.getResultOrdner();

    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {


        m_fs << "FRAME_SKIP" << (int)frame_skip;

        std::vector<float> xsamples,ysamples;

        unsigned long FRAME_COUNT = opticalFlow.getCollisionPoints().at(frame_skip - 1).size();

        for (unsigned frame_count = 1; frame_count < FRAME_COUNT; frame_count++) {

            ushort m_valid_collision_points = 0;
            ushort m_invalid_collision_points = 0;

            unsigned long POINTS = opticalFlow.getCollisionPoints().at(frame_skip-1).at(frame_count).size();
            for ( unsigned points = 0 ; points < POINTS; points++ ) {

                cv::Point2f collisionpoints = opticalFlow.getCollisionPoints().at(frame_skip-1).at(frame_count).at
                        (points);

                xsamples.push_back(collisionpoints.x);
                ysamples.push_back(collisionpoints.y);

                if ( ( collisionpoints.x ) > 0 &&
                     ( collisionpoints.y ) > 0 &&
                     ( collisionpoints.x ) < Dataset::getFrameSize().width  &&
                     ( collisionpoints.y ) < Dataset::getFrameSize().height
                        ) {
                    m_valid_collision_points++;
                }
                else {
                    m_invalid_collision_points++;
                }
            }

            if (m_valid_collision_points == 0 ) {
                std::cout << "number of invalid collision points in frame " << frame_count << " are " <<
                        m_invalid_collision_points << std::endl;
                //xsamples.push_back(0);
                //ysamples.push_back(0);

            }
        }

        std::vector<std::string> list_gp_lines;
        std::vector<std::pair<double, double>> xypoints_collision;

        cv::Mat_<float> samples_xy_collision(2, xsamples.size());


        for ( auto i = 0; i < xsamples.size(); i++) {
            samples_xy_collision(0,i) = xsamples.at(i)/10;
            samples_xy_collision(1,i) = ysamples.at(i)/10;
        }

        m_fs << "collision_points" << "[";
        for (unsigned i = 0; i < samples_xy_collision.cols; i++) {
            xypoints_collision.push_back(std::make_pair(samples_xy_collision[0][i], samples_xy_collision[1][i]));
            m_fs << "{:" << "x" <<  samples_xy_collision[0][i] << "y" << samples_xy_collision[1][i] << "}";
        }
        m_fs << "]";


        // Linear least square
        fitLineForCollisionPoints(samples_xy_collision, list_gp_lines);

        //Plot
        Gnuplot gp;
        gp << "set xlabel 'x'\nset ylabel 'y'\n";
        //gp << "set xrange[" + std::to_string(0) + ":" + std::to_string(50) + "]\n" << "set yrange[" + std::to_string(0) + ":" + std::to_string(50)  + "]\n";
        gp << list_gp_lines.at(0);
        gp << "set title \"" + opticalFlow.getGeneratePath() + " with frameskips = " + std::to_string(frame_skip) + "\"\n";
        //gp << "plot '-' with lines title " + std::string("'collision ") + std::to_string(xypoints_collision.size()) + "'\n";
        //gp.send1d(xypoints_collision);
        //std::cout << m_valid_collision_points << "for frameskip " << frame_skip << std::endl;
    }

}



void VectorRobustness::fitLineForCollisionPoints(const cv::Mat_<float> &samples_xy, std::vector<std::string> &list_gp_lines) {

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
    coord2 = std::to_string((2000 - c ) / m) + ",375";
    gp_line = "set arrow from " + coord1 + " to " + coord2 + " nohead lc rgb \'red\'\n";
    list_gp_lines.push_back(gp_line);
}
