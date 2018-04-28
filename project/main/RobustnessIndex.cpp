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


void RobustnessIndex::make_video_from_png(const std::string &videoOrdner) {

    cv::VideoWriter video_write;
    cv::Mat temp_image;

    boost::filesystem::directory_iterator end_iter;

    boost::filesystem::path dir_path = videoOrdner;

    std::cout << dir_path.string() << std::endl;
    assert(boost::filesystem::exists(dir_path) != 0);

    std::string file_name, path;
    boost::filesystem::path temp;
    bool video_writer_init = false;

    for (boost::filesystem::directory_iterator dir_iter(dir_path); dir_iter != end_iter; ++dir_iter) {
        if (boost::filesystem::is_regular_file(dir_iter->status())) {
            std::string extension = boost::filesystem::extension(*dir_iter);
            if (extension == ".png") {
                std::cout << *dir_iter << std::endl;
                temp = *dir_iter;
                temp_image = cv::imread(temp.string(), cv::IMREAD_COLOR);
                if (video_writer_init == false) {
                    if (!video_write.open((dir_path.string() + "/movement_video.avi"), CV_FOURCC('D', 'I', 'V', 'X'),
                            5.0,
                            cv::Size(temp_image.cols, temp_image.rows), true)) {
                        std::cerr << "failed to initialise the video write" << std::endl;
                        throw;
                    }
                    if (!video_write.isOpened()) {
                        std::cerr << "Could not open video" << std::endl;
                    }

                    video_writer_init = true;
                }
                /*cv::namedWindow("video", CV_WINDOW_AUTOSIZE);
                cv::imshow("video", temp_image);
                cv::waitKey(1000);*/
                video_write.write(temp_image);
            } else {
                std::cout << "ignoring extension : " << extension << " path " << *dir_iter << std::endl;
            }
        }
        cv::destroyAllWindows();
    }

    video_write.release();
}

void PixelRobustness::generatePixelRobustness(const OpticalFlow &opticalFlow, const OpticalFlow &opticalFlow_base_algo) {

    auto position = opticalFlow.getResultOrdner().find('/');
    std::string suffix = opticalFlow.getResultOrdner().replace(position, 1, "_");
    unsigned COUNT;
    if ( suffix == "_ground_truth") {
        COUNT = 1;
    }
    else {
        COUNT = DATA_PROCESSING_COUNT;
    }

    // shape of algorithhm, with shape of ground truth
    for ( unsigned data_processing_index = 0; data_processing_index < COUNT; data_processing_index++ ) {

        for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

            std::cout << "generating pixel robustness in RobustnessIndex.cpp for " << suffix << " " << frame_skip
                    << " for dataprocessing " << data_processing_index << std::endl;

            std::vector<cv::Point2f>  xsamples, ysamples;
            std::vector<cv::Point2f>  xsamples_dimension, ysamples_displacement;

            unsigned long FRAME_COUNT = opticalFlow.getShapePoints().at(data_processing_index).at(frame_skip - 1).size();

            for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

                unsigned long POINTS = opticalFlow.getShapePoints().at(data_processing_index).at(frame_skip - 1).at(frame_count).size();
                for (unsigned points = 0; points < POINTS; points++) {

                    std::pair<cv::Point2i, cv::Point2f> shapepoints = opticalFlow.getShapePoints().at(data_processing_index).at(
                            frame_skip - 1).at(frame_count).at
                            (points);

                    xsamples.push_back(shapepoints.first);
                    ysamples.push_back(shapepoints.second);

                }

                unsigned long POINTS_DIM = opticalFlow.getMeanDisplacementPoints().at(data_processing_index).at(frame_skip - 1).at(frame_count).size();
                    for (unsigned points = 0; points < POINTS_DIM; points++) {

                    std::pair<cv::Point2i, cv::Point2f> displacementPoints = opticalFlow.getMeanDisplacementPoints().at(
                            data_processing_index).at(frame_skip - 1).at(frame_count).at(points);

                    xsamples_dimension.push_back(displacementPoints.first);
                    ysamples_displacement.push_back(displacementPoints.second);
                }
            }

            // Send to plotter

            if ( suffix == "_ground_truth") {
                m_fs << (std::string("pixel_density") + suffix) << "[";
            }
            else {
                m_fs << (std::string("pixel_density") + std::string("frame_skip") + std::to_string(frame_skip) +
                         std::string("_dataprocessing_") + std::to_string(data_processing_index) + suffix) << "[";
            }

            for (unsigned i = 0; i < xsamples.size(); i++) {
                m_fs << "{:" << "good_pixels" << xsamples[i] << "total_pixels" << ysamples[i] << "}";
            }
            m_fs << "]";

            // Send to plotter
            m_fs << (std::string("obj_displacement") + std::string("frame_skip") + std::to_string(frame_skip) +
                     std::string("_dataprocessing_") + std::to_string(data_processing_index) + suffix) << "[";

            for (unsigned i = 0; i < xsamples_dimension.size(); i++) {
                m_fs << "{:" << "objDim" << xsamples_dimension[i] << "objDisp" << ysamples_displacement[i] << "}";
            }
            m_fs << "]";



            if ( data_processing_index < 0 ) {

                std::map<std::pair<float, float>, int> scenario_displacement_occurence;

                scenario_displacement_occurence = opticalFlow.getScenarioDisplacementOccurence().at(frame_skip - 1);

                m_fs << (std::string("scenario_displacement_occurence") + std::string("frame_skip") + std::to_string(frame_skip) +
                         std::string("_dataprocessing_") + std::to_string(data_processing_index) + suffix) << "[";

                for ( auto it=scenario_displacement_occurence.begin(); it != scenario_displacement_occurence.end(); it++) {

                    if ( it->second > 1 ) {
                        std::cout << cv::Point2f(it->first.first, it->first.second) << " " << it->second << std::endl;
                        m_fs << "{:" << "x" << it->first.first << "y" << it->first.second << "occurence" << it->second << "}";
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
        COUNT = DATA_PROCESSING_COUNT;
    }

    for ( unsigned data_processing_index = 0; data_processing_index < COUNT; data_processing_index++ ) {

        for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

            std::cout << "generating vector robustness in RobustnessIndex.cpp for " << suffix << " " << frame_skip
                    << " for dataprocessing " << data_processing_index << std::endl;

            std::vector<float> xsamples,ysamples;
            std::vector<float> xsamples_line,ysamples_line;

            unsigned long FRAME_COUNT = opticalFlow.getCollisionPoints().at(data_processing_index).at(frame_skip - 1).size();

            for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

                ushort m_valid_collision_points = 0;
                ushort m_invalid_collision_points = 0;

                unsigned long POINTS = opticalFlow.getCollisionPoints().at(data_processing_index).at(frame_skip-1).at(frame_count).size();

                for ( unsigned points = 0 ; points < POINTS; points++ ) {

                    //    std::vector<std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > > >
                    cv::Point2f collisionpoints = opticalFlow.getCollisionPoints().at(data_processing_index).at(frame_skip-1).at(frame_count).at(points);

                    xsamples.push_back(collisionpoints.x);
                    ysamples.push_back(collisionpoints.y);
                    m_valid_collision_points++;

                    cv::Point2f lineangles = opticalFlow.getLineAngles().at(data_processing_index).at(frame_skip-1).at(frame_count).at
                            (points);

                    xsamples_line.push_back(std::tanh(lineangles.x)*180/CV_PI);
                    ysamples_line.push_back(std::tanh(lineangles.y)*180/CV_PI);

                    /*
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
                    */
                }

                if (m_valid_collision_points == 0 ) {

                    xsamples.push_back(std::numeric_limits<float>::infinity());
                    ysamples.push_back(std::numeric_limits<float>::infinity());

                    xsamples_line.push_back(std::numeric_limits<float>::infinity());
                    ysamples_line.push_back(std::numeric_limits<float>::infinity());

                    std::cout << "number of invalid collision points in frame " << frame_count << " are " <<
                              m_invalid_collision_points << std::endl;
                    //xsamples.push_back(0);
                    //ysamples.push_back(0);

                }
            }

            std::string plot_least_square_line_list;
            std::vector<std::pair<double, double>> xypoints_collision;
            std::vector<std::pair<double, double>> xypoints_lineangles;

            cv::Mat_<float> samples_xy_collision(2, xsamples.size());


            for ( auto i = 0; i < xsamples.size(); i++) {
                samples_xy_collision(0,i) = xsamples.at(i);
                samples_xy_collision(1,i) = ysamples.at(i);
            }

            if ( suffix == "_ground_truth") {
                m_fs << (std::string("collision_points") + suffix ) << "[";

            } else {
                m_fs << (std::string("collision_points") + std::string("frame_skip") + std::to_string(frame_skip) + std::string("_dataprocessing_") + std::to_string(data_processing_index) + suffix ) << "[";

            }

            for (unsigned i = 0; i < samples_xy_collision.cols; i++) {
                xypoints_collision.push_back(std::make_pair(samples_xy_collision[0][i], samples_xy_collision[1][i]));
                m_fs << "{:" << "x" <<  samples_xy_collision[0][i] << "y" << samples_xy_collision[1][i] << "}";
            }
            m_fs << "]";


            m_fs << (std::string("line_angles") + std::string("frame_skip") + std::to_string(frame_skip) + std::string("_dataprocessing_") + std::to_string(data_processing_index) + suffix ) << "[";

            for (unsigned i = 0; i < xsamples_line.size(); i++) {
                xypoints_lineangles.push_back(std::make_pair(xsamples_line.at(i), ysamples_line.at(i)));
                m_fs << "{:" << "obj1" <<  xypoints_lineangles.at(i).first << "obj2" << xypoints_lineangles.at(i).second << "}";
            }
            m_fs << "]";

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
