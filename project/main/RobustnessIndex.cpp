//
// Created by veikas on 02.02.18.
//

#include <opencv/cv.hpp>
#include <iostream>
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

void PixelRobustness::generatePixelRobustness(const OpticalFlow &opticalFlow_gt, const OpticalFlow &opticalFlow_base_algo) {

    auto position = opticalFlow_gt.getResultOrdner().find('/');
    std::string suffix = opticalFlow_gt.getResultOrdner().replace(position, 1, "_");
    unsigned COUNT;
    if ( suffix == "_generated") {
        COUNT = 1;
    }
    else {
        COUNT = 4;
    }

    // shape of algorithhm, with shape of ground truth
    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

        for ( unsigned post_processing_index = 0; post_processing_index < COUNT; post_processing_index++ ) {

            std::vector<float> xsamples, ysamples;

            unsigned long FRAME_COUNT = opticalFlow_gt.getShapePoints().at(frame_skip - 1).at(post_processing_index).size();

            for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

                unsigned long POINTS = opticalFlow_gt.getShapePoints().at(frame_skip - 1).at(post_processing_index).at(frame_count).size();
                for (unsigned points = 0; points < POINTS; points++) {

                    cv::Point2f shapepoints = opticalFlow_gt.getShapePoints().at(frame_skip - 1).at(post_processing_index).at(frame_count).at
                            (points);

                    xsamples.push_back(shapepoints.x);
                    ysamples.push_back(shapepoints.y);

                }
            }

            std::string plot_least_square_line_list;
            std::vector<std::pair<double, double>> xypoints_shape;

            cv::Mat_<float> samples_xy_shape(2, xsamples.size());


            // Calculate Jaccard Index
            for (auto i = 0; i < xsamples.size(); i++) {
                samples_xy_shape(0, i) = i + 1; //xsamples.at(i);
                samples_xy_shape(1, i) = xsamples.at(i) / (xsamples.at(i) + ysamples.at(i));
            }

            m_fs << (std::string("shape_points") + std::string("frame_skip") + std::to_string(frame_skip) +
                     std::string("_postprocessing_") + std::to_string(post_processing_index) + suffix) << "[";

            for (unsigned i = 0; i < samples_xy_shape.cols; i++) {
                xypoints_shape.push_back(std::make_pair(samples_xy_shape[0][i], samples_xy_shape[1][i]));
                m_fs << "{:" << "x" << samples_xy_shape[0][i] << "y" << samples_xy_shape[1][i] << "}";
            }
            m_fs << "]";
        }
    }

}


void VectorRobustness::generateVectorRobustness(const OpticalFlow &opticalFlow_gt, const OpticalFlow &opticalFlow_base_algo) {

    auto position = opticalFlow_gt.getResultOrdner().find('/');
    std::string suffix = opticalFlow_gt.getResultOrdner().replace(position, 1, "_");
    unsigned COUNT;
    if ( suffix == "_generated") {
        COUNT = 1;
    }
    else {
        COUNT = 4;
    }

    for (unsigned frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++) {

        for ( unsigned post_processing_index = 0; post_processing_index < COUNT; post_processing_index++ ) {
            std::vector<float> xsamples,ysamples;

            unsigned long FRAME_COUNT = opticalFlow_gt.getCollisionPoints().at(frame_skip - 1).at(post_processing_index).size();

            for (unsigned frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

                ushort m_valid_collision_points = 0;
                ushort m_invalid_collision_points = 0;

                unsigned long POINTS = opticalFlow_gt.getCollisionPoints().at(frame_skip-1).at(post_processing_index).at(frame_count).size();
                for ( unsigned points = 0 ; points < POINTS; points++ ) {

                    cv::Point2f collisionpoints = opticalFlow_gt.getCollisionPoints().at(frame_skip-1).at(post_processing_index).at(frame_count).at
                            (points);

                    xsamples.push_back(collisionpoints.x);
                    ysamples.push_back(collisionpoints.y);
                    m_valid_collision_points++;
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
                    std::cout << "number of invalid collision points in frame " << frame_count << " are " <<
                              m_invalid_collision_points << std::endl;
                    //xsamples.push_back(0);
                    //ysamples.push_back(0);

                }
            }

            std::string plot_least_square_line_list;
            std::vector<std::pair<double, double>> xypoints_collision;

            cv::Mat_<float> samples_xy_collision(2, xsamples.size());


            for ( auto i = 0; i < xsamples.size(); i++) {
                samples_xy_collision(0,i) = xsamples.at(i);
                samples_xy_collision(1,i) = ysamples.at(i);
            }

            m_fs << (std::string("collision_points") + std::string("frame_skip") + std::to_string(frame_skip) + std::string("_postprocessing_") + std::to_string(post_processing_index) + suffix ) << "[";

            for (unsigned i = 0; i < samples_xy_collision.cols; i++) {
                xypoints_collision.push_back(std::make_pair(samples_xy_collision[0][i], samples_xy_collision[1][i]));
                m_fs << "{:" << "x" <<  samples_xy_collision[0][i] << "y" << samples_xy_collision[1][i] << "}";
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
