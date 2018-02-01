
#include <vector>
#include <cmath>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>
#include <map>
#include <chrono>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <png++/png.hpp>
#include <gnuplot-iostream/gnuplot-iostream.h>

#include "kitti/log_colormap.h"
#include <kitti/mail.h>
#include <kitti/io_flow.h>
#include <vires/vires_common.h>

#include "datasets.h"
#include "GroundTruthFlow.h"
#include "kbhit.h"

#include <unordered_map>
#include <bits/unordered_map.h>

#include "ObjectTrajectory.h"
#include "Dataset.h"
#include "GroundTruthScene.h"
#include "ObjectFlow.h"


//Creating a movement path. The path is stored in a x and y vector

using namespace std::chrono;


void GroundTruthFlow::prepare_directories() {

    char char_dir_append[20];

    if (!m_dataset.getBasePath().compare(CPP_DATASET_PATH) || !m_dataset.getBasePath().compare(VIRES_DATASET_PATH) ) {

        std::cout << "Creating GT Flow directories" << std::endl;
        // create flow directories
        for (int i = 1; i < 10; ++i) {
            // delete ground truth image and ground truth flow directories
            sprintf(char_dir_append, "%02d", i);
            boost::filesystem::path path = m_dataset.getGroundTruthFlowPath().string() + "/flow_occ_" + char_dir_append;
            if (boost::filesystem::exists(path)) {
                system(("rm -rf " + path.string()).c_str());
            }
            boost::filesystem::create_directories(path);
        }
        std::cout << "Ending GT Flow directories" << std::endl;
    }
}


void GroundTruthFlow::generate_gt_scene_flow_vector() {

    prepare_directories();

    cv::Mat tempGroundTruthImage;
    tempGroundTruthImage.create(m_dataset.getFrameSize(), CV_8UC3);
    assert(tempGroundTruthImage.channels() == 3);


    std::map<std::string, double> time_map = {{"generate",0}, {"ground truth", 0}};

    auto tic = steady_clock::now();
    auto toc = steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();


    std::cout << "ground truth flow will be stored in " << m_dataset.getGroundTruthFlowPath().string() << std::endl;

    for ( int i = 0; i < m_list_objects.size(); i++ ) {
        m_list_objects.at(i).generate_base_flow_vector();
        // extend the flow vectors by skipping frames and then storing them as pngs
        m_list_objects.at(i).generate_extended_flow_vector();
    }

    char folder_name_flow[50];
    cv::FileStorage fs;
    fs.open(m_dataset.getGroundTruthFlowPath().string() + "/" + folder_name_flow + "/" + "gt_flow.yaml",
            cv::FileStorage::WRITE);
    std::vector<std::vector<std::pair<cv::Point2i, cv::Point2i> > > objects;



    for ( unsigned frame_skip = 1; frame_skip < m_list_objects.at(0).getFlowPoints().get().size() ;
          frame_skip++ ) {

        sprintf(folder_name_flow, "flow_occ_%02d", frame_skip);

        std::cout << "saving flow files for frame_skip " << frame_skip << std::endl;

        for (ushort frame_count = 0; frame_count < (m_list_objects.at(0).getFlowPoints().get().at
                (frame_skip-1)).size(); frame_count++) {

            char file_name_image[50];
            sprintf(file_name_image, "000%03d_10.png", frame_count);
            std::string temp_gt_flow_image_path = m_dataset.getGroundTruthFlowPath().string() + "/" + folder_name_flow + "/"
                                                  + file_name_image;

            fs << "frame_count" << frame_count;
            extrapolate_flowpoints(temp_gt_flow_image_path, frame_skip, frame_count, m_list_objects,
                                              m_dataset);
        }
        fs.release();
    }

    // plotVectorField (F_gt_write,m_base_directory_path_image_out.parent_path().string(),file_name);
    toc_all = steady_clock::now();
    time_map["ground truth"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth flow generation time - " << time_map["ground truth"]  << "ms" << std::endl;

}



void GroundTruthFlow::extrapolate_flowpoints(std::string temp_gt_flow_image_path, unsigned frame_skip, unsigned
frame_count,
                                        std::vector<Objects> list_objects, Dataset &dataset) {

    FlowImage F_gt_write(dataset.getFrameSize().width, dataset.getFrameSize().height);
    cv::Mat tempMatrix;
    tempMatrix.create(dataset.getFrameSize(),CV_32FC3);
    assert(tempMatrix.channels() == 3);
    //tempMatrix = cv::Scalar::all(0);
    for ( unsigned i = 0; i < list_objects.size(); i++ ) {

        // object shape
        int width = list_objects.at(i).getShapeImageData().get().cols;
        int height = list_objects.at(i).getShapeImageData().get().rows;

        // gt_displacement
        cv::Point2i gt_next_pts = list_objects.at(i).getFlowPoints().get().at(frame_skip-1).at(frame_count).first;
        cv::Point2f gt_displacement = list_objects.at(i).getFlowPoints().get().at(frame_skip-1).at(frame_count).second;

        cv::Mat roi;
        roi = tempMatrix.
                colRange(gt_next_pts.x, (gt_next_pts.x + width)).
                rowRange(gt_next_pts.y, (gt_next_pts.y + height));
        //bulk storage
        roi = cv::Scalar(gt_displacement.x, gt_displacement.y, 1.0f);

/*
        //cv::Vec3f *datagt_next_ptsr = tempMatrix.gt_next_ptsr<cv::Vec3f>(0); // pointer to the first channel of the first element in the
        // first row. The r, g b  value of single pixels are continous.
        float *array = (float *)malloc(3*sizeof(float)*dataset.getFrameSize().width*dataset.getFrameSize().height);
        cv::MatConstIterator_<cv::Vec3f> it = roi.begin<cv::Vec3f>();
        for (unsigned i = 0; it != roi.end<cv::Vec3f>(); it++ ) {
            for ( unsigned j = 0; j < 3; j++ ) {
                *(array + i ) = (*it)[j];
                i++;
            }
        }
        FlowImage temp = FlowImage(array, dataset.getFrameSize().width, dataset.getFrameSize().height );
        F_gt_write = temp;

 */
    }

    //Create png Matrix with 3 channels: x gt_displacement. y displacment and Validation bit
    for (int32_t row=0; row<dataset.getFrameSize().height; row++) { // rows
        for (int32_t column=0; column<dataset.getFrameSize().width; column++) {  // cols
            if (tempMatrix.at<cv::Vec3f>(row,column)[2] > 0.5 ) {
                F_gt_write.setFlowU(column,row,tempMatrix.at<cv::Vec3f>(row,column)[1]);
                F_gt_write.setFlowV(column,row,tempMatrix.at<cv::Vec3f>(row,column)[0]);
                F_gt_write.setValid(column,row,1.0f);
                //trajectory.store_in_yaml(fs, cv::Point2i(row, column), cv::Point2i(xValue, yValue) );
            }
        }
    }
    F_gt_write.write(temp_gt_flow_image_path);
}

void GroundTruthFlow::generatePixelRobustness() {

    calcCovarMatrix();

}

void GroundTruthFlow::generateVectorRobustness() {

}



void GroundTruthFlow::common(cv::Mat_<uchar> &samples_xy, std::vector<std::string> &list_gp_lines ) {

    float m,c;
    std::string coord1;
    std::string coord2;
    std::string gp_line;
    // XY, 2XY and 2X2Y all gives the same correlation
    cv::Mat_<float> covar, mean, corr;
    cv::Scalar mean_x, mean_y, stddev_x,stddev_y;

    cv::Vec4f line;
    cv::Mat mat_samples(1,samples_xy.cols,CV_32FC(2));


    std::cout << "\nsamples_xy\n" << samples_xy;
    cv::calcCovarMatrix( samples_xy, covar, mean, cv::COVAR_NORMAL|cv::COVAR_COLS|cv::COVAR_SCALE, CV_32FC1);

    cv::meanStdDev(samples_xy.row(0),mean_x,stddev_x);
    cv::meanStdDev(samples_xy.row(1),mean_y,stddev_y);

    assert(std::floor(mean(0)*100) == std::floor(mean_x(0)*100));
    assert(std::floor(mean(1)*100) == std::floor(mean_y(0)*100));

    cv::Mat_<float> stddev(2,2);
    stddev << stddev_x[0]*stddev_x[0], stddev_x[0]*stddev_y[0], stddev_x[0]*stddev_y[0], stddev_y[0]*stddev_y[0];
    corr = covar/stddev;

    std::cout << "\nMean\n" << mean << "\nCovar\n" << covar <<
              "\nstddev_x\n" << stddev_x << "\nstddev_y\n" << stddev_y <<
              "\ncorr\n" << corr << std::endl;


    for (unsigned i=0;i<samples_xy.cols;i++) {
        mat_samples.at<cv::Vec<float,2>>(0,i)[0] = samples_xy[0][i];
        mat_samples.at<cv::Vec<float,2>>(0,i)[1] = samples_xy[1][i];
    }

    cv::fitLine(mat_samples,line,CV_DIST_L2,0,0.01,0.01); // radius and angle from the origin - a kind of constraint
    m = line[1]/line[0];
    c = line[3] - line[2]*m;
    coord1 = "0," + std::to_string(c);
    coord2 = std::to_string(-c/m) + ",0";
    gp_line = "set arrow from " + coord1 + " to " + coord2 + " nohead lc rgb \'red\'\n";
    list_gp_lines.push_back(gp_line);
}


/**
 * Given any number of vectors, the function will compute
 * 1. The mean of the gaussian approximaiton to the distribution of the sample points.
 * 2. The covariance for the Guassian approximation to the distribution of the sample points.
 *
 */
void GroundTruthFlow::calcCovarMatrix() {

    for ( int i = 0; i < m_list_objects.size(); i++ ) {
    }

    std::vector<std::pair<double,double>> xypoints_1, xypoints_2, xypoints_3;

    cv::Mat_<uchar> samples_xy(2,9);
    std::vector<std::string> list_gp_lines;

    //------------------------------------------------------------------------

    samples_xy << 1,3,2,5,8,7,12,2,4,8,6,9,4,3,3,2,7,7;
    common(samples_xy, list_gp_lines);
    for ( unsigned i = 0; i<samples_xy.cols; i++) {
        xypoints_1.push_back(std::make_pair(samples_xy[0][i], samples_xy[1][i]));
    }

    //------------------------------------------------------------------------

    samples_xy.row(0) = 5*samples_xy.row(0);
    common(samples_xy, list_gp_lines);
    for ( unsigned i = 0; i<samples_xy.cols; i++) {
        xypoints_2.push_back(std::make_pair(samples_xy[0][i], samples_xy[1][i]));
    }

    //------------------------------------------------------------------------

    samples_xy.row(1) = 2*samples_xy.row(1);
    common(samples_xy, list_gp_lines);
    for ( unsigned i = 0; i<samples_xy.cols; i++) {
        xypoints_3.push_back(std::make_pair(samples_xy[0][i], samples_xy[1][i]));
    }


    //------------------------------------------------------------------------

    //------------------------------------------------------------------------

    //Plot
    Gnuplot gp;
    gp << "set xlabel 'x'\nset ylabel 'y'\n";
    gp << "set xrange[0:80]\n" << "set yrange[0:20]\n";
    //gp_line = "set arrow from 0,0 to $x1,$y2 nohead lc rgb \'red\'\n";
    std::cout << list_gp_lines[0];
    gp << list_gp_lines.at(0);
    gp << list_gp_lines.at(1);
    gp << list_gp_lines.at(2);
    gp << "plot '-' with lines title 'xy', '-' with lines title 'x_2,y', '-' with lines title 'x_2_y_2'\n";
    gp.send1d(xypoints_1);
    gp.send1d(xypoints_2);
    gp.send1d(xypoints_3);

    // Two matrices sample
    cv::Mat_<uchar> x_sample(1,9);  x_sample << 1,3,2,5,8,7,12,2,4;
    cv::Mat_<uchar> y_sample(1,9);  y_sample << 8,6,9,4,3,3,2,7,7;
    std::vector<cv::Mat> matgt_next_ptsr;
    matgt_next_ptsr.push_back(x_sample);
    matgt_next_ptsr.push_back(y_sample);
    //cv::calcCovarMatrix( &matgt_next_ptsr, 2, covar_x, mean_x, cv::COVAR_NORMAL|cv::COVAR_ROWS, CV_32FC1);

}


