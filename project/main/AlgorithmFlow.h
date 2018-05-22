//
// Created by veikas on 26.01.18.
//

#ifndef MAIN_FLOW_H
#define MAIN_FLOW_H

#include "datasets.h"
#include <iostream>
#include <boost/filesystem/path.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/features2d.hpp>
#include <opencv/cv.hpp>
#include "Dataset.h"
#include "PlotFlow.h"
#include "OpticalFlow.h"


class AlgorithmFlow : public OpticalFlow {

    // Each point on GroundTruthFlow is a vector of points in AlgorithmFlow. Hence both the base and fast movement
    // consists of an additional vector wrappper.



private:
    ALGO_TYPES mAlgo;

public:

    AlgorithmFlow( ALGO_TYPES algo, std::string opticalFlowName, std::vector<Objects*> &ptr_list_gt_objects, std::vector<Objects*> &ptr_list_simulated_base_objects, std::vector<Objects*> &ptr_list_simulated_objects, ushort stepSize ) : mAlgo(algo),
    OpticalFlow(opticalFlowName, ptr_list_gt_objects, ptr_list_simulated_base_objects, ptr_list_simulated_objects, stepSize) {

    }

    void prepare_directories(std::string noise, ushort fps, ushort stepSize) override ;

    void run_optical_flow_algorithm(FRAME_TYPES frame_types, std::string  noise, ushort fps);

    virtual void execute(const cv::Mat &prevGray, const cv::Mat &curGray, std::vector<cv::Point2f> &frame_prev_pts, std::vector<cv::Point2f> &frame_next_pts, bool &needToInit) {
        std::cout << "cannot be called" << std::endl;
    }
};


class Farneback : public AlgorithmFlow {

private:
    cv::Mat flowFrame;

public:

    Farneback(ALGO_TYPES algo, std::string opticalFlowName, std::vector<Objects*> &ptr_list_gt_objects, std::vector<Objects*> &ptr_list_simulated_base_objects, std::vector<Objects*> &ptr_list_simulated_objects, ushort stepSize ) : AlgorithmFlow( algo, opticalFlowName, ptr_list_gt_objects, ptr_list_simulated_base_objects, ptr_list_simulated_objects, stepSize ) {

        flowFrame.create(Dataset::getFrameSize(), CV_32FC2);
        flowFrame = cv::Scalar_<float>(0,0); //  the flow frame consists of next iterations
        assert(flowFrame.channels() == 2);

    }

    void execute(const cv::Mat &prevGray, const cv::Mat &curGray, std::vector<cv::Point2f> &frame_prev_pts, std::vector<cv::Point2f> &frame_next_pts, bool &needToInit) override {

        std::vector<uchar> status;
        // Initialize parameters for the optical generate_flow_frame algorithm
        float pyrScale = 0.5;
        int numLevels = 1;
        int windowSize = 5;
        int numIterations = 1;
        int neighborhoodSize = 2; // polyN
        float stdDeviation = 1.1; // polySigma

        std::vector<float> err;

        const int MAX_COUNT = 5000;

        // Calculate optical generate_flow_frame map using Farneback algorithm
        // Farnback returns displacement frame and LK returns points.
        cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, MAX_COUNT, 0.03);

        cv::calcOpticalFlowFarneback(prevGray, curGray, flowFrame, pyrScale, numLevels, windowSize,
                                     numIterations, neighborhoodSize, stdDeviation,
                                     cv::OPTFLOW_USE_INITIAL_FLOW);
        frame_prev_pts.clear();
        frame_next_pts.clear();
        for (int32_t row = 0; row < Dataset::getFrameSize().height; row += mStepSize) { // rows
            for (int32_t col = 0; col < Dataset::getFrameSize().width; col += 1) {  // cols

                cv::Point2f algo_displacement ( flowFrame.at<cv::Point2f>(row, col).x, flowFrame.at<cv::Point2f>(row, col).y );

                if (( cvFloor(std::abs(algo_displacement.x)) == 0 && cvFloor(std::abs(algo_displacement.y)) == 0 )) {
                    continue;
                }

                frame_next_pts.push_back(cv::Point2f(col, row));
                frame_prev_pts.push_back(cv::Point2f((col - algo_displacement.x), (row - algo_displacement.y)));

                status.push_back(1);
            }
        }
    }

};

class LukasKanade : public AlgorithmFlow {


public:

    LukasKanade(ALGO_TYPES algo, std::string opticalFlowName, std::vector<Objects*> &ptr_list_gt_objects, std::vector<Objects*> &ptr_list_simulated_base_objects, std::vector<Objects*> &ptr_list_simulated_objects, ushort stepSize ) : AlgorithmFlow( algo, opticalFlowName, ptr_list_gt_objects, ptr_list_simulated_base_objects, ptr_list_simulated_objects, stepSize ) {

    }

    void execute(const cv::Mat &prevGray, const cv::Mat &curGray, std::vector<cv::Point2f> &frame_prev_pts, std::vector<cv::Point2f> &frame_next_pts, bool &needToInit) override {

        std::vector<uchar> status;
        // Initialize parameters for the optical generate_flow_frame algorithm
        int numLevels = 1;

        std::vector<float> err;
        const int MAX_COUNT = 5000;
        cv::Size winSize(5, 5);
        cv::Size subPixWinSize(3, 3);

        if ( !needToInit ) {

            cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, MAX_COUNT, 0.03);
            cv::calcOpticalFlowPyrLK(prevGray, curGray, frame_prev_pts, frame_next_pts, status,
                                     err, winSize, numLevels, termcrit, 0, 0.001);
        }

        else  {
            // automatic initialization
            cv::goodFeaturesToTrack(curGray, frame_next_pts, MAX_COUNT, 0.01, 10, cv::Mat(), 3, false, 0.04);
            // Refining the location of the feature points
            assert(frame_next_pts.size() <= MAX_COUNT );
            std::cout << "before subpixel size " << frame_next_pts.size() << std::endl;
            cv::TermCriteria termcrit_subpixel(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, MAX_COUNT, 0.03);
            cv::cornerSubPix(curGray, frame_next_pts, subPixWinSize, cv::Size(-1, -1), termcrit_subpixel);
            std::cout << "after subpixel size " << frame_next_pts.size() << std::endl;
            needToInit = false;
        }

        printf("old frame_next_pts size is %ld and new frame_next_pts size is %ld\n", frame_prev_pts.size(), frame_next_pts.size());


    }


};


#endif //MAIN_FLOW_H
