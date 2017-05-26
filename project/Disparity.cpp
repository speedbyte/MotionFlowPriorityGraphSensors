/*
 * Disparity.cpp
 *
 *  Created on: May 24, 2017
 *      Author: marci
 */

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
using namespace cv;

using namespace std;

Mat colorDisparityMap(Mat *);

/*
 * calculate disparity map
 * Usage ./Disparity left_img right_img
 */

int main(int argc, char* argv[]) {

	Mat img1, img2, g1, g2;
	Mat disp, cDisp;

	string left = argv[1];
	string right = argv[2];

	img1 = imread(left);
	img2 = imread(right);

	cv::namedWindow("Example1", cv::WINDOW_AUTOSIZE);
	imshow("Example", img1);
	cv::waitKey(0);
	cv::destroyWindow("Example1");

	cv::namedWindow("Example2", cv::WINDOW_AUTOSIZE);
	imshow("Example2", img2);
	cv::waitKey(0);
	cv::destroyWindow("Example2");

	cvtColor(img1, g1, CV_BGR2GRAY);
	cvtColor(img2, g2, CV_BGR2GRAY);

	//creates disparity map
	//@param numdisparity
	//@param blocksize
	//wide camera setting or close objects -> increase numdisparity

	cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(128, 11);

	sbm->setPreFilterCap(31);
	sbm->setPreFilterSize(5);
	sbm->setTextureThreshold(500);
	sbm->setSpeckleWindowSize(0);
	sbm->setSpeckleRange(8);
	sbm->setMinDisparity(0);
	sbm->setUniquenessRatio(0);
	sbm->setDisp12MaxDiff(1);

	sbm->compute(g1, g2, disp);
	normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);

	cv::namedWindow("Example3", cv::WINDOW_AUTOSIZE);
	imshow("Example3", disp);
	cv::waitKey(0);
	cv::destroyWindow("Example3");

	cDisp = colorDisparityMap( &disp);

	cv::namedWindow("Example4", cv::WINDOW_AUTOSIZE);
	imshow("Example4", cDisp);
	cv::waitKey(0);
	cv::destroyWindow("Example4");

}


//colors the disparity map
Mat colorDisparityMap(Mat *disp) {


	Mat cDisp;
	Mat coloredDisp;
	double min;
	double max;

	minMaxIdx(*disp, &min, &max);

	convertScaleAbs(*disp, cDisp, 255 / (max - min));
	applyColorMap(cDisp, coloredDisp, 2);

	return coloredDisp;

}

