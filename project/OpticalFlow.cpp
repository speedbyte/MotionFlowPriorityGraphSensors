/*
 *
 * Adjusted OpenCV Optical FLow Demo. Takes video as argument
 *
 */

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>

using namespace cv;
using namespace std;

Point2f point;
bool addRemovePt = false;

int main(int argc, char** argv) {
	VideoCapture cap;
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	Size subPixWinSize(10, 10), winSize(31, 31);

	const int MAX_COUNT = 500;
	bool needToInit = true;
	int i = 0;


	cv::CommandLineParser parser(argc, argv, "{@input||}{help h||}");
	string input = parser.get<string>("@input");

	if (input.size() == 1 && isdigit(input[0]))
		cap.open(input[0] - '0');
	else
		cap.open(input);

	if (!cap.isOpened()) {
		cout << "Could not initialize capturing...\n";
		return 0;
	}

	namedWindow("LK Demo", 1);

	Mat gray, prevGray, image, frame;
	vector<Point2f> points[2];

	for (;;) {
		cap >> frame;
		if (frame.empty())
			break;

		frame.copyTo(image);
		cvtColor(image, gray, COLOR_BGR2GRAY);


		if (i == 20 || needToInit) {
			// automatic initialization
			goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3,
					0, 0.04);
			cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1),
					termcrit);
			addRemovePt = false;
			i = 0;
		} else if (!points[0].empty()) {
			vector<uchar> status;
			vector<float> err;
			if (prevGray.empty())
				gray.copyTo(prevGray);
			calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status,
					err, winSize, 3, termcrit, 0, 0.001);
			size_t i, k;
			for (i = k = 0; i < points[1].size(); i++) {
				if (addRemovePt) {
					if (norm(point - points[1][i]) <= 5) {
						addRemovePt = false;
						continue;
					}
				}

				if (!status[i])
					continue;

				points[1][k++] = points[1][i];
				circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
			}
			points[1].resize(k);
		}

		if (addRemovePt && points[1].size() < (size_t) MAX_COUNT) {
			vector<Point2f> tmp;
			tmp.push_back(point);
			cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
			points[1].push_back(tmp[0]);
			addRemovePt = false;
		}

		needToInit = false;
		imshow("LK Demo", image);

		char c = (char) waitKey(30);
		if (c == 27)
			break;
		switch (c) {
		case 'r':
			needToInit = true;
			break;
		case 'c':
			points[0].clear();
			points[1].clear();
			break;

		}
		i++;

		std::swap(points[1], points[0]);
		cv::swap(prevGray, gray);
	}

	return 0;
}
