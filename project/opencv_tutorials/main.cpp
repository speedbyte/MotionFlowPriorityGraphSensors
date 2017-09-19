/*
 * main.c
 *
 *  Created on: 29.04.2010
 *      Author: antrit01
 */

/* --Sparse Optical Flow Demo Program--
 * by Anton Tratkovski 19.05.2010
 */

#include <string.h>
#include <time.h>

#include <stdlib.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <math.h>
//#include <cmath>

static const double pi = 3.14159265358979323846;

inline static double square(int a)
{
	return a * a;
}

/* This is just an inline that allocates images.  I did this to reduce clutter in the
 * actual computer vision algorithmic code.  Basically it allocates the requested image
 * unless that image is already non-NULL.  It always leaves a non-NULL image as-is even
 * if that image's size, depth, and/or channels are different than the request.
 */
inline static void allocateOnDemand ( IplImage **img, CvSize size, int depth, int channels )
{
	if ( *img != NULL )	return;

	*img = cvCreateImage( size, depth, channels );
	if ( *img == NULL )
	{
		fprintf(stderr, "Error: Couldn't allocate image.  Out of memory?\n");
		exit(-1);
	}
}

struct TwoFrames {
	IplImage *frame1;
	IplImage *frame2;
};

struct Pointer {
	CvPoint p1;
	CvPoint p2;
};

float LaengePtr (struct Pointer P){
	int x, y;
	x = (P.p1.x > P.p2.x)? (P.p1.x - P.p2.x) : (P.p2.x - P.p1.x);
	y = (P.p1.y > P.p2.y)? (P.p1.y - P.p2.y) : (P.p2.y - P.p1.y);
	return sqrt(square(x) + square(y));
}
int LaengePtrX (struct Pointer P){
	return (P.p1.x > P.p2.x)? (P.p1.x - P.p2.x) : (P.p2.x - P.p1.x);
}
int LaengePtrY (struct Pointer P){
	return (P.p1.y > P.p2.y)? (P.p1.y - P.p2.y) : (P.p2.y - P.p1.y);
}

void changeLaenge(struct Pointer *ptr, double mal){
	ptr->p1.x = ptr->p1.x * mal;
	ptr->p1.y = ptr->p1.y * mal;
	ptr->p2.x = ptr->p2.x * mal;
	ptr->p2.y = ptr->p2.y * mal;
}

int FlowPointer(IplImage *frame, struct Pointer ptr, CvSize frame_size, double angle2, int loop) {

	//Vareablen zu Textausgabe

	//changeLaenge( &ptr, 250/loop);

	CvFont Font_ = cvFont(0.5, 1);
	char str[256];
	int main_line_thickness = 1;
	CvScalar main_line_color = CV_RGB(0,255,0);
	CvPoint p3;
	p3.x = frame_size.width/2;
	p3.y = frame_size.height/2;
	CvPoint p4;
	int Dx, Dy;
	Dx = ptr.p2.x - p3.x;
	Dy = ptr.p2.y - p3.y;
	p4.x = ptr.p1.x - Dx;
	p4.y = ptr.p1.y - Dy;

	// Textausgabe der x y Werte
	sprintf(str, "X = %i ", (ptr.p1.x - ptr.p2.x));
	cvPutText(frame, str, cvPoint(frame_size.width-40, 10), &Font_, cvScalar(0, 0, 0, 0));

	sprintf(str, "Y = %i ", (ptr.p1.y - ptr.p2.y));
	cvPutText(frame, str, cvPoint(frame_size.width-40, 20), &Font_, cvScalar(0, 0, 0, 0));

	cvLine(frame, p3, p4 , main_line_color, main_line_thickness, CV_AA, 0);

	p3.x = (int) (p4.x + 9 * cos(angle2 - pi / 5 + pi));
	p3.y = (int) (p4.y + 9 * sin(angle2 - pi / 5 + pi));
	cvLine(frame, p4, p3, main_line_color, main_line_thickness, CV_AA, 0);

	p3.x = (int) (p4.x + 9 * cos(angle2 + pi / 5 + pi));
	p3.y = (int) (p4.y + 9 * sin(angle2 + pi / 5 + pi));
	cvLine(frame, p3, p4, main_line_color, main_line_thickness, CV_AA, 0);

	return 1;
}

struct Pointer FramesToPointerBM( struct TwoFrames tf, CvSize fs ){

	static IplImage *frame1 = NULL, *frame1_1C = NULL, *frame2_1C = NULL, *pyramid1 = NULL, *pyramid2 = NULL;
	static CvPoint point;

	allocateOnDemand( &frame1_1C, fs, IPL_DEPTH_8U, 1 ); 	//Alokieren des Bildes frame1_C1
	cvConvertImage(tf.frame1, frame1_1C, CV_CVTIMG_FLIP);	//�bertragen au das Bild des erste Frames

	allocateOnDemand( &frame1, fs, IPL_DEPTH_8U, 1 );
	cvConvertImage(tf.frame1, frame1, CV_CVTIMG_FLIP);

	allocateOnDemand( &frame2_1C, fs, IPL_DEPTH_8U, 1 );
	cvConvertImage(tf.frame2, frame2_1C, CV_CVTIMG_FLIP);

	//CvSize blockSice = cvSize(40,40);
	CvSize blockSize = cvSize(4,4);
	CvSize shiftSize = cvSize(1,1);
	CvSize maxRange = cvSize(3,3);

	pyramid1 = cvCreateImage(cvSize(fs.width/blockSize.width,fs.height/blockSize.height),IPL_DEPTH_32F,1);
	pyramid2 = cvCreateImage(cvSize(fs.width/blockSize.width,fs.height/blockSize.height),IPL_DEPTH_32F,1);

	cvCalcOpticalFlowBM( frame1_1C, frame2_1C, blockSize, shiftSize, maxRange, 0, pyramid1, pyramid2 );

	cvShowImage("Optical Flow from Cam 1",frame1_1C);
	cvShowImage("Optical Flow from Cam 2",frame2_1C);

	CvPoint d, b;
	b.x = 0;
	b.y = 0;
	d.x = 0;
	d.y = 0;

	for(int i=0;i<pyramid1->height;i+=3)
	{
		b.y = i*blockSize.height; //height
		for(int j=0;j<pyramid1->width;j+=3)
		{
			b.x = j*blockSize.width; //width
			d.x = b.x + (int)( (( (float*)(pyramid1->imageData + i*pyramid1->widthStep) )[j] )*blockSize.width);
			d.y = b.y + (int)((( ( (float*)(pyramid2->imageData + i*pyramid2->widthStep) )[j] ))*blockSize.height);
		}
	}

	struct Pointer pointer;
	pointer.p1 = b;
	pointer.p2 = d;
	return pointer;
}


struct Pointer FramesToPointerPyrLK( struct TwoFrames tf, CvSize fs ){

	const int Feat_Count = 300 ; // war zu Beginn auf 400 - Anzahl der Pfeile

	static IplImage *frame1 = NULL, *frame1_1C = NULL, *frame2_1C = NULL, *eig_image = NULL, *temp_image = NULL, *pyramid1 = NULL, *pyramid2 = NULL;
	static CvPoint point;

	allocateOnDemand( &frame1_1C, fs, IPL_DEPTH_8U, 1 ); 	//Alokieren des Bildes frame1_C1
	cvConvertImage(tf.frame1, frame1_1C, CV_CVTIMG_FLIP);	//�bertragen au das Bild des erste Frames

	allocateOnDemand( &frame1, fs, IPL_DEPTH_8U, 1 );
	cvConvertImage(tf.frame1, frame1, CV_CVTIMG_FLIP);

	allocateOnDemand( &frame2_1C, fs, IPL_DEPTH_8U, 1 );
	cvConvertImage(tf.frame2, frame2_1C, CV_CVTIMG_FLIP);

	allocateOnDemand( &eig_image, fs, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &temp_image, fs, IPL_DEPTH_32F, 1 );

	CvPoint2D32f frame1_features[Feat_Count];

	int number_of_features;
	int number_of_good_features;

	number_of_features = Feat_Count; // <- 400
	number_of_good_features = Feat_Count;

	cvGoodFeaturesToTrack(frame1_1C, eig_image, temp_image, frame1_features, &number_of_features, .01, .01, NULL, 3, 0.04, 0);
	int i;
	for (i = 0; i < Feat_Count; i++){
		cvCircle(tf.frame1,cvPoint(frame1_features[i].x,frame1_features[i].y),1,cvScalar(0,250,250,0));
	}


	CvPoint2D32f frame2_features[Feat_Count];

	char optical_flow_found_feature[Feat_Count];

	float optical_flow_feature_error[Feat_Count];

	CvSize optical_flow_window = cvSize(10,10);

	CvTermCriteria optical_flow_termination_criteria
			= cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 3 );

	allocateOnDemand( &pyramid1, fs, IPL_DEPTH_8U, 1 );
	allocateOnDemand( &pyramid2, fs, IPL_DEPTH_8U, 1 );

	cvCalcOpticalFlowPyrLK(frame1_1C, frame2_1C, pyramid1, pyramid2, frame1_features, frame2_features, number_of_features, optical_flow_window, 3, optical_flow_found_feature, optical_flow_feature_error, optical_flow_termination_criteria,0 );

	cvShowImage("Optical Flow from Cam 1",frame1_1C);
	cvShowImage("Optical Flow from Cam 2",frame2_1C);

	for( i = 0; i < number_of_features; i++)
	{
		printf("%i \t, %i \n", frame1_features[i].x ,frame2_features[i].x );
	}
	//cvWaitKey(0);


	CvPoint d, b;
	b.x = 0;
	b.y = 0;
	d.x = 0;
	d.y = 0;


	for( i = 0; i < number_of_features; i++)
	{
		/* If Pyramidal Lucas Kanade didn't really find the feature, skip it. */
		if (optical_flow_found_feature[i] == 0) {
			number_of_good_features--;
			continue;
		}


		CvPoint p,q;
		p.x = (int) frame1_features[i].x;
		p.y = (int) frame1_features[i].y;
		q.x = (int) frame2_features[i].x;
		q.y = (int) frame2_features[i].y;
		//point = p;

		// Main_line
		b.x = b.x + p.x;
		b.y = b.y + p.y;
		d.x = d.x + q.x;
		d.y = d.y + q.y;

	}

	// Kleine Nebenroutine (durchschnittliche linie ...)
	b.x = b.x / (number_of_good_features / 3);
	b.y = b.y / (number_of_good_features / 3);
	d.x = d.x / (number_of_good_features / 3);
	d.y = d.y / (number_of_good_features / 3);

	struct Pointer pointer;
	pointer.p1 = b;
	pointer.p2 = d;
	return pointer;
}

struct Pointer FramesToPointerHS( struct TwoFrames tf, CvSize fs ){

	static IplImage *frame1 = NULL, *frame1_1C = NULL, *frame2_1C = NULL, *eig_image = NULL, *temp_image = NULL, *pyramid1 = NULL, *pyramid2 = NULL;
	CvTermCriteria criteria;
	criteria.epsilon = 1;
	criteria.max_iter = 2;
	criteria.type = 1;

	allocateOnDemand( &pyramid1, fs, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &pyramid2, fs, IPL_DEPTH_32F, 1 );
	int usePrevious = 2;
	static float lambda = 0.01;

	cvCalcOpticalFlowHS(tf.frame1,tf.frame2,usePrevious,pyramid1,pyramid2,lambda,criteria);

	CvPoint d, b,p,q;
	b.x = 0;
	b.y = 0;
	d.x = 0;
	d.y = 0;
	p.x = 0;
	p.y = 0;
	q.x = 0;
	q.y = 0;

	cvShowImage("Optical Flow from Cam 2", pyramid1); // zeigt die ermittelten Bewegungsframes, in x- und y-Richtung
	cvShowImage("Optical Flow from Cam", pyramid2);

	//auswertung der Velx und Vely Bilder !!!

	for(int i=0;i<pyramid1->height;i+=3)
	{
		b.y = i; //height

		int j;
		for (j=0;j<pyramid1->width;j+=3)
		{
			b.x = j; //width
			d.x = b.x + (int)( ( (float*)(pyramid1->imageData + i*pyramid1->widthStep) )[j] );
			d.y = b.y + (int)( ( (float*)(pyramid2->imageData + i*pyramid2->widthStep) )[j] );
			p.x=(p.x + b.x);
			p.y=(p.y + b.y);
			q.x=(q.x + d.x);
			q.y=(q.y + d.y);

		}
	}

	struct Pointer pointer;
	pointer.p1 = p;
	pointer.p2 = q;
	return pointer;
}

//Fuktion die anhand der l�nge des Vektors eine Dauer in mS ausgibt um optimal den Flow zu erkennen
int lenghtToLooptime(int lenght){
	/*if (lenght < 10) return 500;
    if (20 > lenght > 10) return 250;
    else return 125; */
	if (lenght < 5) return 250;
	if (10 > lenght && lenght> 5) return 125;
	else return 60;
}

//Funktion zum Einstellen der Wartezeit bis zum Loopende
/*int looptime (int t_u, int t_s, int looptime){
     struct timeval tv;
     struct timezone tz;
     gettimeofday(&tv, &tz);
     t_s = tv.tv_sec - t_s;
     t_u = tv.tv_usec - t_u;
     int tsp = (t_s * 1000000) + t_u;
     if ((looptime - (tsp/1000)) < 1) return 1;
     else return (looptime - (tsp/1000));
}*/

struct Pointer CalcTriPtr (struct Pointer ptr1,struct Pointer ptr2,struct Pointer ptr3){
	struct Pointer ptr_x;
	ptr_x.p1.x= 0;
	ptr_x.p1.y= 0;
	ptr_x.p2.x= (LaengePtrX(ptr1)*3 + LaengePtrX(ptr2)*2 + LaengePtrX(ptr3))/6;
	ptr_x.p2.y= (LaengePtrY(ptr1)*3 + LaengePtrY(ptr2)*2 + LaengePtrY(ptr3))/6;
	return ptr_x;
}


int main(void)
{
	printf("Start \n");
	int p[3];
	p[0] = CV_IMWRITE_PXM_BINARY; //CV_IMWRITE_PNG_COMPRESSION; CV_IMWRITE_JPEG_QUALITY; CV_IMWRITE_PXM_BINARY
	p[1] = 0;
	p[2] = 0;

	int LOOPtime = 100;
	//double poineter_scale = 0.3;
	int loop = 1;
	//int wait_time = 1;
	CvFont Font_= cvFont(0.5,1); //Vareablen zur Textausgabe
	char str[256];
	char str2[256];

	CvCapture *input_video;
	input_video  = cvCaptureFromCAM(0);
	if (input_video == NULL)
	{
		/* Either the video didn't exist OR it uses a codec OpenCV
		 * doesn't support.
		 */
		fprintf(stderr, "Error: No Camera found.\n");
		return -1;
	}
	printf("Cam AN \n");

	/* Read the video's frame size out of the Cam */
	CvSize frame_size;
	frame_size.height =	(int) cvGetCaptureProperty( input_video, CV_CAP_PROP_FRAME_HEIGHT );
	frame_size.width =	(int) cvGetCaptureProperty( input_video, CV_CAP_PROP_FRAME_WIDTH );

	cvNamedWindow("Optical Flow from Cam 1", 0);
	cvNamedWindow("Optical Flow from Cam 2", 0);

	printf("Fenster AN \n");

	//zeiten fur die ausgabe
	int t_0 = 0, t_1;

	//Schreibt das Video mit
	char VideoPath[256] = "D:\\ORGA\\BachelorBetreuung\\Anton Tratkovski\\CV_HS_Auswertung\\images\\images\\Mitschnitt.mpeg";
	char BildPath[256] = "D:\\ORGA\\BachelorBetreuung\\Anton Tratkovski\\CV_HS_Auswertung\\images\\Bild.jpg";
	char TextPath[256] = "D:\\ORGA\\BachelorBetreuung\\Anton Tratkovski\\CV_HS_Auswertung\\images\\Werte.txt";
	CvVideoWriter * VW = cvCreateVideoWriter(VideoPath,CV_FOURCC('P','I','M','1'),25,frame_size);
	printf("Writer eingerichtet\n");

	//strukturen f�r die Frames
	struct TwoFrames Two_F;
	struct TwoFrames Two_F2;
	IplImage * RecordFrame;
	RecordFrame = cvCreateImage(frame_size, IPL_DEPTH_8U,3);
	//Two_F.frame1 =  cvCreateImage( frame_size, IPL_DEPTH_8U, 1 );
	//Two_F.frame2 =  cvCreateImage( frame_size, IPL_DEPTH_8U, 1 );

	Two_F2.frame1 =  cvCreateImage( frame_size, IPL_DEPTH_8U, 1 );
	Two_F2.frame2 =  cvCreateImage( frame_size, IPL_DEPTH_8U, 1 );
	allocateOnDemand( &Two_F2.frame1, frame_size, IPL_DEPTH_8U, 1 );
	allocateOnDemand( &Two_F2.frame2, frame_size, IPL_DEPTH_8U, 1 );

	IplImage *vel_x, *vel_y;

	struct Pointer ptr; //ptr_v, ptr_vv;

//	ptr_v.p1.x=ptr_v.p1.y=ptr_v.p2.x=ptr_v.p2.y=0;
//	ptr_vv.p1.x=ptr_vv.p1.y=ptr_vv.p2.x=ptr_vv.p2.y=0;

	int RecordFlag;
	FILE *datei;
	datei = fopen (TextPath, "w");
	fprintf(datei,"Loop;Zeit[ms];L�nge;L�ngeX;L�ngeY\n");

	//Zeitsteuerung �ber clock()
	clock_t start, start2, end;
	start2 = clock();

	int i = 0;
	char c = NULL;

	while(c != 'q'){

		printf("WHILE  ");

		start = clock(); // Z�hlt cpu-Ticks, ungefair 1 000 000 pro Sekunde

		Two_F.frame1 = cvQueryFrame( input_video ); //holt ein Bild von der Kamera
		cvSaveImage(BildPath ,Two_F.frame1, p); // speichert und l�d das bild wieder
		Two_F.frame1 = cvLoadImage( BildPath,0); // so verschlechtert sich die qualit�t

		//cvWaitKey(LOOPtime/2);

		Two_F.frame2 = cvQueryFrame( input_video ); //holt ein Bild von der Kamera
		cvSaveImage(BildPath ,Two_F.frame2, p); // speichert und l�d das bild wieder
		Two_F.frame2 = cvLoadImage( BildPath, 0); // so verschlechtert sich die qualit�t
		end = clock();
		sprintf(str2, "Cam: %i ", ((end - start)));
		start = clock();

		//		allocateOnDemand( &frame2_1C, fs, IPL_DEPTH_8U, 1 );
		//		cvConvertImage(tf.frame2, frame2_1C, CV_CVTIMG_FLIP);


		cvConvertImage(Two_F.frame1, Two_F2.frame1,0);
		cvConvertImage(Two_F.frame2, Two_F2.frame2,0);

//		cvShowImage("Optical Flow from Cam 1", Two_F2.frame1);
//		cvShowImage("Optical Flow from Cam 2", Two_F2.frame2);

		// diese Funktionen verwandeln zwei Bilder in einen 2D-Richtungsvektor
		ptr = FramesToPointerHS(Two_F2,frame_size);			//mit dem HS OF Allgoritm  "cvCalcOpticalFlowHS"
		//ptr = FramesToPointerPyrLK(Two_F2,frame_size);		//mit dem LK OF Allgoritm  "cvCalcOpticalFlowPyrLK"
		//ptr = FramesToPointerBM(Two_F2,frame_size);				//mit dem HS OF Allgoritm  "cvCalcOpticalFlowBM"



		double angle;		angle = atan2( (double) ptr.p1.y - ptr.p2.y, (double) ptr.p1.x - ptr.p2.x );
		//double hypotenuse;	hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x) );
		FlowPointer(Two_F.frame1,ptr,frame_size, angle,LOOPtime);

		end = clock();
		sprintf(str, "Alg: %i ", ((end - start)));
		cvPutText(Two_F.frame1, str, cvPoint(10, 10), &Font_, cvScalar(0, 0, 0, 0));
		cvPutText(Two_F.frame1, str2, cvPoint(10, 20), &Font_, cvScalar(0, 0, 0, 0));
		printf(str);
		printf("\tuSec \tX=%i, \tY=%i\n", (ptr.p1.x - ptr.p2.x), (ptr.p1.y - ptr.p2.y));
		cvShowImage("Optical Flow from Cam 1", Two_F.frame1);

		if (c == 'r')
			RecordFlag = 1;

		if (RecordFlag == 1){
			i++;
			cvConvertImage(Two_F.frame1,RecordFrame,CV_CVTIMG_FLIP);
			//cvWriteFrame(VW,RecordFrame);
			fprintf(datei,"%i;%i;%f;%i;%i\n",i,clock()-start2,LaengePtr(ptr),LaengePtrX(ptr),LaengePtrY(ptr));
		}

		c=cvWaitKey(10);
		//LOOPtime = lenghtToLooptime(LaengePtr(ptr));
	}


	cvReleaseCapture(&input_video);
	cvDestroyAllWindows();
	fclose (datei);
	cvReleaseVideoWriter(&VW);
	exit(0);

}



void LK(int argc, char *argv) {

	//Zeitsteuerung �ber clock()
	clock_t start, start2, end;
	start2 = clock();

	int i = 0;
	char c = NULL;

//    while(c != 'q') {
//    }

	//fprintf(datei,"Loop;Zeit[ms];Länge;LängeX;LängeY\n");

	//zeiten fur die ausgabe
	int t_0 = 0, t_1;

	cv::Point2f point;
	bool addRemovePt = false;

	cv::VideoCapture input_video;
	cv::Mat image;
	input_video.open("../../../video_dataset/megamind.avi");

	/* Read the video's frame size out of the Cam */
	CvSize frame_size;
	frame_size.height =	(int) input_video.get(CV_CAP_PROP_FRAME_HEIGHT );
	frame_size.width =	(int) input_video.get(CV_CAP_PROP_FRAME_WIDTH );

	cvNamedWindow("Optical Flow from Cam 1", 0);
	cvNamedWindow("Optical Flow from Cam 2", 0);

	printf("Fenster AN \n");

	//Schreibt das Video mit
	boost::filesystem::path VideoPath = "Mitschnitt.mpeg";
	boost::filesystem::path BildPath = "Bild.jpg";
	boost::filesystem::path TextPath = "Werte.txt";
	cv::VideoWriter video_out;
	video_out.open(VideoPath.string(),CV_FOURCC('P','I','M','1'),25,frame_size);
	printf("Writer eingerichtet\n");


	while ( cv::waitKey(1) != 'q' && cap.isOpened() ) {
		cap.grab();
		cap.retrieve(image);
	}
	cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
	cv::Size subPixWinSize(10, 10), winSize(31, 31);

	const int MAX_COUNT = 500;
	bool needToInit = true;
	int i = 0;



	for (;;) {
		cap >> frame;
		if (frame.empty())
			break;

		frame.copyTo(image);
		cvtColor(image, gray, cv::COLOR_BGR2GRAY);


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
		cv::imshow("LK Demo", image);

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
}
