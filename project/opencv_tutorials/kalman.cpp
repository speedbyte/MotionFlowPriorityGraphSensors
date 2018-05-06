#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>


#include<ncurses.h>
using namespace cv;

static inline Point calcPoint(Point2f center, double R, double angle)
{
    return center + Point2f((float)cos(angle), (float)-sin(angle))*(float)R;
}

static void help()
{
    printf( "\nExamle of c calls to OpenCV's Kalman filter.\n"
"   Tracking of rotating point.\n"
"   Rotation speed is constant.\n"
"   Both state and measurements vectors are 1D (a point angle),\n"
"   Measurement is the real point angle + gaussian noise.\n"
"   The real and the estimated points are connected with yellow line segment,\n"
"   the real and the measured points are connected with red line segment.\n"
"   (if Kalman filter works correctly,\n"
"    the yellow segment should be shorter than the red one).\n"
            "\n"
"   Pressing any key (except ESC) will reset the tracking with a different speed.\n"
"   Pressing ESC will stop the program.\n"
            );
}


#define drawCross( center, color, d )                                 \
line( img, Point( center.x - d, center.y - d ), Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
line( img, Point( center.x + d, center.y - d ), Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )


int main2(int, char**)
{
    help();
    Mat img(500, 500, CV_8UC3);
    KalmanFilter KF(2, 1, 0);
    Mat state(2, 1, CV_32F); /* (phi, delta_phi) */
    Mat processNoise(2, 1, CV_32F);
    Mat measurement = Mat::zeros(1, 1, CV_32F);
    char code = (char)-1;

    for(;;)
    {
        randn( state, Scalar::all(0), Scalar::all(0.1) );
        cv::Mat_<float> val(2,2);
        val << 1, 1, 0, 1 ;
        KF.transitionMatrix = val;

        setIdentity(KF.measurementMatrix);
        setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
        setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
        setIdentity(KF.errorCovPost, Scalar::all(1));

        randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));

        for(;;)
        {
            Point2f center(img.cols*0.5f, img.rows*0.5f);
            float R = img.cols/3.f;
            double stateAngle = state.at<float>(0);
            Point statePt = calcPoint(center, R, stateAngle);

            Mat prediction = KF.predict();
            double predictAngle = prediction.at<float>(0);
            Point predictPt = calcPoint(center, R, predictAngle);

            randn( measurement, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));

            // generate measurement
            measurement += KF.measurementMatrix*state;

            double measAngle = measurement.at<float>(0);
            Point measPt = calcPoint(center, R, measAngle);


            img = Scalar::all(0);
            drawCross( statePt, Scalar(255,255,255), 3 );
            drawCross( measPt, Scalar(0,0,255), 3 );
            drawCross( predictPt, Scalar(0,255,0), 3 );
            line( img, statePt, measPt, Scalar(0,0,255), 3, CV_AA, 0 );
            line( img, statePt, predictPt, Scalar(0,255,255), 3, CV_AA, 0 );

            if(theRNG().uniform(0,4) != 0)
                KF.correct(measurement);

            randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
            state = KF.transitionMatrix*state + processNoise;

            imshow( "Kalman", img );
            code = (char)waitKey(100);

            if( code > 0 )
                break;
        }
        if( code == 27 || code == 'q' || code == 'Q' )
            break;
    }

    return 0;
}

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"


/*
 * Tell us where the pointer is.
 *
 * Modified from http://www.gusnet.cx/proj/miscunix/code/xquerypointer.c
 *
 * Compile with:
 *   cc -Wall -I/usr/X11R6/include -L/usr/X11R6/lib -lXm -o xquerypointer xquerypointer.c
 * or on solaris:
 *   cc -I/usr/openwin/include xquerypointer.c -L/usr/openwin/lib -lX11
 */

#include <stdio.h>
#include <stdlib.h>
#include <X11/Xlib.h>
#include <chrono>

int GetCursorPos(cv::Point *mousePos)
{
    // Gave a warning.
    // unsigned int snooze_time = 100000;
    Display *dpy;
    Window root;
    Window ret_root;
    Window ret_child;
    int root_x;
    int root_y;
    int win_x;
    int win_y;
    unsigned int mask;

    dpy = XOpenDisplay(NULL);
    root = XDefaultRootWindow(dpy);

    if(XQueryPointer(dpy, root, &ret_root, &ret_child, &root_x, &root_y,
                     &win_x, &win_y, &mask))
    {
        // original version
        //    printf("root loc: %4d,%4d win loc: %3d,%3d mask: 0x%08X\n",
        //           root_x, root_y, win_x, win_y, mask);

        // This returns in -geometry format
        // I added \n so it actually shows something so people who test it know it works.
        printf("+%d+%d\n", root_x, root_y);
    }
    else
    {
        // your script will break with this output, send it to stderr and let the script
        // return something sensible like +10+10
        printf("hmmmm, where is that sneaky pointer?\n");
    }

    *mousePos = cv::Point(root_x, root_y);
    return 0;
}


using namespace cv;
using namespace std;

cv::Point mousePos;

static void onMouse(int event,int x,int y,int,void*)
{
    //this function will be called every time you move your mouse over the image
    // the coordinates will be in x and y variables
    //Mat img2;img.copyTo(img2);
    //line(img2,Point(x,0),Point(x,img2.cols),Scalar(0,0,255),2);
    //line(img2,Point(0,y),Point(img2.rows,y),Scalar(0,0,255),2);
    //imshow("Image",img2);
    mousePos = {x,y};
}

int main( )
{

    KalmanFilter KF(4, 2, 0);  // dimension of the state vector, dimension in the measurement, dimension of the control vector, precision ( by default 32F )
    cv::namedWindow("mouse kalman", CV_WINDOW_AUTOSIZE);
    setMouseCallback("mouse kalman", onMouse);
    //getyx(WINDOW *win,int y,int x);
    //GetCursorPos(&mousePos);

// intialization of KF...
    cv::Mat_<float>  transferMatrix(4,4); // define the transfer matrix. output = transfer*input
    transferMatrix<< 1,0,1,0,  0,1,0,1, 0,0,1,0,   0,0,0,1;

    KF.transitionMatrix = transferMatrix;
    Mat_<float> measurement(2,1);

    measurement.setTo(Scalar(0)); // initial measurement is set to 0

    // initial prediction is known, but initial velocity is not known
    KF.statePre.at<float>(0) = mousePos.x;
    KF.statePre.at<float>(1) = mousePos.y;
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;

    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
    setIdentity(KF.measurementNoiseCov, Scalar::all(10));
    setIdentity(KF.errorCovPost, Scalar::all(.1));
// Image to show mouse tracking

    Mat img(600, 800, CV_8UC3);
    vector<Point> mouseVal, kalmanVal;

    mouseVal.clear();
    kalmanVal.clear();

    std::srand(unsigned(std::time(0)));

    while(1)
    {
        // First predict, to update the internal statePre variable
        Mat prediction = KF.predict();
        Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

        // Get mouse point
        //GetCursorPos(&mousePos);
        measurement(0) = mousePos.x;
        measurement(1) = mousePos.y;
        //setIdentity(KF.measurementNoiseCov, Scalar(100,200)); // equal uncertainity for both x and y position

        setIdentity(KF.measurementNoiseCov, Scalar::all(std::rand()%10));  // how uncertain are the meausurements

        // The update phase
        Mat estimated = KF.correct(measurement);

        Point statePt(estimated.at<float>(0),estimated.at<float>(1));
        Point measPt(measurement(0),measurement(1));
        // plot points
        imshow("mouse kalman", img);
        img = Scalar::all(0);

        mouseVal.push_back(measPt);
        kalmanVal.push_back(statePt);

        drawCross( statePt, Scalar(255,255,255), 5 );
        drawCross( measPt, Scalar(0,0,255), 5 );

        for (int i = 0; i < mouseVal.size()-1; i++)
            line(img, mouseVal[i], mouseVal[i+1], Scalar(255,255,0), 1);

        for (int i = 0; i < kalmanVal.size()-1; i++)
            line(img, kalmanVal[i], kalmanVal[i+1], Scalar(0,155,255), 1);

        waitKey(10);

    }

    return 0;
}
