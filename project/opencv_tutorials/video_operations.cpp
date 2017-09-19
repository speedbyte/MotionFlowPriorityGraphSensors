

#include <opencv2/videoio.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <iostream>
#include <opencv/cv.hpp>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>


/* --Sparse Optical Flow Demo Program--
 * by Anton Tratkovski 19.05.2010
 */

static const double pi = 3.14159265358979323846;

struct TwoFrames {
    cv::Mat frame1;
    cv::Mat frame2;
};

struct Points {
    cv::Point p1;
    cv::Point p2;
};

double Laengepts (struct Points P){
    int x, y;
    x = (P.p1.x > P.p2.x)? (P.p1.x - P.p2.x) : (P.p2.x - P.p1.x);
    y = (P.p1.y > P.p2.y)? (P.p1.y - P.p2.y) : (P.p2.y - P.p1.y);
    return std::sqrt(std::pow(x,2) + std::pow(y,2));
}

int LaengeptsX (struct Points P){
    return (P.p1.x > P.p2.x)? (P.p1.x - P.p2.x) : (P.p2.x - P.p1.x);
}
int LaengeptsY (struct Points P){
    return (P.p1.y > P.p2.y)? (P.p1.y - P.p2.y) : (P.p2.y - P.p1.y);
}

void changeLaenge(struct Points *pts, double mal){
    pts->p1.x = (int)(pts->p1.x * mal);
    pts->p1.y = (int)(pts->p1.y * mal);
    pts->p2.x = (int)(pts->p2.x * mal);
    pts->p2.y = (int)(pts->p2.y * mal);
}


int FlowPoints(cv::Mat frame, struct Points pts, cv::Size frame_size, double angle2, int loop) {

    //Vareablen zu Textausgabe

    //changeLaenge( &pts, 250/loop);

    CvFont Font_ = cvFont(0.5, 1);
    char str[256];
    int main_line_thickness = 1;
    cv::Scalar main_line_color(0,255,0);
    cv::Point p3;
    p3.x = frame_size.width/2;
    p3.y = frame_size.height/2;
    cv::Point p4;
    int Dx, Dy;
    Dx = pts.p2.x - p3.x;
    Dy = pts.p2.y - p3.y;
    p4.x = pts.p1.x - Dx;
    p4.y = pts.p1.y - Dy;

    // Textausgabe der x y Werte
    sprintf(str, "X = %i ", (pts.p1.x - pts.p2.x));
    cv::putText(frame, str, cvPoint(frame_size.width-40, 10), Font_.font_face, 1, cvScalar(0, 0, 0, 0));

    sprintf(str, "Y = %i ", (pts.p1.y - pts.p2.y));
    cv::putText(frame, str, cvPoint(frame_size.width-40, 20), Font_.font_face, 1, cvScalar(0, 0, 0, 0));

    cv::line(frame, p3, p4 , main_line_color, main_line_thickness, CV_AA, 0);

    p3.x = (int) (p4.x + 9 * cos(angle2 - pi / 5 + pi));
    p3.y = (int) (p4.y + 9 * sin(angle2 - pi / 5 + pi));
    cv::line(frame, p4, p3, main_line_color, main_line_thickness, CV_AA, 0);

    p3.x = (int) (p4.x + 9 * cos(angle2 + pi / 5 + pi));
    p3.y = (int) (p4.y + 9 * sin(angle2 + pi / 5 + pi));
    cv::line(frame, p3, p4, main_line_color, main_line_thickness, CV_AA, 0);

    return 1;
}


struct Points FramesToPointerBM( struct TwoFrames tf, CvSize fs ){

    static cv::Mat frame1, frame1_1C, frame2_1C, pyramid1, pyramid2;
    static CvPoint point;

    frame1_1C.create( fs, CV_8UC1 ); 	//Alokieren des Bildes frame1_C1
    cv::cvtColor(tf.frame1, frame1_1C, CV_CVTIMG_FLIP);	//�bertragen au das Bild des erste Frames

    frame1.create( fs, CV_8UC1 );
    cv::cvtColor(tf.frame1, frame1, CV_CVTIMG_FLIP);

    frame1.create( fs, CV_8UC1 );
    cv::cvtColor(tf.frame2, frame2_1C, CV_CVTIMG_FLIP);

    //cv::Size blockSice = cv::Size(40,40);
    cv::Size blockSize = cv::Size(4,4);
    cv::Size shiftSize = cv::Size(1,1);
    cv::Size maxRange = cv::Size(3,3);

    pyramid1.create(cv::Size(fs.width/blockSize.width,fs.height/blockSize.height),CV_32FC1);
    pyramid2.create(cv::Size(fs.width/blockSize.width,fs.height/blockSize.height),CV_32FC1);

    //cv::calcOpticalFlowBM( frame1_1C, frame2_1C, blockSize, shiftSize, maxRange, 0, pyramid1, pyramid2 );

    cv::imshow("Optical Flow from Cam 1",frame1_1C);
    cv::imshow("Optical Flow from Cam 2",frame2_1C);

    CvPoint d, b;
    b.x = 0;
    b.y = 0;
    d.x = 0;
    d.y = 0;

    for(int i=0;i<pyramid1.rows;i+=3)
    {
        b.y = i*blockSize.height; //height
        for(int j=0;j<pyramid1.cols;j+=3)
        {
            b.x = j*blockSize.width; //width
            d.x = b.x + (int)( (( (float*)(pyramid1.data + i*pyramid1.step) )[j] )*blockSize.width);
            d.y = b.y + (int)((( ( (float*)(pyramid2.data + i*pyramid2.step) )[j] ))*blockSize.height);
        }
    }

    struct Points pointer;
    pointer.p1 = b;
    pointer.p2 = d;
    return pointer;
}

struct Points FramesToPointerPyrLK( struct TwoFrames tf, cv::Size fs ){

    const int Feat_Count = 300 ; // war zu Beginn auf 400 - Anzahl der Pfeile

    static cv::Mat frame1, frame1_1C, frame2_1C, eig_image, temp_image, pyramid1, pyramid2;
    static cv::Point point;

    frame1_1C.create( fs, CV_8UC1 ); 	//Alokieren des Bildes frame1_C1
    cv::cvtColor(tf.frame1, frame1_1C, CV_CVTIMG_FLIP);	//�bertragen au das Bild des erste Frames

    frame1.create(  fs, CV_8UC1 );
    cv::cvtColor(tf.frame1, frame1, CV_CVTIMG_FLIP);

    frame2_1C.create(  fs, CV_8UC1 );
    cv::cvtColor(tf.frame2, frame2_1C, CV_CVTIMG_FLIP);

    eig_image.create(  fs, CV_32FC1);
    temp_image.create(  fs, CV_32FC1);

    std::vector<cv::Point2f> frame1_features(Feat_Count);

    int number_of_features;
    int number_of_good_features;

    number_of_features = Feat_Count; // <- 400
    number_of_good_features = Feat_Count;

    /**
    void cv::goodFeaturesToTrack(
            cv::InputArray  image,                         // Input, CV_8UC1 or CV_32FC1
            cv::OutputArray corners,                       // Output vector of corners - either Vector cv::Point2f or
                                                                cv::Mat(x,2)
            int             maxCorners,                    // Keep this many corners
            double          qualityLevel,                  // (fraction) rel to best
            double          minDistance,                   // Discard corner this close
            cv::InputArray  mask              = noArray(), // Ignore corners where mask=0
            int             blockSize         = 3,         // Neighborhood used
            bool            useHarrisDetector = false,     // false='Shi Tomasi metric'
            double          k                 = 0.04       // Used for Harris metric
    );

    CVAPI(void)  cvGoodFeaturesToTrack( const CvArr* image, CvArr* eig_image,
                                        CvArr* temp_image, CvPoint2D32f* corners,
                                        int* corner_count, double  quality_level,
                                        double  min_distance,
                                        const CvArr* mask CV_DEFAULT(NULL),
                                        int block_size CV_DEFAULT(3),
                                        int use_harris CV_DEFAULT(0),
                                        double k CV_DEFAULT(0.04) );
    */

    cv::goodFeaturesToTrack(frame1_1C, frame1_features, number_of_features, .01, .01, cv::noArray(), 3, 0, 0.04);

    int i;
    for (i = 0; i < Feat_Count; i++){
        cv::circle(tf.frame1,cv::Point2f(frame1_features[i].x,frame1_features[i].y),1,cvScalar(0,250,250,0));
    }


    std::vector<cv::Point2f> frame2_features(Feat_Count);

    std::vector<bool> optical_flow_found_feature(Feat_Count);

    std::vector<float> optical_flow_feature_error(Feat_Count);

    cv::Size optical_flow_window = cv::Size(10,10);

    CvTermCriteria optical_flow_termination_criteria
            = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 3 );

    pyramid1.create(fs, CV_8UC1);
    pyramid2.create(fs, CV_8UC1);

/**
/* It is Lucas & Kanade method, modified to use pyramids.
   Also it does several iterations to get optical flow for
   every point at every pyramid level.
   Calculates optical flow between two images for certain set of points (i.e.
   it is a "sparse" optical flow, which is opposite to the previous 3 methods)
    CVAPI(void)  cvCalcOpticalFlowPyrLK( const CvArr*  prev, const CvArr*  curr,
                                         CvArr*  prev_pyr, CvArr*  curr_pyr,
                                         const CvPoint2D32f* prev_features,
                                         CvPoint2D32f* curr_features,
                                         int       count,
                                         CvSize    win_size,
                                         int       level,
                                         char*     status,
                                         float*    track_error,
                                         CvTermCriteria criteria,
                                         int       flags );

 void cv::calcOpticalFlowPyrLK(
  cv::InputArray       prevImg,            // Prior image (t-1), CV_8UC1
  cv::InputArray       nextImg,            // Next image (t), CV_8UC1
  cv::InputArray       prevPts,            // Vector of 2d start points (CV_32F)
  cv::InputOutputArray nextPts,            // Results: 2d end points (CV_32F)
  cv::OutputArray      status,             // For each point, found=1, else=0
  cv::OutputArray      err,                // Error measure for found points
  cv::Size             winSize         = Size(15,15),   // size of search window
  int                  maxLevel        = 3,             // Pyramid layers to add
  cv::TermCriteria     criteria        = TermCriteria(  // How to end search
                         cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                         30,
                         0.01
                       ),
  int                  flags           = 0,    // use guesses, and/or eigenvalues
  double               minEigThreshold = 1e-4  // for spatial gradient matrix
);

 */


    cv::calcOpticalFlowPyrLK(frame1_1C, frame2_1C, frame1_features, frame2_features,
                                     optical_flow_found_feature, optical_flow_feature_error,
                                     optical_flow_window, 3, optical_flow_termination_criteria, 0 );

    cv::imshow("Optical Flow from Cam 1",frame1_1C);
    cv::imshow("Optical Flow from Cam 2",frame2_1C);

    for( i = 0; i < number_of_features; i++)
    {
        printf("%f \t, %f \n", frame1_features[i].x ,frame2_features[i].x );
    }
    //cvWaitKey(0);


    cv::Point d, b;
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


        cv::Point p,q;
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

    struct Points pointer;
    pointer.p1 = b;
    pointer.p2 = d;
    return pointer;
}

struct Points FramesToPointsHS( struct TwoFrames tf, cv::Size fs ){

    static cv::Mat frame1, frame1_1C, frame2_1C, eig_image, temp_image, pyramid1, pyramid2;
    cv::Mat flow(tf.frame1.rows, tf.frame1.cols, CV_32FC2);

/**
 void cv::calcOpticalFlowFarneback(
  cv::InputArray       prevImg,    // An input image
  cv::InputArray       nextImg,    // Image immediately subsequent to 'prevImg'
  cv::InputOutputArray flow,       // Flow vectors will be recorded here
  double               pyrScale,   // Scale between pyramid levels (< '1.0')
  int                  levels,     // Number of pyramid levels
  int                  winsize,    // Size of window for pre-smoothing pass
  int                  iterations, // Iterations for each pyramid level
  int                  polyN,      // Area over which polynomial will be fit
  double               polySigma,  // Width of fit polygon, usually '1.2*polyN'
  int                  flags       // Option flags, combine with OR operator

 The polyN argument determines the size of the area considered when fitting the polynomial around a point. This is
 different from winsize, which is used only for presmoothing. polyN could be thought of as analogous to the window
 size associated with Sobel derivatives. If this number is large, high-frequency fluctuations will not contribute to
 the polynomial fitting. Closely related to polyN is polySigma, which is the source of the intrinsic scale for the
 motion field. The derivatives computed as part of the fit use a Gaussian kernel (not the one associated with the
 smoothing) with variance polySigma and whose total extent is polyN. The value of polySigma should be a bit more than
 20% of polyN. (The pairings polyN=5, polySigma=1.1, and polyN=7, polySigma=1.5 have been found to work well and are
 recommended in source code.
);

/* Estimate optical flow for each pixel using the two-frame G. Farneback algorithm
    CVAPI(void) cvCalcOpticalFlowFarneback( const CvArr* prev, const CvArr* next,
                                            CvArr* flow, double pyr_scale, int levels,
                                            int winsize, int iterations, int poly_n,
                                            double poly_sigma, int flags );
 */

    int polyN = 5;
    double polySigma = 1.5;
    double pyrScale = 0.5;
    int winsize = 5;
    int levels = 2;
    int iterations = 3;

    cv::calcOpticalFlowFarneback(tf.frame1, tf.frame2, flow, pyrScale, levels, winsize,iterations,polyN,polySigma,
                                 cv::OPTFLOW_FARNEBACK_GAUSSIAN);

    cv::Point d, b,p,q;
    b.x = 0;
    b.y = 0;
    d.x = 0;
    d.y = 0;
    p.x = 0;
    p.y = 0;
    q.x = 0;
    q.y = 0;

    cv::imshow("Optical Flow from Cam 2", pyramid1); // zeigt die ermittelten Bewegungsframes, in x- und y-Richtung
    cv::imshow("Optical Flow from Cam", pyramid2);

    //auswertung der Velx und Vely Bilder !!!

    for(int i=0;i<pyramid1.rows;i+=3)
    {
        b.y = i; //height

        int j;
        for (j=0;j<pyramid1.cols;j+=3)
        {
            b.x = j; //width
            d.x = b.x + (int)( ( (float*)(pyramid1.data + i*pyramid1.step) )[j] );
            d.y = b.y + (int)( ( (float*)(pyramid2.data + i*pyramid2.step) )[j] );
            p.x=(p.x + b.x);
            p.y=(p.y + b.y);
            q.x=(q.x + d.x);
            q.y=(q.y + d.y);

        }
    }

    struct Points pointer;
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

struct Points CalcTriPtr (struct Points pts1,struct Points pts2,struct Points pts3){
    struct Points pts_x;
    pts_x.p1.x= 0;
    pts_x.p1.y= 0;
    pts_x.p2.x= (LaengeptsX(pts1)*3 + LaengeptsX(pts2)*2 + LaengeptsX(pts3))/6;
    pts_x.p2.y= (LaengeptsY(pts1)*3 + LaengeptsY(pts2)*2 + LaengeptsY(pts3))/6;
    return pts_x;
}


int main_opticalflow_comparison() {

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

    cv::VideoCapture input_video;
    input_video.open(0);
    if (input_video.isOpened() == NULL)
    {
        /* Either the video didn't exist OR it uses a codec OpenCV
         * doesn't support.
         */
        fprintf(stderr, "Error: No Camera found.\n");
        return -1;
    }
    printf("Cam AN \n");


    /* Read the video's frame size out of the Cam */
    cv::Size frame_size;
    frame_size.height =	(int) input_video.get(CV_CAP_PROP_FRAME_HEIGHT );
    frame_size.width =	(int) input_video.get(CV_CAP_PROP_FRAME_WIDTH );

    cvNamedWindow("Optical Flow from Cam 1", 0);
    cvNamedWindow("Optical Flow from Cam 2", 0);

    printf("Fenster AN \n");

    //zeiten fur die ausgabe
    int t_0 = 0, t_1;

    //Schreibt das Video mit
    boost::filesystem::path VideoPath = "../../../video_dataset/Mitschnitt.mpeg";
    boost::filesystem::path BildPath = "../../../video_dataset/Bild.jpg";
    boost::filesystem::path TextPath = "../../../video_dataset/Werte.txt";



    cv::Mat image, frame, gray;

    cv::VideoWriter video_out;
    video_out.open(VideoPath.string(),CV_FOURCC('P','I','M','1'),25,frame_size);
    printf("Writer eingerichtet\n");

    //strukturen f�r die Frames
    struct TwoFrames Two_F;
    struct TwoFrames Two_F2;
    cv::Mat RecordFrame;
    RecordFrame.create(frame_size, CV_8UC3);
    // Two_F2.frame1.create(frame_size, CV_8UC1 );
    // Two_F2.frame2.create(frame_size, CV_8UC1 );

    Two_F2.frame1.create(frame_size, CV_8UC1 );
    Two_F2.frame2.create(frame_size, CV_8UC1 );

    cv::Mat vel_x, vel_y;

    struct Points pts; //pts_v, pts_vv;

//	pts_v.p1.x=pts_v.p1.y=pts_v.p2.x=pts_v.p2.y=0;
//	pts_vv.p1.x=pts_vv.p1.y=pts_vv.p2.x=pts_vv.p2.y=0;

    int RecordFlag;
    FILE *datei;
    datei = fopen (TextPath.string(), "w");
    fprintf(datei,"Loop;Zeit[ms];Länge;LängeX;LängeY\n");

    //Zeitsteuerung über clock()
    clock_t start, start2, end;
    start2 = clock();

    int i = 0;
    char c = NULL;

    while(c != 'q') {

        printf("WHILE  ");

        start = clock(); // Z�hlt cpu-Ticks, ungefair 1 000 000 pro Sekunde

        input_video.read(Two_F.frame1); //holt ein Bild von der Kamera
        cv::imwrite(BildPath.string() ,Two_F.frame1); // speichert und läd das bild wieder
        Two_F.frame1 = cv::imread(BildPath.string(),0); // so verschlechtert sich die qualität

        //cvWaitKey(LOOPtime/2);

        input_video.read(Two_F.frame2); //holt ein Bild von der Kamera
        cv::imwrite(BildPath.string() ,Two_F.frame2); // speichert und läd das bild wieder
        Two_F.frame2 = cv::imread(BildPath.string(),0); // so verschlechtert sich die qualität

        end = clock();

        sprintf(str2, "Cam: %i ", (int)((end - start)));
        start = clock();

        //		frame2_1C.create( fs, CV_8UC1 );
        //		cv::cvtColor(tf.frame2, frame2_1C, CV_CVTIMG_FLIP);


        cv::cvtColor(Two_F.frame1, Two_F2.frame1,0);
        cv::cvtColor(Two_F.frame2, Two_F2.frame2,0);

		cv::imshow("Optical Flow from Cam 1", Two_F2.frame1);
		cv::imshow("Optical Flow from Cam 2", Two_F2.frame2);

        // diese Funktionen verwandeln zwei Bilder in einen 2D-Richtungsvektor
        pts = FramesToPointsHS(Two_F2,frame_size);			//mit dem HS OF Allgoritm  "cvCalcOpticalFlowHS"
        //pts = FramesToPointsPyrLK(Two_F2,frame_size);		//mit dem LK OF Allgoritm  "cvCalcOpticalFlowPyrLK"
        //pts = FramesToPointsBM(Two_F2,frame_size);				//mit dem HS OF Allgoritm  "cvCalcOpticalFlowBM"



        double angle;		angle = atan2( (double) pts.p1.y - pts.p2.y, (double) pts.p1.x - pts.p2.x );
        //double hypotenuse;	hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x) );
        FlowPoints(Two_F.frame1,pts,frame_size, angle,LOOPtime);

        end = clock();
        sprintf(str, "Alg: %i ", (int)((end - start)));
        cv::putText(Two_F.frame1, str, cv::Point(10, 10), Font_.font_face, 1,  cvScalar(0, 0, 0, 0));
        cv::putText(Two_F.frame1, str2, cv::Point(10, 20), Font_.font_face, 1,  cvScalar(0, 0, 0, 0));
        printf(str);
        printf("\tuSec \tX=%i, \tY=%i\n", (pts.p1.x - pts.p2.x), (pts.p1.y - pts.p2.y));
        cv::imshow("Optical Flow from Cam 1", Two_F.frame1);

        if (c == 'r')
            RecordFlag = 1;

        if (RecordFlag == 1){
            i++;
            cv::cvtColor(Two_F.frame1,RecordFrame,CV_CVTIMG_FLIP);
            video_out.write(RecordFrame);
            fprintf(datei,"%i;%i;%f;%i;%i\n",i,(int)(clock()-start2),Laengepts(pts),LaengeptsX(pts),LaengeptsY(pts));
        }

        c = (char)cv::waitKey(10);
        //LOOPtime = lenghtToLooptime(Laengepts(pts));

        input_video.release();
        cv::destroyAllWindows();
        std::fclose(datei);
        video_out.release();
        exit(0);
    }
}


void start_video_capture(boost::filesystem::path input_video_file) {

    cv::VideoCapture video_read;
    if ( input_video_file.empty() ) {
        video_read.open(0);
        //frame properties : fps 30 ; width : 640 ; height : 480
        //codec GPJM
    }
    else {
        video_read.open(input_video_file.string());
    }

    int width = (int)video_read.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = (int)video_read.get(cv::CAP_PROP_FRAME_HEIGHT);
    int fps = (int)video_read.get(cv::CAP_PROP_FPS);
    printf("frame properties : fps %d ; width : %d ; height : %d\n", fps, width, height);

    unsigned f = (unsigned)video_read.get(cv::CAP_PROP_FOURCC);
    printf("codec %c%c%c%c\n", (char)(f>>24), (char)(f>>16), (char)(f>>8), (char)f);

    cv::Mat image(height, width, CV_8UC3, cv::Scalar(255,255,255));
    cv::namedWindow("frame", CV_WINDOW_AUTOSIZE);

    while(char(cv::waitKey(1)) != 'q' && video_read.isOpened()) {
        video_read.grab();
        // do the heavier decoding in the second step after the grab is successful.
        video_read.retrieve(image);
        //video_read >> image;
        if (image.empty()) {
            break;
        }
        cv::imshow("frame", image);
    }

    cv::VideoWriter video_out;
    boost::filesystem::path output_video_file("../../../video_dataset/my_video.avi");
    video_out.open(
            output_video_file.string(),
            CV_FOURCC('D','I','V','X'),   // MPEG-4 codec
            30.0,                         // Frame rate (FPS)
            cv::Size( height, width),         // Write out frames at 640x480 resolution
            true                          // Expect only color frames
    );
    video_out.write(image);

    video_read.release();
    video_out.release();

}

int optical_flow_simple() {

    cv::Point2f point;
    bool addRemovePt = true;
    cv::VideoCapture cap;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
    cv::Size subPixWinSize(10, 10), winSize(31, 31);

    const int MAX_COUNT = 500;
    bool needToInit = true;
    int i = 0;


    cap.open('../../../video_dataset(megamind.avi');

    if (!cap.isOpened()) {
        std::cout << "Could not initialize capturing...\n";
        return 0;
    }

    cv::namedWindow("LK Demo", 1);

    cv::Mat gray, prevGray, image, frame;
    std::vector<cv::Point2f> points[2];

    for (;;) {
        cap >> frame;
        if (frame.empty())
            break;

        frame.copyTo(image);
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);


        if (i == 20 || needToInit) {
            // automatic initialization
            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, cv::Mat(), 3,
                                0, 0.04);
            cornerSubPix(gray, points[1], subPixWinSize, cv::Size(-1, -1),
                         termcrit);
            addRemovePt = false;
            i = 0;
        } else if (!points[0].empty()) {
            std::vector<uchar> status;
            std::vector<float> err;
            if (prevGray.empty())
                gray.copyTo(prevGray);
            cv::calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status,
                                     err, winSize, 3, termcrit, 0, 0.001);
            size_t i, k;
            for (i = k = 0; i < points[1].size(); i++) {
                if (addRemovePt) {
                    if (std::norm(point - points[1][i]) <= 5) {
                        addRemovePt = false;
                        continue;
                    }
                }

                if (!status[i])
                    continue;

                points[1][k++] = points[1][i];
                circle(image, points[1][i], 3, cv::Scalar(0, 255, 0), -1, 8);
            }
            points[1].resize(k);
        }

        if (addRemovePt && points[1].size() < (size_t) MAX_COUNT) {
            std::vector<cv::Point2f> tmp;
            tmp.push_back(point);
            cornerSubPix(gray, tmp, winSize, cv::Size(-1, -1), termcrit);
            points[1].push_back(tmp[0]);
            addRemovePt = false;
        }

        needToInit = false;
        imshow("LK Demo", image);

        char c = (char) cv::waitKey(30);
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

int main (int argc, char *argv[]) {

    boost::filesystem::path input_video_file = "../../../video_dataset/megamind.avi";
    boost::filesystem::path input_camera;
    try {
        if (boost::filesystem::exists(input_video_file) == 0) {
            throw ("No file exists");
        }
    }
    catch(std::string &e) {
        std::cout << "Error in reading video flle";
        return 0;
    }
    //start_video_capture(input_video_file);
    //start_video_capture(input_camera);
    optical_flow_simple();
    //main_opticalflow_comparison();
}

