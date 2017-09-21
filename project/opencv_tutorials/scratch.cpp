
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



struct Points FramesToPointsBM( struct TwoFrames tf, CvSize fs ){

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

 */

    int polyN = 5;
    double polySigma = 1.5;
    double pyrScale = 0.5;
    int winsize = 5;
    int levels = 2;
    int iterations = 3;

    cv::calcOpticalFlowFarneback(tf.frame1, tf.frame2, flow, pyrScale, levels, winsize,iterations,polyN,polySigma,
                                 cv::OPTFLOW_FARNEBACK_GAUSSIAN);

    cv::Point d(0,0),b(0,0),p(0,0),q(0,0);

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

    struct Points points;
    points.p1 = p;
    points.p2 = q;
    return points;
}


//pts = FramesToPointsHS(Two_F2,frame_size);			//mit dem HS OF Allgoritm  "cvCalcOpticalFlowHS"
//pts = FramesToPointsBM(Two_F2,frame_size);				//mit dem HS OF Allgoritm  "cvCalcOpticalFlowBM"
