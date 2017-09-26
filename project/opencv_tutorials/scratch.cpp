
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

/**
/* It is Lucas & Kanade method, modified to use pyramids.
   Also it does several iterations to get optical flow for
   every point at every pyramid level.
   Calculates optical flow between two images for certain set of points (i.e.
   it is a "sparse" optical flow, which is opposite to the previous 3 methods)

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
*/


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



//pts = FramesToPointsHS(Two_F2,frame_size);			//mit dem HS OF Allgoritm  "cvCalcOpticalFlowHS"
//pts = FramesToPointsBM(Two_F2,frame_size);				//mit dem HS OF Allgoritm  "cvCalcOpticalFlowBM"
