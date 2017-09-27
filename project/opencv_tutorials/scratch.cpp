
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


scratch() {
    std::vector<std::vector<float>> all_x;
    std::vector<std::vector<float>> all_y;

    std::vector<float> displacement_vector_x;
    std::vector<float> displacement_vector_y;

    cv::Point frame_center;
    frame_center.x = frame_size.width/2;
    frame_center.y = frame_size.height/2;

    displacement_vector_x.clear();
    displacement_vector_y.clear();
    //std::vector<float>().swap(displacement_vector_x);

    displacement_vector_x.push_back(cv::abs(prev_pts[i].x - next_pts[i].x));
    displacement_vector_y.push_back(cv::abs(prev_pts[i].y - next_pts[i].y));

    double magnitude = cv::norm(prev_pts[i] - next_pts[i]);
    std::cout << "Mag" << magnitude << std::endl;

    sprintf(str, "X = %f ", (displacement_vector_x[i]));
    cv::putText(frame, str, cv::Point(frame_size.width-40, 10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));

    sprintf(str, "Y = %f ", (displacement_vector_y[i]));
    cv::putText(frame, str, cv::Point(frame_size.width-40, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));

    cv::arrowedLine(frame, prev_pts[i], next_pts[i] , cv::Scalar(0,255,0), 1, CV_AA, 0);

    end = clock();

    sprintf(str, "Alg: %i ", (int) ((end - start)));
    cv::putText(frame, str, cv::Point(10, 10), cv::FONT_HERSHEY_PLAIN, 1, cvScalar(0, 0, 0));
    cv::putText(frame, str2, cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1, cvScalar(0, 0, 0));

    //fprintf(datei, "%i;%i;%f;%f;%f\n", (int)i, (int) (clock() - start2), magnitude,
    //        displacement_vector_x[i], displacement_vector_y[i] );
    all_x.push_back(displacement_vector_x);
    all_y.push_back(displacement_vector_y);

}


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
