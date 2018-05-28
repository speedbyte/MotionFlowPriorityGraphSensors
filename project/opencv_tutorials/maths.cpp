#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <gnuplot-iostream/gnuplot-iostream.h>
#include "opencv2/opencv.hpp"
//#include <cmath>
#include <cassert>

void fitLine() {


        cv::Mat img(500, 500, CV_8UC3);
        cv::RNG rng(-1);
        for(;;) {

            char key;
            int   i, count = rng.uniform(0,100) + 3, outliers = count/5;
            float a        = (float) rng.uniform(0., 200.);
            float b        = (float) rng.uniform(0., 40.);
            float angle    = (float) rng.uniform(0., CV_PI);
            float cos_a    = std::cos(angle), sin_a = std::sin(angle);
            cv::Point pt1, pt2;
            std::vector< cv::Point > points( count );
            cv::Vec4f line;
            float d, t;

            b = MIN( a*0.3f, b );
/*
            // generate some points that are close to the line
            for( i = 0; i < count - outliers; i++ ) {
                float x = (float)rng.uniform(-1.,1.)*a;
                float y = (float)rng.uniform(-1.,1.)*b;
                points[i].x = cvRound(x*cos_a - y*sin_a + img.cols/2);
                points[i].y = cvRound(x*sin_a + y*cos_a + img.rows/2);
            }

            // generate outlier points
            for( ; i < count; i++ ) {
                points[i].x = rng.uniform(0, img.cols);
                points[i].y = rng.uniform(0, img.rows);
            }
*/
            points[0] =  cv::Point( 10 , img.rows/2+10);
            points[1] =  cv::Point( 50 , img.rows/2+20);

            // draw the points
            img = cv::Scalar::all(0);
            for( i = 0; i < count; i++ )
                cv::circle(
                        img,
                        points[i],
                        2,
                        i < count - outliers
                        ? cv::Scalar(0, 0, 255)
                        : cv::Scalar(0,255,255),
                        cv::FILLED,
                        cv::LINE_AA,
                        0
                );

            // find the optimal line
            cv::fitLine( points, line, cv::DIST_L1, 1, 0.001, 0.001);

            // ... and the long enough line to cross the whole image
            // vx,vy, x, y
            d = sqrt( (double)line[0]*line[0] + (double)line[1]*line[1] );
            line[0] /= d; // normalized vector in x
            line[1] /= d; // normalized vector in y
            t = (float)(img.cols + img.rows);
            pt1.x = cvRound(line[2] - line[0]*t);
            pt1.y = cvRound(line[3] - line[1]*t);
            pt2.x = cvRound(line[2] + line[0]*t);
            pt2.y = cvRound(line[3] + line[1]*t);
            cv::line( img, pt1, pt2, cv::Scalar(0,255,0), 3, cv::LINE_AA, 0 );

            cv::imshow( "Fit Line", img );

            key = (char) cv::waitKey(0);
            if( key == 27 || key == 'q' || key == 'Q' ) // 'ESC'
                break;
        }
}

void mahalanobis() {

    /**
     * Take all camera points and calculate their magnitude scalar and store it in a matrix
     * Find the mean of the magnitude scalar
     * Find the variance of the camera magnitude samples.
     * Find the radar lemniscate model points
     * Now subtract the camera magnitude mean from a radar magnitude vector. Multiply with the transpose and the
     * covariance of the camera population.
     * Then take the square root and this is the new mahalanobis distance.
     * This way do it for each and every radar vector.
     * The final value would be a row of mahalonibis distance.
     * Do this mahalanobis distance for each object for camera, and radar. If there are two objects, then there are 4
     * DM in total.
     */
    std::vector<float> noise_x,noise_y,  realx, realy;
    std::vector<float> magnitude_real, magnitude_noise;
    std::vector<signed> theta;
    std::vector<std::pair<double, double> > xy_pts_A, xy_pts_B, xy_pts_dm ;

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(0, 360);
    for (int n = 0; n < 100; ++n) {
        //Use dis to transform the random unsigned int generated by gen into a double in [1, 2)
        theta.push_back(dis(gen));
        std::cout << theta[n] << ' '; //Each call to dis(gen) generates a new random double
    }

    std::cout << '\n';


    // Lemniscate Noise Model
    for ( int i = 0; i< 100; i++) {
        magnitude_real.push_back(0);
        magnitude_noise.push_back(0);
        // Noise
        noise_x.push_back(1*std::cos(theta[i]*CV_PI/180.0)/(1.0+std::pow(std::sin(theta[i]*CV_PI/180.0), 2))) ;
        noise_y.push_back(1*(std::cos(theta[i]*CV_PI/180.0)*std::sin(theta[i]*CV_PI/180.0))/(0.2+std::pow(sin
                                                                                                        (theta[i]*CV_PI/180.0)
                , 2))) ;
        // Real

        realx.push_back(i) ;
        realy.push_back(realx[i]);
        xy_pts_A.push_back(std::make_pair(realx[i], realy[i]));
        xy_pts_B.push_back(std::make_pair(noise_x[i], noise_y[i]));
    }
    cv::Mat mean, covar, dm, icovar(10,10,CV_32FC1);

    // magnitude_real contains 10 samples.
    // magnitude_noise contains 10 samples.
    cv::magnitude(realx,realy,magnitude_real);
    cv::magnitude(noise_x,noise_y,magnitude_noise);

    cv::Mat vec1(1,100,CV_32FC1),M2(1,100,CV_32FC1);
    memcpy(vec1.data, magnitude_real.data(), magnitude_real.size()*sizeof(float));
    memcpy(M2.data, magnitude_noise.data(), magnitude_noise.size()*sizeof(float));

    std::cout << "\nMagnitude Real\n" << vec1;
    std::cout << "\nMagnitude Noise\n" << M2;

    cv::Mat samples(2,100,CV_32FC1);
    vec1.copyTo(samples.row(0)); // magnitude real
    vec1.copyTo(samples.row(1)); // magnitude noise

    // Covariance of samples in magnitude_noise, and magnitude_real
    cv::calcCovarMatrix(vec1, covar, mean, cv::COVAR_NORMAL | cv::COVAR_SCALE | cv::COVAR_COLS);
    icovar = covar.inv();
    std::cout << "\nmean\n" << mean << "\ncovar\n" << covar <<  "\nicovar\n" << icovar << std::endl;

    icovar.convertTo(icovar,CV_32FC1);
    std::cout << vec1.type() << " " << icovar.type();

    assert(vec1.type() == M2.type());
    assert(vec1.type() == icovar.type());
    assert(vec1.size() == M2.size());
    //assert((vec1.size().width*vec1.size().height*vec1.channels()) == icovar.rows);

    for ( int i = 0; i< 100; i++ ) {
        dm.push_back(std::pow((vec1.at<float>(0)-mean.at<float>(0)),2)*icovar);
        xy_pts_dm.push_back(std::make_pair(realx[i],dm.at<float>(i)));
    }
    //dm = cv::Mahalanobis(vec1, M2, icovar);
    //plot();

    Gnuplot gp;
    gp << "set xrange [-2:2]\nset yrange [-2:2]\n";
    gp << "plot '-' with lines title 'real', '-' with points title 'noise', '+' with points title 'dm'\n";
    gp.send1d(xy_pts_A);
    gp.send1d(xy_pts_B);
    //gp.send1d(xy_pts_dm);
    usleep(1000*1000); // millis*1000
    cv::waitKey(0);
}

#define SIZE 4

void meanStdDeviation() {

    std::vector<float> vec0(SIZE),vec1(SIZE);

    cv::randu(vec0, 0, 6);
    cv::randn(vec1, 0, 4);

    for ( auto &x:vec0) {
        x = 1;
    }

    for ( auto &x:vec0) {
        x = 2;
    }

    cv::Scalar mean1, stddev1;
    cv::Scalar mean2, stddev2;

    std::copy(vec0.begin(), vec0.end(), std::ostream_iterator<float>(std::cout, " "));
    std::cout << std::endl;
    std::copy(vec1.begin(), vec1.end(), std::ostream_iterator<float>(std::cout, " "));
    std::cout << std::endl;

    cv::meanStdDev(vec0,mean1,stddev1);  // Automatically gives 4 channels.
    std::cout << "Mean " << mean1 << " Std Dev " << stddev1 << std::endl;
    cv::meanStdDev(vec1,mean2,stddev2);  // Automatically gives 4 channels.
    std::cout << "Mean " << mean2 << " Std Dev " << stddev2 << std::endl;

    cv::Mat samples1(1, SIZE, CV_32FC1, vec0.data(), sizeof(float));
    cv::Mat samples2(1, SIZE, CV_32FC1, vec1.data(), sizeof(float));

    std::cout << "samples1 " << samples1 << "\n";
    std::cout << "samples2 " << samples2 << std::endl;

    cv::Mat samples(2, SIZE, CV_32FC1);
    //samples1.copyTo(samples.row(0));
    //samples2.copyTo(samples.row(1));

    std::cout << "combine samples " << samples << std::endl;

//    samples << 2.0, 2.0, 2.0, 5.0, 6.0, 5.0, 7.0, 3.0, 4.0, 7.0,
//               6.0, 4.0, 5.0, 3.0, 4.0, 6.0, 2.0, 5.0, 1.0, 3.0;//    1, 1, 1, 1, 1, 1, 1, 1, 1, 1;

    // operator overloading is only valid for template type Mat
    //samples <<
    //        2,2,2,2,2,2,6,2,2,2;
    //        //4,4,4,4,4,4,4,4,4,5;

    cv::Mat cov1, mu1;
    cv::Mat cov2, mu2;

    // I dont need a double precision
    calcCovarMatrix(samples1, cov1, mu1, cv::COVAR_NORMAL | cv::COVAR_SCALE | cv::COVAR_COLS, CV_32FC1 );

    // I dont need a double precision
    calcCovarMatrix(samples2, cov2, mu2, cv::COVAR_NORMAL | cv::COVAR_SCALE | cv::COVAR_COLS, CV_32FC1 );

    // SCALE - divides by sample size, that is 10 in the above sample. ROWS means that the dataset elements are in columns

    std::cout << "cov: " << std::endl;
    std::cout << cov1 << std::endl;
    std::cout << "mu: " << std::endl;
    std::cout << mu1 << std::endl;


    assert(((std::floor(mean1(0)*10000))/10000.0) == ((std::floor(mu1.at<float>(0,0)*10000))/10000.0));

    std::cout << "-------------------" << std::endl;
    cv::Mat abc(2, SIZE, CV_32FC2);
    std::cout << abc.isContinuous() << std::endl;
    cv::randu(abc, 0, 10);
    std::cout << abc << std::endl;

    cv::Mat newshape = abc.reshape(1,2);

    std::cout << newshape << std::endl;

    calcCovarMatrix(newshape, cov1, mu1, cv::COVAR_NORMAL | cv::COVAR_SCALE | cv::COVAR_COLS, CV_32FC1);

    std::cout << "cov: " << std::endl;
    std::cout << cov1 << std::endl;
    std::cout << "mu: " << std::endl;
    std::cout << mu1 << std::endl;





}


void maha_points() {

    cv::Scalar mean_blue, stddev_blue;
    cv::Mat cov_blue, icov_blue, mu_blue;

    cv::Scalar mean_snow, stddev_snow;
    cv::Mat cov_snow, icov_snow, mu_snow;

    // point vector displacement x and y point2f. so its a 2 channel cluster_size. convert this 2 channel into 1 channel.
    std::vector<cv::Point2f> vec0_pts(SIZE), vec1_pts(SIZE);

    cv::randu(vec0_pts, 0, 6);
    cv::randu(vec1_pts, 0, 4);

    std::cout << vec0_pts << std::endl;

    cv::Mat samples_blue(SIZE, 1, CV_32FC2, vec0_pts.data());
    cv::Mat samples_snow(SIZE, 1, CV_32FC2, vec1_pts.data());

    //samples_blue.setTo(cv::Scalar(1,3));
    //samples_snow.setTo(cv::Scalar(2,4));

    cv::meanStdDev(samples_blue, mean_blue, stddev_blue);
    cv::meanStdDev(samples_snow, mean_snow, stddev_snow);

    std::cout << "samples_blue " << samples_blue << "\n";
    std::cout << "samples_snow " << samples_snow << std::endl;

    std::cout << "mean_blue " << mean_blue << "\n";
    std::cout << "mean_snow " << mean_snow << std::endl;

    samples_blue = samples_blue - mean_blue;
    samples_snow = samples_snow - mean_snow;

    cv::Mat samples_blue_rescale = samples_blue.reshape(1,SIZE);
    cv::Mat samples_snow_rescale = samples_snow.reshape(1,SIZE);

    cv::Mat roi_pts = samples_blue_rescale.colRange(0,2);
    std::cout << "rescale samples_blue " <<  roi_pts << std::endl;
    std::cout << "rescale samples_snow " << samples_snow_rescale << std::endl;

    calcCovarMatrix(samples_blue_rescale, cov_blue, mu_blue, cv::COVAR_NORMAL | cv::COVAR_SCALE | cv::COVAR_ROWS, CV_32FC1);
    calcCovarMatrix(samples_snow_rescale, cov_snow, mu_snow, cv::COVAR_NORMAL | cv::COVAR_SCALE | cv::COVAR_ROWS, CV_32FC1);

    std::cout << "cov_blue: " << cov_blue  << std::endl;
    std::cout << "cov_snow: " << cov_snow  << std::endl;
    std::cout << "mu_blue: " << mu_blue << std::endl;
    std::cout << "mu_snow: " << mu_snow << std::endl;

    cv::Mat cov_pooled;
    cov_pooled = ( cov_blue*samples_blue.cols + cov_snow*samples_snow.cols ) / (samples_blue.cols + samples_snow.cols);

    // matrix 2*2
    cv::Mat icov_pooled = cov_pooled.inv(cv::DECOMP_SVD);
    std::cout << "cov_pooled" << cov_pooled << "\n icov " << icov_pooled << std::endl;

    cv::Mat_<float> mean_difference(2,1);
    mean_difference.at<float>(0,0) = (float)(mean_blue(0) - mean_snow(0));
    mean_difference.at<float>(1,0) = (float)(mean_blue(1) - mean_snow(1));

    std::cout << mean_difference << std::endl;

    auto ma = mean_difference.t()*icov_pooled*mean_difference;   // 1*2 * 2*2 * 2*1
    //double ma = cv::Mahalanobis(samples1, samples2, icov);
    std::cout << ma << std::endl;



}


void cartToPolar() {
    std::vector<cv::Point2f> xy;
    xy.push_back(cv::Point2f(10, 20));
    xy.push_back(cv::Point2f(6, 8));
    xy.push_back(cv::Point2f(5, 5));

    cv::Mat_<float> xpts(xy.size(), 1, &xy[0].x, 2 * sizeof(float)); // matrix of x points
    cv::Mat_<float> ypts(xy.size(), 1, &xy[0].y, 2 * sizeof(float)); // matrix of y points

    // Proves that there is only one allocated data and Mat is simply pointing to the std::vector i.e it does not
    // have its own data.
    xy[0].x = 3;
    xy[0].y = 4;

    cv::Matx<float,3,1> magnitude;
    cv::Matx<float,3,1> angle;
    cv::cartToPolar(xpts, ypts, magnitude, angle);

    std::cout << "\nsamples_xy\n" << xy;
    std::cout << "\nmagnitude\n" << magnitude.t();
    std::cout << "\nangle\n" << (angle.t() * ( 180. / CV_PI ))<< std::endl;
    //std::cout << "\ncovar: " << covar.at<float>(0,0);
    //cv::Mahalanobis(xpts, ypts, covar);

}

void polarToCart() {

    cv::Vec2f x,y;
    cv::Vec2f mag(7,5), angle(200,70);
    cv::polarToCart(mag, angle, x, y, true);
    std::cout << x[0]+x[1] << " " << y[0]+y[1] << std::endl;

}

/**
 * Given any number of vectors, the function will compute
 * 1. The mean of the gaussian approximaiton to the distribution of the sample points.
 * 2. The covariance for the Guassian approximation to the distribution of the sample points.
 *
 */

void common(cv::Mat_<uchar> &samples_xy, std::vector<std::string> &list_gp_lines ) {

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

void calcCovarMatrix() {


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
    std::vector<cv::Mat> matPtr;
    matPtr.push_back(x_sample);
    matPtr.push_back(y_sample);
    //cv::calcCovarMatrix( &matPtr, 2, covar_x, mean_x, cv::COVAR_NORMAL|cv::COVAR_ROWS, CV_32FC1);


}

void linearPolar() {
    cv::Mat src = cv::imread("../../../pics_dataset/lena.png", 0); // read a grayscale img
    cv::Mat dst; // empty.
    cv::linearPolar(src,dst, cv::Point(src.cols/2,src.rows/2), 120, cv::INTER_CUBIC );
    cv::imshow("linear", dst);
    cv::waitKey(0);
}

void logPolar() {
    cv::Mat src = cv::imread("my.png", 0); // read a grayscale img
    cv::Mat dst; // empty.
    cv::logPolar(src,dst,cv::Point(src.cols/2,src.rows/2),40,cv::INTER_CUBIC );
    cv::imshow("linear", dst);
    cv::waitKey();
}

void solveLinear() {
    /**
     x + y + z = -1
     x + 2y + 3z = 0
     x + 3y + 4z = -2
     */
    cv::Matx<float,3,3> coefficients (1,1,1,1,2,3,1,3,4);
    cv::Matx<float,3,1> rhs(-1,0,-2);

    cv::Matx<cv::Complexf,3,1> result_object;
    cv::Matx<cv::Complexf,3,1> result_manual;
    cv::Mat_<float> result_static(3,1,CV_32FC2);

    std::cout << "---------manual method solving linear equation\n";
    result_manual = (cv::Matx<float,3,3>)coefficients.inv()*rhs;
    std::cout << result_manual << std::endl;

    std::cout << "---------object method solving linear equation\n";
    result_object = coefficients.solve(rhs); // equivalent to coefficients.inv()*rhs and this can also handle Complexf
    std::cout << result_object << std::endl;

    std::cout << "---------static method solving linear equation\n";
    cv::solve(coefficients, rhs, result_static);
    std::cout << result_static << std::endl;

    cv::Matx<float,2,2> coefficients2d (2,-3,5,6);
    cv::Matx<float,2,1> rhs2d(3,6);
    std::cout << coefficients2d.inv();
    std::cout << "---------2d object method solving linear equation\n";
    cv::Matx<cv::Complexf,2,1> result_object2d = coefficients2d.solve(rhs2d); // equivalent to coefficients.inv()*rhs
    std::cout << result_object2d << cv::determinant(coefficients2d) << std::endl;

    cv::Matx33f coefficient = {1,2,3,4,5,6,7,8,9};
    float determinant  = cv::determinant(coefficient);
    std::cout << coefficient.t() << determinant;

}

void solvePoly() {

    cv::Vec3f coefficients(6,-5,1); // 6,5,1 or 1,-1,1
    std::pair<std::complex<double>, std::complex<double>> roots_manual;
    cv::Matx<cv::Complexf,2,1> roots_static;
    cv::Matx<float,2,1>  roots_eigen;
    cv::Matx<float,2,2>  vector_eigen;

    float c = coefficients.operator()(0);
    float b = coefficients.operator()(1);
    float a = coefficients.operator()(2);

    std::cout << "---------manual method solving quadratic equation\n";
    double discriminant;
    discriminant = std::pow(b,2)-4*a*c;
    if ( discriminant < 0) {
        roots_manual = std::make_pair(std::complex<double>((-b/(2*a)),std::sqrt(-discriminant)/(2*a)),
                       std::complex<double>((-b/(2*a)),-std::sqrt(-discriminant)/(2*a)));
    }
    else {
        roots_manual = std::make_pair(((-b + std::sqrt(discriminant))/2*a), ((-b - std::sqrt(discriminant))/2*a));
    }
    std::cout << roots_manual.first << std::endl;
    std::cout << roots_manual.second << std::endl;

    std::cout << "---------static method solving quadratic equation\n";
    cv::solvePoly(coefficients, roots_static, 300); // Number of iterations
    std::cout << roots_static << std::endl;

}


void eigen() {

    cv::Matx<float,2,2> coefficients_eigen(0,1, -6, 5);
    //cv::Matx<float,2,2> coefficients_eigen(3,0,0,2);    //M << 2,1,1,2;    //M << 3,0,0,2;
    cv::Matx<float,2,1> roots_eigen;
    cv::Matx<float,2,2> vector_eigen;



    cv::Matx<float,3,3> coefficients_complex_eigen(0,1,0,0,0,1,1,0,0);
    std::cout << "---------using eigen values\n";
    cv::eigen(coefficients_eigen,roots_eigen,vector_eigen);
    std::cout << coefficients_eigen << std::endl;
    std::cout << roots_eigen << std::endl;
    std::cout << vector_eigen << std::endl;


}

void linearLeastSquare() {

}


void matrixOperations() {
    float determinant;
    cv::Matx33f coefficients (1,1,1,1,2,3,1,3,4), transpose;
    transpose = coefficients.t(); // equivalent to coefficients.inv()*rhs;
    determinant = cv::determinant(coefficients);
}

/*
    cv::calcCovarMatrix();
    cv::meanStdDev();
    cv::polarToCart();
    cv::countNonZero();
    cv::dct();
    cv::dft();
    cv::eigen();
    cv::discriminant(); // of a square matrix
    cv::magnitude(); // magnitudes from a 2D vector field.
    cv::Mahalanobis(); // compute Mahalonobis distance between two vectors.
*/

int main ( int argc, char *argv[]) {

    cv::Point3f hello;


    std::cout << "\nfitLine----------------------------------------------" << std::endl;
    //fitLine();

    std::cout << "\ncartToPolar----------------------------------------------" << std::endl;
    //cartToPolar();
    std::cout << "\npolarToCart----------------------------------------------" << std::endl;
    //polarToCart();
    std::cout << "\ncalcCovarMatrix----------------------------------------------" << std::endl;
    //calcCovarMatrix();
    std::cout << "\nsolveLinear----------------------------------------------" << std::endl;
    //solveLinear();
    std::cout << "\nsolvePoly----------------------------------------------" << std::endl;
    //solvePoly();
    std::cout << "\nmean and std dev----------------------------------------------" << std::endl;
    //meanStdDeviation();
    std::cout << "\nlinearPolar----------------------------------------------" << std::endl;
    //linearPolar();
    std::cout << "\nmahalonobis----------------------------------------------" << std::endl;
    //mahalanobis();
    std::cout << "\neigen----------------------------------------------" << std::endl;
    //eigen();
    std::cout << "\nlinear least square----------------------------------------------" << std::endl;
    //linearLeastSquare();
    std::cout << "\nangle----------------------------------------------" << std::endl;
    //angle_and_magnitude()
    maha_points();


    return 0;

}

