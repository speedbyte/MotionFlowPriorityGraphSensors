#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <gnuplot-iostream/gnuplot-iostream.h>

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
        noise_x.push_back(1*cos(theta[i]*CV_PI/180.0)/(1.0+std::pow(sin(theta[i]*CV_PI/180.0), 2))) ;
        noise_y.push_back(1*(cos(theta[i]*CV_PI/180.0)*sin(theta[i]*CV_PI/180.0))/(0.2+std::pow(sin
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

    cv::Mat M1(1,100,CV_32FC1),M2(1,100,CV_32FC1);
    memcpy(M1.data, magnitude_real.data(), magnitude_real.size()*sizeof(float));
    memcpy(M2.data, magnitude_noise.data(), magnitude_noise.size()*sizeof(float));

    std::cout << "\nMagnitude Real\n" << M1;
    std::cout << "\nMagnitude Noise\n" << M2;

    cv::Mat samples(2,100,CV_32FC1);
    M1.copyTo(samples.row(0)); // magnitude real
    M1.copyTo(samples.row(1)); // magnitude noise

    // Covariance of samples in magnitude_noise, and magnitude_real
    cv::calcCovarMatrix(M1, covar, mean, cv::COVAR_NORMAL | cv::COVAR_SCALE | cv::COVAR_COLS);
    icovar = covar.inv();
    std::cout << "\nmean\n" << mean << "\ncovar\n" << covar <<  "\nicovar\n" << icovar << std::endl;

    icovar.convertTo(icovar,CV_32FC1);
    std::cout << M1.type() << " " << icovar.type();

    assert(M1.type() == M2.type());
    assert(M1.type() == icovar.type());
    assert(M1.size() == M2.size());
    //assert((M1.size().width*M1.size().height*M1.channels()) == icovar.rows);

    for ( int i = 0; i< 100; i++ ) {
        dm.push_back(std::pow((M1.at<float>(0)-mean.at<float>(0)),2)*icovar);
        xy_pts_dm.push_back(std::make_pair(realx[i],dm.at<float>(i)));
    }
    //dm = cv::Mahalanobis(M1, M2, icovar);
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

void meanStdDeviation() {
    std::vector<int> m0(4),m1(6);

    cv::randu(m0, 0, 4);
    cv::randu(m1, 0, 4);

    cv::Scalar mean, stddev;
    std::copy(m0.begin(), m0.end(), std::ostream_iterator<float>(std::cout, " "));
    std::cout << std::endl;
    std::copy(m1.begin(), m1.end(), std::ostream_iterator<float>(std::cout, " "));
    std::cout << std::endl;

    cv::meanStdDev(m0,mean,stddev);  // Automatically gives 4 channels.
    std::cout << "Mean " << mean << " Std Dev " << stddev << std::endl;
    cv::meanStdDev(m0,mean,stddev);  // Automatically gives 4 channels.
    std::cout << "Mean " << mean << " Std Dev " << stddev << std::endl;
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
void calcCovarMatrix() {

    std::vector<std::string> list_gp_lines;
    float m,c;
    std::string coord1;
    std::string coord2;
    std::string gp_line;

    // XY, 2XY and 2X2Y all gives the same correlation
    cv::Mat_<float> covar, mean, corr;
    cv::Scalar mean_x, mean_y, stddev_x,stddev_y;
    cv::Mat_<uchar> samples_xy(2,9);
    std::vector<std::pair<double,double>> xypoints_1, xypoints_2, xypoints_3;

    cv::Vec4f line;
    cv::Mat mat_samples(1,samples_xy.cols,CV_32FC(2));

    //------------------------------------------------------------------------

    samples_xy << 1,3,2,5,8,7,12,2,4,8,6,9,4,3,3,2,7,7;
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

    for ( unsigned i = 0; i<samples_xy.cols; i++) {
        xypoints_1.push_back(std::make_pair(samples_xy[0][i], samples_xy[1][i]));
    }
    cv::meanStdDev(samples_xy.row(0),mean_x,stddev_x);


    for (unsigned i=0;i<samples_xy.cols;i++) {
        mat_samples.at<cv::Vec<float,2>>(0,i)[0] = samples_xy[0][i];
    }
    for (unsigned i=0;i<samples_xy.cols;i++) {
        mat_samples.at<cv::Vec<float,2>>(0,i)[1] = samples_xy[1][i];
    }
    cv::fitLine(mat_samples,line,CV_DIST_L2,0,0.01,0.01); // radius and angle from the origin - a kind of constraint
    m = line[1]/line[0];
    c = line[3] - line[2]*m;
    coord1 = "0," + std::to_string(c);
    coord2 = std::to_string(-c/m) + ",0";
    gp_line = "set arrow from " + coord1 + " to " + coord2 + " nohead lc rgb \'red\'\n";
    list_gp_lines.push_back(gp_line);

    //------------------------------------------------------------------------

    samples_xy.row(0) = 5*samples_xy.row(0);
    std::cout << "\nsamples_xy\n" << samples_xy;
    cv::calcCovarMatrix( samples_xy, covar, mean, cv::COVAR_NORMAL|cv::COVAR_COLS|cv::COVAR_SCALE, CV_32FC1);
    cv::meanStdDev(samples_xy.row(0),mean_x,stddev_x);
    cv::meanStdDev(samples_xy.row(1),mean_y,stddev_y);

    assert(std::floor(mean(0)*100) == std::floor(mean_x(0)*100));
    assert(std::floor(mean(1)*100) == std::floor(mean_y(0)*100));

    stddev << stddev_x[0]*stddev_x[0], stddev_x[0]*stddev_y[0], stddev_x[0]*stddev_y[0], stddev_y[0]*stddev_y[0];
    corr = covar/stddev;

    std::cout << "\nMean\n" << mean << "\nCovar\n" << covar <<
              "\nstddev_x\n" << stddev_x << "\nstddev_y\n" << stddev_y <<
              "\ncorr\n" << corr << std::endl;


    for ( unsigned i = 0; i<samples_xy.cols; i++) {
        xypoints_2.push_back(std::make_pair(samples_xy[0][i], samples_xy[1][i]));
    }

    for (unsigned i=0;i<samples_xy.cols;i++) {
        mat_samples.at<cv::Vec<float,2>>(0,i)[0] = samples_xy[0][i];
    }
    for (unsigned i=0;i<samples_xy.cols;i++) {
        mat_samples.at<cv::Vec<float,2>>(0,i)[1] = samples_xy[1][i];
    }
    cv::fitLine(mat_samples,line,CV_DIST_L2,0,0.01,0.01); // radius and angle from the origin - a kind of constraint
    m = line[1]/line[0];
    c = line[3] - line[2]*m;
    coord1 = "0," + std::to_string(c);
    coord2 = std::to_string(-c/m) + ",0";
    gp_line = "set arrow from " + coord1 + " to " + coord2 + " nohead lc rgb \'red\'\n";
    list_gp_lines.push_back(gp_line);

    //------------------------------------------------------------------------

    samples_xy.row(1) = 2*samples_xy.row(1);
    std::cout << "\nsamples_xy\n" << samples_xy;
    cv::calcCovarMatrix( samples_xy, covar, mean, cv::COVAR_NORMAL|cv::COVAR_COLS|cv::COVAR_SCALE, CV_32FC1);
    cv::meanStdDev(samples_xy.row(0),mean_x,stddev_x);
    cv::meanStdDev(samples_xy.row(1),mean_y,stddev_y);

    assert(std::floor(mean(0)*100) == std::floor(mean_x(0)*100));
    assert(std::floor(mean(1)*100) == std::floor(mean_y(0)*100));

    stddev << stddev_x[0]*stddev_x[0], stddev_x[0]*stddev_y[0], stddev_x[0]*stddev_y[0], stddev_y[0]*stddev_y[0];
    corr = covar/stddev;

    std::cout << "\nMean\n" << mean << "\nCovar\n" << covar <<
              "\nstddev_x\n" << stddev_x << "\nstddev_y\n" << stddev_y <<
              "\ncorr\n" << corr << std::endl;


    for ( unsigned i = 0; i<samples_xy.cols; i++) {
        xypoints_3.push_back(std::make_pair(samples_xy[0][i], samples_xy[1][i]));
    }

    for (unsigned i=0;i<samples_xy.cols;i++) {
        mat_samples.at<cv::Vec<float,2>>(0,i)[0] = samples_xy[0][i];
    }
    for (unsigned i=0;i<samples_xy.cols;i++) {
        mat_samples.at<cv::Vec<float,2>>(0,i)[1] = samples_xy[1][i];
    }
    cv::fitLine(mat_samples,line,CV_DIST_L2,0,0.01,0.01); // radius and angle from the origin - a kind of constraint
    m = line[1]/line[0];
    c = line[3] - line[2]*m;
    coord1 = "0," + std::to_string(c);
    coord2 = std::to_string(-c/m) + ",0";
    gp_line = "set arrow from " + coord1 + " to " + coord2 + " nohead lc rgb \'red\'\n";
    list_gp_lines.push_back(gp_line);

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
     x + y + z = 3
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
    meanStdDeviation();
    std::cout << "\nlinearPolar----------------------------------------------" << std::endl;
    //linearPolar();
    std::cout << "\nmahalonobis----------------------------------------------" << std::endl;
    //mahalanobis();
    std::cout << "\neigen----------------------------------------------" << std::endl;
    //eigen();
    std::cout << "\nlinear least square----------------------------------------------" << std::endl;
    //linearLeastSquare();
    std::cout << (1%1) << (2%1) << (1%2) << (5%3) << (3%5) << (4%1) << (0%4);

    return 0;

}

