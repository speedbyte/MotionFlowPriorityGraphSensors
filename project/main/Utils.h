//
// Created by veikas on 13.04.18.
//

#ifndef MAIN_UTILS_H
#define MAIN_UTILS_H


#include <iomanip>
#include <opencv2/highgui.hpp>

class Utils {

public:


    static void random_data_generator(std::string data_type, std::vector<float> &raw_data, std::map<float, int> hist={{0.0f, 1}}) {

        std::cout << FLT_MAX << " " << INT32_MIN << " " << INT32_MAX << " " << std::numeric_limits<float>::infinity() << std::endl;

        time_t rawtime; time(&rawtime);
        std::cout << asctime(localtime(&rawtime));
        // Seed with a real random value, if available
        std::random_device r;
        std::cout << r() << std::endl;

        std::default_random_engine e1(r());
        std::uniform_real_distribution<float> uniform_dist(-10.0f, 10.0f);
        float mean = uniform_dist(e1);
        std::cout << "Randomly-chosen mean: " << mean << '\n';

        // Generate a normal distribution around that mean
        std::seed_seq seed{r(), r(), r(), r(), r(), r(), r(), r()};
        std::mt19937 e2(seed);
        std::normal_distribution<> normal_dist(mean, 2);

        if ( data_type == "hist") {
            for (int n = 0; n < 10000; ++n) {
                ++hist[std::round((normal_dist(e2))*100)/100];
            }
            std::cout << "Normal distribution around " << mean << ":\n";
            for (auto p : hist) {
                std::cout << std::fixed << std::setprecision(2) << std::setw(2)
                          << p.first << ' ' << std::string(p.second, '*') << '\n';
            }
        }
        else if ( data_type == "vector") {
            // Choose a random mean between -10 and 10
            std::srand ( unsigned ( std::time(0) ) );

            // set some values:
            for (int i=0; i<10000; ++i) raw_data.push_back((float(std::round((normal_dist(e2))*100)/100))); // 1 2 3 4 5 6 7 8 9
            // using built-in random generator:
            std::random_shuffle ( raw_data.begin(), raw_data.end() );

            // print out content:
            /*
            std::cout << "raw_data contains:";
            for (std::vector<float>::iterator it=raw_data.begin(); it!=raw_data.end(); ++it)
                std::cout << ' ' << *it;
            */
            //randn(m1,mean,stddev);
        }

    }

    static cv::Point3f translate_and_rotate_points(const cv::Point3f initial, const cv::Point3f delta, const cv::Point3f orientation) {

        cv::Matx34f translate_matrix = {
                1,0,0,delta.x,
                0,1,0,delta.y,
                0,0,1,delta.z
        };

        float h_mat = orientation.x;
        float p_mat = orientation.y;
        float r_mat = orientation.z;

        cv::Matx33f rot_matrix = {
                cos(p_mat)*cos(h_mat) ,  -cos(r_mat)*sin(h_mat) + sin(r_mat)*sin(p_mat)*cos(h_mat),   sin(r_mat)*sin(h_mat) + cos(r_mat)*sin(p_mat)*cos(h_mat),
                cos(p_mat)*sin(h_mat) ,  cos(r_mat)*cos(h_mat) + sin(r_mat)*sin(p_mat)*sin(h_mat) ,  -sin(r_mat)*cos(h_mat) + cos(r_mat)*sin(p_mat)*sin(h_mat),
                -sin(p_mat)           ,  sin(r_mat)*cos(p_mat)                                    ,   cos(r_mat)*cos(p_mat)};

        cv::Matx31f vecPoints = {initial.x, initial.y, initial.z};
        cv::Matx31f rotated = rot_matrix * vecPoints;
        cv::Matx41f rotated_extend = {rotated(0), rotated(1), rotated(2), 1};
        cv::Matx31f translate = translate_matrix * rotated_extend;

        cv::Matx31f final = translate;

        return cv::Point3f(final(0), final(1), final(2));
    }


    static cv::Point2f worldToCamera(cv::Point3f final, float fov_rad, float fx, float fy) {

        //transform to VTD coordinates, x = depth, y = width, z = height. Hence the camera co-ordinates needs to be changed similarly.
        cv::Point3f pos = cv::Point3f(-final.y, -final.z, final.x);

        float fx_x =  pos.x * fx /pos.z;
        float fy_y =  pos.y * fy /pos.z;

        float distToImagePlane = 0.5 * Dataset::getFrameSize().height / tan(fov_rad/ 2); // [px] from camera position.
        float pxSize = 2.2e-6; // [m/px]

        float x = pos.x * ((distToImagePlane) / pos.z);
        float y = pos.y * ((distToImagePlane) / pos.z);

        // Change from optical axis to origin ( top, left )
        float x_image =  Dataset::getFrameSize().width/2 + x;
        float y_image =  Dataset::getFrameSize().height/2 + y;

        return cv::Point2f(x_image, y_image);

    }
    
    static void drawHistogramLegacy(std::vector<float> &raw_data, float &max_voted) {
        // create histogram of values for displacement.

        std::map<float, int> histogram_x, histogram_y;
        for (const auto &e : raw_data) {
            ++histogram_x[e];
        }
        cv::Point2f max_occurence = {};

        int iterator_distance;
        std::map<float, int>::iterator temp_iterator;
        std::map<float, int>::iterator temp_iterator_base;

        for ( std::map<float, int>::iterator it = histogram_x.begin(); it != histogram_x.end(); it++) {
            if (max_occurence.x <= it->second ) {
                max_occurence.x = it->second;
                max_voted = it->first;
                temp_iterator_base = it;
                iterator_distance = (int)std::distance(histogram_x.begin(), it);
                std::cout << it->first << " histogram " << it->second << "endx\n";
            }
        }


        for (const auto &x : histogram_x) {
            if (max_occurence.x <= x.second ) {
                max_occurence.x = x.second;
                max_voted = x.first;
                //std::cout << x.first << " histogram " << x.second << "endx\n";
            }
        }

        // Average around the peak
        temp_iterator = temp_iterator_base;
        if ( iterator_distance > 2 ) {
            // Forward direction
            temp_iterator = temp_iterator++;
            max_voted += (temp_iterator)->first;
            max_voted = max_voted/2;
        }
        temp_iterator = temp_iterator_base;
        if ( iterator_distance > 2 ) {
            // Backward direction
            temp_iterator = temp_iterator--;
            max_voted += (temp_iterator)->first;
            max_voted = max_voted/2;
        }
        std::cout << max_voted << " histogram " << max_occurence.x << "endx\n";
        
    }

    static void drawHistogram( std::vector<float> &raw_data, std::vector<cv::Mat> &histogram)
    {

        //random_data_generator("vector", raw_data);

        float min = *std::min_element(raw_data.begin(), raw_data.end());
        float max = *std::max_element(raw_data.begin(), raw_data.end());

        /// Establish the number of bins
        int histSize = 200;

        /// Set the ranges ( for B,G,R) )

        float range[] = { (min-0.1f*std::abs(min)), (max+0.1f*std::abs(max)) } ;

        assert(range[0]<range[1]);

        const float* histRange = { range };

        cv::Mat rangeVector;
        for ( float x = 0; x < histSize; x++) {
            rangeVector.push_back((range[0] + x*(range[1]-range[0])/histSize));
        }

        bool uniform = true; bool accumulate = false;

        cv::Mat b_hist;
        cv::Mat_<float> raw_dataMat;

        for ( auto it = raw_data.begin(); it != raw_data.end(); it++ ) {
            raw_dataMat.push_back(*it);
        }

        /// Compute the histograms:
        calcHist (&raw_dataMat, 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );

        //std::cout << b_hist;

        // Draw the histograms for B, G and R
        int hist_w = 1024; int hist_h = 768;
        int bin_w = cvRound( (double) hist_w/histSize );

        cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

        /// Normalize the result to [ 0, histImage.rows ]
        cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );


        cv::threshold(
                b_hist,
                b_hist,
                100,
                0,
                cv::THRESH_TOZERO
        );

        //std::cout << b_hist << std::endl;

        /// Draw for each channel
        for( int i = 1; i < histSize; i++ )
        {
            line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                  cv::Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                  cv::Scalar( 255, 0, 0), 2, 8, 0  );
            /*
            line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                             Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                             Scalar( 0, 255, 0), 2, 8, 0  );
            line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                             Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                             Scalar( 0, 0, 255), 2, 8, 0  );
                             */

        }

        histogram.push_back(rangeVector);
        histogram.push_back(b_hist);

        /// Display
        cv::namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
        cv::imshow("calcHist Demo", histImage );

        cv::waitKey(100);
        cv::destroyAllWindows();

    }

    static float getHistogramPeak(const std::vector<cv::Mat> &histogram) {

        float max_voted;
        double    max_val;
        cv::Point max_pt;

        cv::minMaxLoc(
                histogram.at(1),    // input histogram
                NULL,       // don't care about the min value
                &max_val,   // place to put the maximum value
                NULL,       // don't care about the location of the min value
                &max_pt     // place to put the maximum value location (a cv::Point)
        );

        //std::cout << histogram.at(1);

        max_voted = histogram.at(0).at<float>(max_pt);
        return max_voted;

    }

};



#endif //MAIN_UTILS_H
