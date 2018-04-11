#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <opencv/cv.hpp>

cv::RNG rng(12345);


void real_to_camera() {

    // Read input image
    cv::Mat im = cv::imread("../headPose.jpg");

    // 2D image points. If you change the image, you need to change vector
    std::vector<cv::Point2d> image_points;
    image_points.push_back( cv::Point2d(359, 391) );    // Nose tip
    image_points.push_back( cv::Point2d(399, 561) );    // Chin
    image_points.push_back( cv::Point2d(337, 297) );     // Left eye left corner
    image_points.push_back( cv::Point2d(513, 301) );    // Right eye right corner
    image_points.push_back( cv::Point2d(345, 465) );    // Left Mouth corner
    image_points.push_back( cv::Point2d(453, 469) );    // Right mouth corner

    // 3D model points.
    std::vector<cv::Point3d> model_points;
    model_points.push_back(cv::Point3d(0.0f, 0.0f, 0.0f));               // Nose tip
    model_points.push_back(cv::Point3d(0.0f, -330.0f, -65.0f));          // Chin
    model_points.push_back(cv::Point3d(-225.0f, 170.0f, -135.0f));       // Left eye left corner
    model_points.push_back(cv::Point3d(225.0f, 170.0f, -135.0f));        // Right eye right corner
    model_points.push_back(cv::Point3d(-150.0f, -150.0f, -125.0f));      // Left Mouth corner
    model_points.push_back(cv::Point3d(150.0f, -150.0f, -125.0f));       // Right mouth corner

    // Camera internals
    double focal_length = im.cols; // Approximate focal length.
    cv::Point2d center = cv::Point2d(im.cols/2,im.rows/2);
    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion

    std::cout << "Camera Matrix " << std::endl << camera_matrix << std::endl ;
    // Output rotation and translation
    cv::Mat rotation_vector; // Rotation in axis-angle form
    cv::Mat translation_vector;

    // Solve for pose
    cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);


    // Project a 3D point (0, 0, 1000.0) onto the image plane.
    // We use this to draw a line sticking out of the nose

    std::vector<cv::Point3d> nose_end_point3D;
    std::vector<cv::Point2d> nose_end_point2D;
    nose_end_point3D.push_back(cv::Point3d(0,0,1000.0));

    cv::projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);


    for(int i=0; i < image_points.size(); i++)
    {
        circle(im, image_points[i], 3, cv::Scalar(0,0,255), -1);
    }

    cv::line(im,image_points[0], nose_end_point2D[0], cv::Scalar(255,0,0), 2);

    std::cout << "Rotation Vector " << std::endl << rotation_vector << std::endl;
    std::cout << "Translation Vector" << std::endl << translation_vector << std::endl;

    std::cout <<  nose_end_point2D << std::endl;

    // Display image.
    cv::imshow("Output", im);
    cv::waitKey(0);
}

void camera_to_real() {

    // Read input image
    cv::Mat im = cv::imread("headPose.jpg");


    cv::Mat rvec(1,3,cv::DataType<double>::type);
    cv::Mat tvec(1,3,cv::DataType<double>::type);
    cv::Mat rotationMatrix(3,3,cv::DataType<double>::type);

    std::vector<cv::Point2f> imagePoints;
    std::vector<cv::Point3f> objectPoints;
//img points are green dots in the picture
    imagePoints.push_back(cv::Point2f(271.,109.));
    imagePoints.push_back(cv::Point2f(65.,208.));
    imagePoints.push_back(cv::Point2f(334.,459.));
    imagePoints.push_back(cv::Point2f(600.,225.));

//object points are measured in millimeters because calibration is done in mm also
    objectPoints.push_back(cv::Point3f(0., 0., 0.));
    objectPoints.push_back(cv::Point3f(-511.,2181.,0.));
    objectPoints.push_back(cv::Point3f(-3574.,2354.,0.));
    objectPoints.push_back(cv::Point3f(-3400.,0.,0.));

    double focal_length = im.cols; // Approximate focal length.
    cv::Point2d center = cv::Point2d(im.cols/2,im.rows/2);
    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion


    cv::solvePnP(objectPoints, imagePoints, camera_matrix, dist_coeffs, rvec, tvec);
    cv::Rodrigues(rvec,rotationMatrix);
    cv::Mat uvPoint = cv::Mat::ones(3,1,cv::DataType<double>::type); //u,v,1
    uvPoint.at<double>(0,0) = 363.; //got this point using mouse callback
    uvPoint.at<double>(1,0) = 222.;
    cv::Mat tempMat, tempMat2;
    double s;
    tempMat = rotationMatrix.inv() * camera_matrix.inv() * uvPoint;
    tempMat2 = rotationMatrix.inv() * tvec;
    s = 285 + tempMat2.at<double>(2,0); //285 represents the height Zconst
    s /= tempMat.at<double>(2,0);
    std::cout << "P = " << rotationMatrix.inv() * (s * camera_matrix.inv() * uvPoint - tvec) << std::endl;
}

/// Function header
void thresh_callback(int, void* );


/** @function thresh_callback */
void thresh_callback(int, void* )
{
    cv::Mat threshold_output;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    int thresh = 100;
    cv::Mat src_gray;

    /// Detect edges using Threshold
    cv::threshold( src_gray, threshold_output, thresh, 255, cv::THRESH_BINARY );
    /// Find contours
    cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    /// Approximate contours to polygons + get bounding rects and circles
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> boundRect( contours.size() );
    std::vector<cv::Point2f>center( contours.size() );
    std::vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
    { approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
        cv::minEnclosingCircle( (cv::Mat)contours_poly[i], center[i], radius[i] );
    }


    /// Draw polygonal contour + bonding rects + circles
    cv::Mat drawing = cv::Mat::zeros( threshold_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        cv::drawContours( drawing, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
        cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
        cv::circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
    }

    /// Show in a window
    cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    cv::imshow( "Contours", drawing );
}

int main ( int argc, char *argv[] ) {

    cv::RNG(12345);
    cv::Mat image(400,400,CV_8UC3, cv::Scalar::all(255));

    cv::Size obj_dimension = cv::Size(60,90);

    cv::Point3i cam_pos = {image.rows/2,image.cols/2,0};
    cv::Point2i pos_rel_to_origin = {120, 80}; //y,x
    cv::Matx41f world_pos = {pos_rel_to_origin.x, pos_rel_to_origin.y, 0, 1};

    cv::RotatedRect rect(pos_rel_to_origin, obj_dimension, 0);

    cv::Point2f vertices[4];
    rect.points(vertices);
    cv::Matx41f bbox_vertices = {vertices[0].x, vertices[1].y, 0, 0};
    for (int i = 0; i < 4; i++)
        line(image, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0), 2);
    //cv::rectangle(image, rect, cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255)));

    // Old Origin at 0,0
    cv::circle(image, cv::Point(0, 0), 2, cv::Scalar(0,0,255), 2);
    cv::putText(image, (std::to_string(pos_rel_to_origin.x) + "," + std::to_string(pos_rel_to_origin.y)), pos_rel_to_origin, CV_FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0,0,255));

    // New Origin at cam_pos
    cv::circle(image, cv::Point(cam_pos.x, cam_pos.y), 2, cv::Scalar(255,0,0), 2);
    cv::putText(image, std::to_string(cam_pos.x) + "," + std::to_string(cam_pos.y), cv::Point(cam_pos.x, cam_pos.y), CV_FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255,0,0));

    // Calculate new center of mass according to the new origin
    cv::Matx44f translateMatrix = {
            1,0,0,-cam_pos.x,
            0,1,0,-cam_pos.y,
            0,0,1,-cam_pos.z,
            0,0,0,1};
    std::cout << translateMatrix;

    cv::Matx41f coordinates, coordinates_vertices;
    coordinates =translateMatrix*world_pos;
    cv::Point2i pos_rel_to_camera = {coordinates.operator()(0), coordinates.operator()(1)};

    // Rectangle position according to cam_pos
    cv::putText(image, std::to_string(pos_rel_to_camera.x) + "," + std::to_string(pos_rel_to_camera.y), pos_rel_to_origin+cv::Point(20,20), CV_FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(cv::Scalar(255,0,0)));


    cv::Matx41f bbox_pos = {pos_rel_to_camera.x, pos_rel_to_camera.y, 0, 1};
    // Calculate new center of mass according to the new origin
    translateMatrix = {
            1,0,0,-pos_rel_to_camera.x,
            0,1,0,-pos_rel_to_camera.y,
            0,0,1,0,
            0,0,0,1};
    std::cout << translateMatrix;

    coordinates = translateMatrix*bbox_pos;
    coordinates_vertices = translateMatrix*bbox_vertices;

    cv::Point2i pos_rel_to_bbox = {coordinates(0), coordinates(1)};

    cv::Matx31f vetrices_pos = {coordinates_vertices(0), coordinates_vertices(1), coordinates_vertices(2)};
    cv::Matx31f rotated_vetrices;



    // Rectangle position according to cam_pos
    cv::putText(image, std::to_string(pos_rel_to_bbox.x) + "," + std::to_string(pos_rel_to_bbox.y), pos_rel_to_origin+cv::Point(40,40), CV_FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(cv::Scalar(255,0,0)));

    cv::putText(image, std::to_string((int)(coordinates_vertices(0))) + "," + std::to_string((int)coordinates_vertices.operator()(1)), pos_rel_to_origin-cv::Point((int)(coordinates_vertices(0)),(int)(coordinates_vertices(1))), CV_FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(cv::Scalar(0,255,0)));

    double theta = 90*CV_PI/180.0;
    cv::Matx33f Rx = {1,0,0,0,cos(theta),-sin(theta),0,sin(theta),cos(theta)};

    rotated_vetrices = Rx*vetrices_pos;
    cv::rectangle(image, cv::Rect(cv::Point(rotated_vetrices(0), (int)rotated_vetrices(1)),obj_dimension),cv::Scalar(0,0,0));



    

    cv::namedWindow("transform", CV_WINDOW_AUTOSIZE);
    cv::imshow("transform", image);
    cv::waitKey(0);

    cv::Matx33d transform = {};
    cv::destroyAllWindows();

#if 0
    cv::Mat src; cv::Mat src_gray;
    int thresh = 100;
    int max_thresh = 255;
    /// Load source image and convert it to gray
    src = cv::imread( "000011_10.png", 1 );

    /// Convert image to gray and blur it
    cv::cvtColor( src, src_gray, CV_BGR2GRAY );
    cv::blur( src_gray, src_gray, cv::Size(3,3) );

    /// Create Window
    char* source_window = "Source";
    cv::namedWindow( source_window, CV_WINDOW_AUTOSIZE );
    cv::imshow( source_window, src );

    cv::createTrackbar( " Threshold:", "Source", &thresh, max_thresh, thresh_callback );
    thresh_callback( 0 , 0);

    cv::waitKey(0);
#endif

    //real_to_camera();

}