
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <boost/filesystem.hpp>

#define RAW_DATASET_PATH "../../../kitti_dataset/raw_dataset_with_calib/2011_09_28_drive_0016_sync/"
/**
 Read two manual image files without rain. The two image files are from Marcel
 Read two manual image files with rain. Converted via Dennis tool.

 Read two kitti image files without rain. The two image files are from kitti
 Read two kitti image files with rain. Converted via Dennis tool.

 Read two VIRES image files without rain. The two image files are from VIRES
 Read two VIRES image files with rain. Converted via Dennis tool.

 For the ground truth, you need an external source.

 Parallel Threads:
 Thread 1:  1. Calculate Optical Flow between the two image file
            2. Ground Truth displacement vector is generated at the time of generating the files using coordinate system.
            3. Compare with ground truth.
            4. Plot Displacement Vector
            5. Standard deviation of all the displacement vector magnitude.
            6. Standard deviation of all the displacement vector angle.
 Thread 2:  1. Use the pics and run different rain noise ( angle, how strong etc. )
            2. Ground Truth is directly from the testing data in kitti database. Kitti guys generated the ground truth
            using Velodyne.
            3. Follow Step 2 onwards from above.
 Thread 3:  1. Use the pics and run different rain noise ( angle, how strong etc. )
            2. Ground Truth is from VIRES coordinate system.
            2. Follow Step 2 onwards from above.
*/


int main ( int argc, char *argv[]) {
    // Thread 2: Read two kitti image files without rain. The two image files are from kitti

    boost::filesystem::path kitti_dataset_path, kitti_image_name1,
            kitti_image_name2, kitti_final_image_path1, kitti_final_image_path2;

    kitti_dataset_path += RAW_DATASET_PATH;
    kitti_dataset_path += "image_02/data/";
    kitti_image_name1 += "0000000169.png";
    kitti_image_name2 += "0000000170.png";
    kitti_final_image_path1 += kitti_dataset_path;
    kitti_final_image_path1 += kitti_image_name1;
    kitti_final_image_path2 += kitti_dataset_path;
    kitti_final_image_path2 += kitti_image_name2;

    try {
        if ( !boost::filesystem::exists(kitti_final_image_path1) ) {
            throw ("file not found");
        }
    }
    catch ( const char* exception) {
        std::cout << kitti_final_image_path1.string() << ":" <<  exception;
        exit(0);
    }

    try {
        if ( !boost::filesystem::exists(kitti_final_image_path2) ) {
            throw ("file not found");
        }
    }
    catch ( const char* exception) {
        std::cout << kitti_final_image_path2.string() << ":" <<  exception;
        exit(0);
    }

    std::cout << kitti_final_image_path1 << std::endl;
    cv::Mat image_manual_bare1 = cv::imread(kitti_final_image_path1.string(), CV_LOAD_IMAGE_COLOR);
    cv::Mat image_manual_bare2 = cv::imread(kitti_final_image_path2.string(), CV_LOAD_IMAGE_COLOR);
    std::cout << image_manual_bare1.type() << std::endl;
    cv::namedWindow("First Image", CV_WINDOW_AUTOSIZE);
    cv::imshow("First Image", image_manual_bare1);
    cv::namedWindow("Second Image", CV_WINDOW_AUTOSIZE);
    cv::imshow("Second Image", image_manual_bare2);
    cv::Mat image_manual_bare_grid(image_manual_bare2.rows*2, image_manual_bare2.cols, image_manual_bare1.type(), cv::Scalar(0));
    cv::Mat roi_grid_upper = cv::Mat(image_manual_bare_grid, cv::Rect(0,0,image_manual_bare1.cols,image_manual_bare1.rows));
    cv::Mat roi_grid_lower = cv::Mat(image_manual_bare_grid, cv::Rect(0,image_manual_bare1.rows,image_manual_bare1.cols,image_manual_bare1.rows));
    image_manual_bare1.copyTo(roi_grid_upper);
    image_manual_bare2.copyTo(roi_grid_lower);
    cv::namedWindow("Grid", CV_WINDOW_AUTOSIZE);
    cv::imshow("Grid", image_manual_bare_grid);
    cv::waitKey(0);


}