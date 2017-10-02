
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <boost/filesystem.hpp>


#include "GridLayout.h"


#define KITTI_RAW_DATASET_PATH "../../../kitti_dataset/raw_dataset_with_calib/2011_09_28_drive_0016_sync/"
#define KITTI_RAW_CALIBRATION_PATH "../../../kitti_dataset/raw_dataset_with_calib/"
#define MATLAB_DATASET_PATH "../../../matlab_dataset/"
#define CPP_DATASET_PATH "../../../cpp_dataset/"

#define KITTI_FLOW_DATASET_PATH "../../../kitti_dataset/stereo_opticalflow_sceneflow_dataset_with_calib/training/"
#define KITTI_FLOW_CALIBRATION_PATH "../../../kitti_dataset/stereo_opticalflow_sceneflow_dataset_with_calib/calib/"

using boost_path=boost::filesystem::path;

extern void read_kitti_calibration(boost::filesystem::path);
extern void of_algo(boost::filesystem::path dataset_path, std::string video, std::string algo);
extern void make_video_from_png(boost::filesystem::path dataset_path);
extern void disparity(boost::filesystem::path dataset_path);
extern boost_path get_file(const boost_path &dataset_path, const boost_path &subfolder, const boost_path
&file_name);
extern void ground_truth();


int main ( int argc, char *argv[]) {
    // Thread 2: Read two kitti image files without rain. The two image files are from kitti

    boost::filesystem::path calib_path;
    calib_path+=KITTI_RAW_CALIBRATION_PATH;
    calib_path+="calib_cam_to_cam.txt";
    if ( !boost::filesystem::exists(calib_path) ) {
        throw ("FileNotFound error");
    }
    read_kitti_calibration(calib_path);

    boost::filesystem::path kitti_full_image_path1, kitti_full_image_path2;

    kitti_full_image_path1 = get_file(KITTI_RAW_DATASET_PATH, "image_02/data/", "0000000169.png");
    kitti_full_image_path2 = get_file(KITTI_RAW_DATASET_PATH, "image_02/data/", "0000000170.png");

    std::cout << kitti_full_image_path1 << std::endl << kitti_full_image_path2 << std::endl;

    cv::Mat image_manual_bare1(cv::imread(kitti_full_image_path1.string(), CV_LOAD_IMAGE_COLOR));
    cv::Mat image_manual_bare2(cv::imread(kitti_full_image_path2.string(), CV_LOAD_IMAGE_COLOR));

    assert(image_manual_bare1.empty() == 0);
    assert(image_manual_bare2.empty() == 0);

    GridLayout grid_manual(image_manual_bare1, image_manual_bare2);
    cv::Mat image_manual_bare_grid = grid_manual.render();
    ImageShow grid_manual_show;
    grid_manual_show.show(image_manual_bare_grid);

    cv::waitKey(0);
    cv::destroyAllWindows();

    boost::filesystem::path  dataset_path = KITTI_RAW_DATASET_PATH;
    //make_video_from_png(dataset_path);

    of_algo(dataset_path, "2011_09_28_drive_0016_sync.avi", "FB");
    of_algo(dataset_path, "2011_09_28_drive_0016_sync.avi", "LK");
    dataset_path = MATLAB_DATASET_PATH;
    of_algo(dataset_path, "Movement.avi", "FB");
    of_algo(dataset_path, "Movement.avi", "LK");
    //disparity(dataset_path);


}

