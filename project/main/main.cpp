
#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include "GridLayout.h"
#include "datasets.h"
#include "groundTruth.h"


//extern bool eval(std::string result_sha, Mail *mail);
//extern void plotVectorField (FlowImage &F,std::string dir,char* prefix);

extern void read_kitti_calibration(boost::filesystem::path);
//extern void of_algo(boost::filesystem::path dataset_path, std::string video, std::string algo);
extern void make_video_from_png(boost::filesystem::path dataset_path, std::string unterordner);
//extern void disparity(boost::filesystem::path dataset_path);
extern boost::filesystem::path get_file(const boost::filesystem::path &dataset_path, const boost::filesystem::path
&subfolder, const boost::filesystem::path &file_name);


/**
* information about usage of the software
* this method will exit the program
*/
void usage()
{
    printf("usage: videoTest [-k:key] [-c:checkMask] [-v] [-f:bufferId] [-p:x] [-s:IP] [-h]\n\n");
    printf("       -k:key        SHM key that is to be addressed\n");
    printf("       -c:checkMask  mask against which to check before reading an SHM buffer\n");
    printf("       -f:bufferId   force reading of a given buffer (0 or 1) instead of checking for a valid checkMask\n");
    printf("       -p:x          Remote port to send to\n");
    printf("       -s:IP         Server's IP address or hostname\n");
    printf("       -v            run in verbose mode\n");
    exit(1);
}


/**
* main program with high frequency loop for checking the shared memory;
* does nothing else
*/


int main ( int argc, char *argv[]) {
    // Thread 2: Read two kitti image files without rain. The two image files are from kitti


    bool test_kitti_raw_dataset, test_cpp_dataset, test_matlab_dataset, test_kitti_flow_dataset, test_vires_dataset;
    bool generate_ground_truth, generate_LK, generate_FB;


    boost::property_tree::ptree pt;
    boost::property_tree::read_ini("../input.txt", pt);

    for (auto& section : pt)
    {
        std::cout << "[" << section.first << "]\n";
        for (auto& key : section.second)
            std::cout << key.first << "=" << key.second.get_value<std::string>() << "\n";
    }

    try {
        std::cout << pt.get<std::string>("CPP_DATASET.EXECUTE") << std::endl;
        std::strcmp(pt.get<std::string>("CPP_DATASET.EXECUTE").c_str(), "0") == 0 ? test_cpp_dataset = false :
                test_cpp_dataset = true;
        std::strcmp(pt.get<std::string>("MATLAB_DATASET.EXECUTE").c_str(), "0") == 0 ? test_matlab_dataset = false :
                test_matlab_dataset = true;
        std::strcmp(pt.get<std::string>("VIRES_DATASET.EXECUTE").c_str(), "0") == 0 ? test_vires_dataset = false :
                test_vires_dataset = true;
        std::strcmp(pt.get<std::string>("KITTI_RAW_DATASET.EXECUTE").c_str(), "0") == 0 ? test_kitti_raw_dataset =
                                                                                                  false :
                test_kitti_raw_dataset = true;
        std::strcmp(pt.get<std::string>("KITTI_FLOW_DATASET.EXECUTE").c_str(), "0") == 0 ? test_kitti_flow_dataset =
                                                                                                   false :
                test_kitti_flow_dataset = true;
        std::strcmp(pt.get<std::string>("CPP_DATASET.GT").c_str(), "0") == 0 ? generate_ground_truth = false :
        generate_ground_truth = true;
        std::strcmp(pt.get<std::string>("CPP_DATASET.FB").c_str(), "0") == 0 ? generate_FB = false :
                generate_FB = true;
        std::strcmp(pt.get<std::string>("CPP_DATASET.LK").c_str(), "0") == 0 ? generate_LK = false :
                generate_LK = true;
    }
    catch(boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::property_tree
    ::ptree_bad_path> >) {
        std::cerr << "Corrupt config file\n";
        throw;
    }


    if ( test_kitti_raw_dataset ) {
        boost::filesystem::path calib_path = get_file(KITTI_RAW_CALIBRATION_PATH,
                                                      "./", "calib_cam_to_cam.txt");
        read_kitti_calibration(calib_path);

        boost::filesystem::path kitti_full_image_path1 = get_file(KITTI_RAW_DATASET_PATH,
                                                                  "data/2011_09_28_drive_0016_sync/image_02/data/",
                                                                  "0000000169.png");
        boost::filesystem::path kitti_full_image_path2 = get_file(KITTI_RAW_DATASET_PATH,
                                                                  "data/2011_09_28_drive_0016_sync/image_02/data/",
                                                                  "0000000170.png");

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

        std::string result_dir;
        //test_kitti_original();
    }

/* CPP_DATASET ------------- */

    GroundTruth gt(CPP_DATASET_PATH, "data/stereo_flow/", "results/");
    if ( test_cpp_dataset ) {

        if ( generate_ground_truth ) {
            gt.generate_gt_image_and_gt_flow();
        }

        if ( generate_FB ) {
            gt.calculate_flow(CPP_DATASET_PATH, std::string("image_02/"), fb, continous_frames, no_noise);
        }

        if ( generate_LK ) {
            gt.calculate_flow(CPP_DATASET_PATH, std::string("image_02/"), lk, continous_frames, no_noise);
        }

        gt.plot(std::string("results_FB_no_noise"));
    }

/* MATLAB_DATASET ------------- */

    if ( test_matlab_dataset ) {
        // The ground truth calculate_flow and image is calculated directly in the matlab. Hence only results can be
        // calculated here.

        gt.calculate_flow(MATLAB_DATASET_PATH, std::string("image_02/"), fb, continous_frames, no_noise);

        gt.calculate_flow(MATLAB_DATASET_PATH, std::string("image_02/"), fb, continous_frames, static_bg_noise);

        gt.calculate_flow(MATLAB_DATASET_PATH, std::string("image_02/"), fb, continous_frames, static_fg_noise);

        gt.calculate_flow(MATLAB_DATASET_PATH, std::string("image_02/"), fb, continous_frames, dynamic_bg_noise);

        gt.calculate_flow(MATLAB_DATASET_PATH, std::string("image_02/"), fb, continous_frames, dynamic_fg_noise);
    }

/* KITTI_FLOW_DATASET------------- */

    if ( test_kitti_flow_dataset ) {
        // The ground truth calculate_flow and image is already available from the base dataset. Hence only results can be
        // calculated here.

        gt.calculate_flow(KITTI_FLOW_DATASET_PATH, std::string("image_02/"), fb, continous_frames, no_noise);

        make_video_from_png((boost::filesystem::path)KITTI_FLOW_DATASET_PATH, "data/stereo_flow/image_02/");
        make_video_from_png((boost::filesystem::path)KITTI_RAW_DATASET_PATH,
                            "data/2011_09_28_drive_0016_sync/image_02/data/");
        make_video_from_png((boost::filesystem::path)CPP_DATASET_PATH, "data/stereo_flow/image_02/");
    }

    if (test_vires_dataset ) {

/* VIRES_DATASET ------------- */

        GroundTruth gt(VIRES_DATASET_PATH, "data/stereo_flow/", "results/");
        if ( generate_ground_truth ) {
            gt.generate_gt_image_and_gt_flow_vires();
        }

        int m_port;
        int m_sensor_port;
        bool m_triggers;
        int m_frames_to_read;
        bool m_write_ts_gt;

        //of_algo(dataset_path, "2011_09_28_drive_0016_sync.avi", "FB");
        //of_algo(dataset_path, "2011_09_28_drive_0016_sync.avi", "LK");
        //dataset_path = MATLAB_DATASET_PATH;
        //of_algo(dataset_path, "Movement.avi", "FB");
        //of_algo(dataset_path, "Movement.avi", "LK");
        //disparity(dataset_path);

    }

    std::cout << "End of Program\n";
}

