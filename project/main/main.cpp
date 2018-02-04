
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
#include "GroundTruthFlow.h"
#include "AlgorithmFlow.h"
#include "ObjectTrajectory.h"
#include "GroundTruthScene.h"
#include "Objects.h"
#include "RobustnessIndex.h"


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


    //bool kitti_raw_dataset.execute, cpp_dataset.execute, matlab_dataset.execute, kitti_flow_dataset.execute, vires_dataset.execute;

    boost::property_tree::ptree pt;
    boost::property_tree::read_ini("../input.txt", pt);

/*    for (auto& section : pt)
    {
        std::cout << "[" << section.first << "]\n";
        for (auto& key : section.second)
            std::cout << key.first << "=" << key.second.get_value<std::string>() << "\n";
    }
*/
    typedef struct {
        bool execute;
        bool gt;
        bool lk;
        bool fb;
        bool plot;
    } CONFIG_FILE_DATA;

    CONFIG_FILE_DATA kitti_raw_dataset, kitti_flow_dataset, matlab_dataset, cpp_dataset, vires_dataset;

    try {
        std::strcmp(pt.get<std::string>("MATLAB_DATASET.EXECUTE").c_str(), "0") == 0 ? matlab_dataset.execute = false :
                matlab_dataset.execute = true;
        std::strcmp(pt.get<std::string>("KITTI_RAW_DATASET.EXECUTE").c_str(), "0") == 0 ? kitti_raw_dataset.execute =
                                                                                                  false :
                kitti_raw_dataset.execute = true;
        std::strcmp(pt.get<std::string>("KITTI_FLOW_DATASET.EXECUTE").c_str(), "0") == 0 ? kitti_flow_dataset.execute =
                                                                                                   false :
                kitti_flow_dataset.execute = true;

        std::strcmp(pt.get<std::string>("CPP_DATASET.EXECUTE").c_str(), "0") == 0 ? cpp_dataset.execute = false :
                cpp_dataset.execute = true;
        std::strcmp(pt.get<std::string>("CPP_DATASET.GT").c_str(), "0") == 0 ? cpp_dataset.gt = false :
        cpp_dataset.gt = true;
        std::strcmp(pt.get<std::string>("CPP_DATASET.FB").c_str(), "0") == 0 ? cpp_dataset.fb = false :
                cpp_dataset.fb = true;
        std::strcmp(pt.get<std::string>("CPP_DATASET.LK").c_str(), "0") == 0 ? cpp_dataset.lk = false :
                cpp_dataset.lk = true;
        std::strcmp(pt.get<std::string>("CPP_DATASET.PLOT").c_str(), "0") == 0 ? cpp_dataset.plot = false :
                cpp_dataset.plot = true;

        std::strcmp(pt.get<std::string>("VIRES_DATASET.EXECUTE").c_str(), "0") == 0 ? vires_dataset.execute = false :
                vires_dataset.execute = true;
        std::strcmp(pt.get<std::string>("VIRES_DATASET.GT").c_str(), "0") == 0 ? vires_dataset.gt = false :
                vires_dataset.gt = true;
        std::strcmp(pt.get<std::string>("VIRES_DATASET.FB").c_str(), "0") == 0 ? vires_dataset.fb = false :
                vires_dataset.fb = true;
        std::strcmp(pt.get<std::string>("VIRES_DATASET.LK").c_str(), "0") == 0 ? vires_dataset.lk = false :
                vires_dataset.lk = true;
        std::strcmp(pt.get<std::string>("VIRES_DATASET.PLOT").c_str(), "0") == 0 ? vires_dataset.plot = false :
                vires_dataset.plot = true;


    }
    catch(boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::property_tree
    ::ptree_bad_path> >) {
        std::cerr << "Corrupt config file\n";
        throw;
    }


    if ( kitti_raw_dataset.execute ) {
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

    cv::Size_<unsigned> frame_size(1242, 375);
    ushort depth = CV_8U;
    ushort cn = 3;
    {
        if ( cpp_dataset.execute ) {

            std::string input = "data/stereo_flow/image_02";
            Dataset::fillDataset(frame_size, depth, cn, CPP_DATASET_PATH, "data/stereo_flow/image_02", "results");

            if ( cpp_dataset.gt ) {

                Rectangle background(1242, 375);
                NoTrajectory noTrajectory;

                // Canvas is itself registered as an Object with a dummy trajectory
                WhiteNoise whiteNoise;
                ColorfulNoise colorfulNoise;
                Canvas canvas(background, noTrajectory, 60, whiteNoise);

                // Further objects
                Rectangle rectangle1(30, 30); // width, height
                Rectangle rectangle2(100, 30);
                Achterbahn trajectory;
                NoNoise noNoise;

                //Objects obj1(rectangle1, trajectory, 60, colorfulNoise, "rectangle_long");
                Objects obj2(rectangle2, trajectory, 120, noNoise, "rectangle_wide");

                std::vector<Objects> list_of_objects;
                //list_of_objects.push_back(obj1);
                list_of_objects.push_back(obj2);


                GroundTruthSceneInternal gt_scene(canvas, list_of_objects);
                gt_scene.generate_gt_scene();

                GroundTruthFlow gt_flow(list_of_objects);
                gt_flow.generate_gt_scenepixel_displacement();

            }

            AlgorithmFlow fback;
            AlgorithmFlow lkanade;
            if ( cpp_dataset.fb ) {
                fback.calculate_flow(fb, continous_frames, no_noise);
            }

            if ( cpp_dataset.lk ) {
                lkanade.calculate_flow(lk, continous_frames, no_noise);
            }

            if ( cpp_dataset.plot ) {

                PixelRobustness robust;
                std::string resultordner;
                fback.setResultOrdner(fb, continous_frames, no_noise);
                robust.generatePixelRobustness(fback.getResultOrdner());
                //PlotFlow::plot(std::string("results_FB_no_noise"));
                //PlotFlow::plot(std::string("results_LK_no_noise"));

            }
        }
    }

/* MATLAB_DATASET ------------- */

    {
        if ( matlab_dataset.execute ) {

            std::string input = "data/stereo_flow/image_02";
            Dataset::fillDataset(frame_size, depth, cn, MATLAB_DATASET_PATH, input, "results");
            AlgorithmFlow algo;
            // The ground truth calculate_flow and image is calculated directly in the matlab. Hence only results can be
            // calculated here.

            algo.calculate_flow(fb, continous_frames, no_noise);

            algo.calculate_flow(fb, continous_frames, static_bg_noise);

            algo.calculate_flow(fb, continous_frames, static_fg_noise);

            algo.calculate_flow(fb, continous_frames, dynamic_bg_noise);

            algo.calculate_flow(fb, continous_frames, dynamic_fg_noise);
        }
    }

/* KITTI_FLOW_DATASET------------- */

    {
        if ( kitti_flow_dataset.execute ) {

            Dataset::fillDataset(frame_size, depth, cn, KITTI_FLOW_DATASET_PATH, "data/stereo_flow/image_02",
                                 "results");
            AlgorithmFlow algo;
            // The ground truth calculate_flow and image is already available from the base dataset. Hence only results can be
            // calculated here.

            algo.calculate_flow(fb, continous_frames, no_noise);

            //make_video_from_png((boost::filesystem::path)KITTI_FLOW_DATASET_PATH, "data/stereo_flow/image_02/");
            //make_video_from_png((boost::filesystem::path)KITTI_RAW_DATASET_PATH,"data/2011_09_28_drive_0016_sync/image_02/data/");
            //make_video_from_png((boost::filesystem::path)CPP_DATASET_PATH, "data/stereo_flow/image_02/");
        }
    }

/* VIRES_DATASET ------------- */

    {
        if (vires_dataset.execute ) {

            cv::Size_<unsigned> frame_size_vires(1242, 375);

            std::string scenario = "truck";
            std::string input = "data/stereo_flow/image_02_" + scenario;
            Dataset::fillDataset(frame_size, depth, cn, VIRES_DATASET_PATH, input, "results");

            if ( vires_dataset.gt ) {
                GroundTruthSceneExternal gt_scene(scenario);
                gt_scene.generate_gt_scene();
                std::vector<Objects> list_of_objects = gt_scene.getListOfObjects();
                GroundTruthFlow gt_flow(list_of_objects);
                gt_flow.generate_gt_scenepixel_displacement();
            }

            AlgorithmFlow fback;
            AlgorithmFlow lkanade;
            if ( vires_dataset.lk ) {
                fback.calculate_flow(lk, continous_frames, no_noise);
            }

            if ( vires_dataset.fb ) {
                lkanade.calculate_flow(fb, continous_frames, no_noise);
            }

            if ( vires_dataset.plot ) {
                PlotFlow::plot(std::string("results_FB_no_noise"));
                PlotFlow::plot(std::string("results_LK_no_noise"));
            }
            //disparity(dataset_path);
        }
    }

    std::cout << "End of Program\n";
}

