#include <boost/bind.hpp>

#include <vires/RDBHandler.hh>
#include <vires_common.h>

#include <boost/filesystem.hpp>

#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h>




#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <boost/filesystem.hpp>

#include "datasets.h"
#include <kitti/io_flow.h>
#include "GridLayout.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>



using boost_path=boost::filesystem::path;

extern void calculate_ground_truth_image_and_flow(const boost::filesystem::path dataset_path, const std::string
unterordner);

extern void calculate_ground_truth_image_and_flow_vires(const boost::filesystem::path dataset_path, const std::string
unterordner);

extern void calculate_flow(const boost::filesystem::path dataset_path, const std::string result_sha, const std::string
image_input_sha, FRAME_TYPES frame_types, NOISE_TYPES noise);
//extern bool eval(std::string result_sha, Mail *mail);
extern void plotVectorField (FlowImage &F,std::string dir,char* prefix);

extern void read_kitti_calibration(boost::filesystem::path);
extern void of_algo(boost::filesystem::path dataset_path, std::string video, std::string algo);
extern void make_video_from_png(boost::filesystem::path dataset_path, std::string unterordner);
extern void disparity(boost::filesystem::path dataset_path);
extern boost_path get_file(const boost_path &dataset_path, const boost_path &subfolder, const boost_path
&file_name);


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


    ViresInterface vi;

    bool test_kitti_raw_dataset, test_cpp_dataset, test_matlab_dataset, test_kitti_flow_dataset, test_vires_dataset;

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

    if ( test_cpp_dataset ) {
        calculate_ground_truth_image_and_flow(CPP_DATASET_PATH, "data/stereo_flow/");

        calculate_flow(CPP_DATASET_PATH, "results/FB_image_02_slow_no_noise/", std::string
                ("image_02/"), continous_frames, no_noise);

        calculate_flow(CPP_DATASET_PATH, "results/LK_image_02_slow_no_noise/", std::string
                ("image_02/"), continous_frames, no_noise);
    }

/* MATLAB_DATASET ------------- */

    if ( test_matlab_dataset ) {
        // The ground truth calculate_flow and image is calculated directly in the matlab. Hence only results can be
        // calculated here.

        calculate_flow(MATLAB_DATASET_PATH, "results/LK_image_02_slow_no_noise/", std::string
                ("image_02_slow/no_noise/"), continous_frames, no_noise);

        calculate_flow(MATLAB_DATASET_PATH, "results/LK_image_02_slow_static_bg_noise/", std::string
                ("image_02_slow/static_BG/"), continous_frames, static_bg_noise);

        calculate_flow(MATLAB_DATASET_PATH, "results/LK_image_02_slow_static_fg_noise/", std::string
                ("image_02_slow/static_FG/"), continous_frames, static_fg_noise);

        calculate_flow(MATLAB_DATASET_PATH, "results/LK_image_02_slow_dynamic_bg_noise/", std::string
                ("image_02_slow/dynamic_BG/"), continous_frames, dynamic_bg_noise);

        calculate_flow(MATLAB_DATASET_PATH, "results/LK_image_02_slow_dynamic_fg_noise/", std::string
                ("image_02_slow/dynamic_FG/"), continous_frames, dynamic_fg_noise);
    }

/* KITTI_FLOW_DATASET------------- */

    if ( test_kitti_flow_dataset ) {
        // The ground truth calculate_flow and image is already available from the base dataset. Hence only results can be
        // calculated here.

        calculate_flow(KITTI_FLOW_DATASET_PATH, "results/LK_image_02_slow_no_noise", std::string
                ("image_02/no_noise/"), continous_frames, no_noise);

        make_video_from_png((boost::filesystem::path)KITTI_FLOW_DATASET_PATH, "data/stereo_flow/image_02/");
        make_video_from_png((boost::filesystem::path)KITTI_RAW_DATASET_PATH,
                            "data/2011_09_28_drive_0016_sync/image_02/data/");
        make_video_from_png((boost::filesystem::path)CPP_DATASET_PATH, "data/stereo_flow/image_02/");
    }

    if (test_vires_dataset ) {
        std::string m_server;
        boost::filesystem::path m_ts_gt_out_dir;

        int initCounter = 6;

        // initalize the server variable
        std::string serverName = "127.0.0.1";

        /**
        * validate the arguments given in the command line
        */
        for (int i = 1; i < argc; i++) {
            if ((argv[i][0] == '-') || (argv[i][0] == '/')) {
                switch (tolower(argv[i][1])) {
                    case 'k':        // shared memory key
                        unsigned int tempKey;
                        if (strlen(argv[i]) > 3) {
                            sscanf(&argv[i][3], "0x%x", &tempKey);
                            vi.setShmKey(tempKey);
                        }
                        break;

                    case 'c':       // check mask
                        if (strlen(argv[i]) > 3)
                            vi.setCheckMask(atoi(&argv[i][3]));
                        break;

                    case 'f':       // force reading a given buffer
                        if (strlen(argv[i]) > 3)
                            vi.setForceBuffer(atoi(&argv[i][3]));
                        break;

                    case 'v':       // verbose mode
                        vi.setVerbose(true);
                        break;

                    case 'p':        // Remote port
                        if (strlen(argv[i]) > 3)
                            vi.setPort(atoi(&argv[i][3]));
                        break;
                    case 's':       // Server
                        if (strlen(argv[i]) > 3)
                            strcpy((char *)(serverName.c_str()), &argv[i][3]);
                        break;
                    case 'h':
                    default:
                        usage();
                        break;
                }
            }
        }


/* VIRES_DATASET ------------- */

        calculate_ground_truth_image_and_flow_vires(VIRES_DATASET_PATH, "data/stereo_flow/image_02/");

        vi.setServer(serverName.c_str());

        fprintf(stderr, "ValidateArgs: key = 0x%x, checkMask = 0x%x, mForceBuffer = %d\n",
                vi.getShmKey(), vi.getCheckMask(), vi.getForceBuffer());

        // open the network connection to the taskControl (so triggers may be sent)
        fprintf(stderr, "creating network connection....\n");
        vi.openNetwork();  // this is blocking until the network has been opened
        vi.openNetwork_GT();



        // now: open the shared memory (try to attach without creating a new segment)
        fprintf(stderr, "attaching to shared memory 0x%x....\n", vi.getShmKey());

        while (!vi.getShmPtr()) {
            vi.openShm();
            usleep(1000);     // do not overload the CPU
        }

        fprintf(stderr, "...attached! Reading now...\n");

        // now check the SHM for the time being
        while (1) {
            vi.readNetwork();

            if (initCounter <= 0)
                vi.checkShm();

            // has an image arrived or do the first frames need to be triggered
            //(first image will arrive with a certain frame delay only)
            if (vi.getHaveImage() || (initCounter-- > 0)) {
                vi.sendRDBTrigger();

            }

            // ok, reset image indicator
            vi.setHaveImage(0);

            usleep(10000);
        }

        calculate_flow(VIRES_DATASET_PATH, "results/FB_image_02_slow_no_noise", std::string
                ("image_02_slow/no_noise/"), continous_frames, no_noise);

        calculate_flow(VIRES_DATASET_PATH, "results/LK_image_02_slow_no_noise", std::string
                ("image_02_slow/no_noise/"), continous_frames, no_noise);


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


}

