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
#include "GroundTruthScene.h"
#include "GroundTruthObjects.h"
#include "RobustnessIndex.h"


//extern bool eval(std::string result_sha, Mail *mail);
//extern void plotVectorField (FlowImage &F,std::string dir,char* prefix);

extern void read_kitti_calibration(boost::filesystem::path);
//extern void of_algo(boost::filesystem::path dataset_path, std::string video, std::string algo);
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


/*    for (auto& section : pt)
    {
        std::cout << "[" << section.first << "]\n";
        for (auto& key : section.second)
            std::cout << key.first << "=" << key.second.get_value<std::string>() << "\n";
    }
*/
    typedef struct {
        std::string path;
        bool execute;
        bool gt;
        bool lk;
        bool fb;
        bool plot;
    } CONFIG_FILE_DATA;

    CONFIG_FILE_DATA
            cpp_dataset,
            matlab_dataset,
            kitti_flow_dataset,
            vkitti_flow_dataset,
            kitti_raw_dataset,
            vkitti_raw_dataset,
            vires_dataset,
            radar_dataset
    ;

    cv::FileStorage fs_configfile;
    fs_configfile.open("../input.yml", cv::FileStorage::READ);
    if (!fs_configfile.isOpened())
    {
        cerr << "Failed to open " << endl;
        return 1;
    }

    std::map<std::string, CONFIG_FILE_DATA*> map_object;

    map_object["CPP_DATASET"] =  &cpp_dataset;
    map_object["MATLAB_DATASET"] =  &matlab_dataset;
    map_object["KITTI_FLOW_DATASET"] =  &kitti_flow_dataset;
    map_object["KITTI_RAW_DATASET"] =  &kitti_raw_dataset;
    map_object["VKITTI_FLOW_DATASET"] = &vkitti_flow_dataset;
    map_object["VKITTI_RAW_DATASET"] = &vkitti_raw_dataset;
    map_object["RADAR_DATASET"] = &radar_dataset;
    map_object["VIRES_DATASET"] = &vires_dataset;

    cv::FileNode node;
    for (std::map<std::string ,CONFIG_FILE_DATA*>::iterator it=map_object.begin(); it!=map_object.end(); ++it)  {

        std::cout << it->first << '\n';
        node = fs_configfile[(*it).first];
        if ( node.isNone() || node.empty() ) {
            std::cout << (*it).first << " cannot be found" << std::endl;
        }
        it->second->path = node["PATH"].string();
        it->second->execute = (bool)(int)node["EXECUTE"];
        it->second->gt = (bool)(int)(node["GT"]);
        it->second->lk = (bool)(int)(node["LK"]);
        it->second->fb = (bool)(int)(node["FB"]);
        it->second->plot = (bool)(int)(node["PLOT"]);
        //map_input_txt_to_main.push_back(map_object);
        std::cout << it->second->path << " " << it->second->execute << " " << it->second->gt << std::endl;

    }

    /*
D     * novel real-to-virtual cloning method. Photo realistic synthetic dataaset consisting of 50 high resolution monocular
     * videos ( 21260 frames ) for five different virtual worlds in urban settings under different imaging and weather c
     * conditions.
     */
    if ( vkitti_raw_dataset.execute ) {

    }

    if ( radar_dataset.execute ) {

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

    ushort depth = CV_8U;
    ushort cn = 3;
    assert(MAX_ITERATION_RESULTS <= MAX_ITERATION_GT_SCENE_GENERATION_VECTOR);
    assert(MAX_ITERATION_RESULTS <= MAX_ITERATION_GT_SCENE_GENERATION_IMAGES);
    assert(MAX_ITERATION_GT_SCENE_GENERATION_IMAGES <= MAX_ITERATION_GT_SCENE_GENERATION_VECTOR);

    const std::vector < std::string> scenarios_list = {"two"};
    //const std::vector < std::string> environment_list = {"none", "snow", "rain"};
    //const std::vector < std::string> environment_list = {"none", "night"};
    const std::vector < std::string> environment_list = {"none"};

    cv::FileStorage fs;


    std::vector<GroundTruthObjects> list_of_gt_objects_base;
    std::vector<SimulatedObjects> list_of_simulated_objects_base;
    std::vector<Objects *> ptr_list_of_gt_objects_base, ptr_list_of_simulated_objects_base;

    for ( ushort env_index = 0; env_index< environment_list.size(); env_index++) {

        if ( cpp_dataset.execute || vires_dataset.execute ) {

            std::vector<GroundTruthObjects> list_of_gt_objects;
            std::vector<Objects *> ptr_list_of_gt_objects;
            GroundTruthObjects::objectCurrentCount = 0;

            cv::Size_<unsigned> frame_size(800, 600);
            std::string input = "data/stereo_flow/" + scenarios_list[0] + "/";
            std::string output = "results/stereo_flow/" + scenarios_list[0] + "/";

            GroundTruthScene *base_ptr_gt_scene;

            if ( vires_dataset.execute ) {

                Dataset::fillDataset(frame_size, depth, cn, VIRES_DATASET_PATH, input, output);
                // The first iteration "none" will fil the objects_base and the ptr_objects_base and thereafter it is simply visible
                // through out the life cycle of the program.
                GroundTruthSceneExternal gt_scene(scenarios_list[0], environment_list[env_index], list_of_gt_objects_base, vires_dataset.gt);
                base_ptr_gt_scene = &gt_scene;
                base_ptr_gt_scene->prepare_directories();
                base_ptr_gt_scene->generate_gt_scene();
                base_ptr_gt_scene->generate_bounding_box();
                base_ptr_gt_scene->generate_bird_view();

                if ( (env_index == environment_list.size()-1 ) && vires_dataset.gt) {
                    base_ptr_gt_scene->stopSimulation();
                }

            }

            else if ( cpp_dataset.execute  ) {

                Dataset::fillDataset(frame_size, depth, cn, CPP_DATASET_PATH, input, output);
                GroundTruthSceneInternal gt_scene(scenarios_list[0], environment_list[env_index], list_of_gt_objects_base, cpp_dataset.gt);
                base_ptr_gt_scene = &gt_scene;
                base_ptr_gt_scene->prepare_directories();
                base_ptr_gt_scene->generate_gt_scene();
                base_ptr_gt_scene->generate_bird_view();

            }

            if ( environment_list[env_index] == "none") {
                for ( auto obj_count = 0; obj_count < list_of_gt_objects_base.size(); obj_count++ ) {
                    ptr_list_of_gt_objects_base.push_back(&(list_of_gt_objects_base.at(obj_count)));
                }
            }
            for ( auto obj_count = 0; obj_count < list_of_gt_objects_base.size(); obj_count++ ) {
                list_of_gt_objects.push_back(list_of_gt_objects_base.at(obj_count)) ;
            }
            ptr_list_of_gt_objects = ptr_list_of_gt_objects_base;


            // Generate Groundtruth data flow --------------------------------------
            if ( environment_list[env_index] == "none") {

                fs.open((Dataset::getGroundTruthPath().string() + "/values.yml"), cv::FileStorage::WRITE);

                GroundTruthFlow gt_flow(ptr_list_of_gt_objects_base, ptr_list_of_gt_objects);
                gt_flow.generate_flow_frame();

                gt_flow.generate_collision_points();
                gt_flow.generate_shape_points(); // this is to just create Jaccard Index  =  1

                if ( (cpp_dataset.plot && cpp_dataset.execute) || (vires_dataset.plot && vires_dataset.execute) ) {

                    PixelRobustness pixelRobustness(fs);
                    VectorRobustness vectorRobustness(fs);

                    vectorRobustness.generateVectorRobustness(gt_flow);
                    pixelRobustness.generatePixelRobustness(gt_flow);

                    vectorRobustness.make_video_from_png(gt_flow.getGeneratePath());

                }
            }
        }
    }

    /*
     To summarize, we compare six costs: sampling-insensitive absolute differences (BT), three filter-based costs (LoG, Rank, and Mean), hierarchical mutual information (HMI), and normalized cross-correlation (NCC).*
     */

    // Generate Algorithm data flow --------------------------------------
    for ( ushort env_index = 0; env_index< environment_list.size(); env_index++) {

        if ( cpp_dataset.execute || vires_dataset.execute ) {

            std::vector<SimulatedObjects> list_of_simulated_objects;
            std::vector<Objects *> ptr_list_of_simulated_objects;
            SimulatedObjects::SimulatedobjectCurrentCount = 0;

            list_of_simulated_objects.clear();
            for ( ushort obj_count = 0; obj_count < list_of_gt_objects_base.size(); obj_count++ ) {
                //two objects
                int width = list_of_gt_objects_base.at(obj_count).getInertialWidth();
                int height = list_of_gt_objects_base.at(obj_count).getInertialHeight();
                std::vector<std::vector<bool> >  extrapolated_visibility = list_of_gt_objects_base.at(obj_count).get_obj_extrapolated_visibility();

                SimulatedObjects objects( ("simulated_" + list_of_gt_objects_base.at(obj_count).getObjectName()), width, height, extrapolated_visibility);
                list_of_simulated_objects.push_back(objects);
            }

            for ( auto obj_count = 0; obj_count < list_of_simulated_objects.size(); obj_count++ ) {
                ptr_list_of_simulated_objects.push_back(&list_of_simulated_objects.at(obj_count));
            }

            if ( (cpp_dataset.fb && cpp_dataset.execute) || (vires_dataset.fb && vires_dataset.execute )) {

                AlgorithmFlow fback( environment_list[env_index], ptr_list_of_gt_objects_base, ptr_list_of_simulated_objects);

                fback.generate_flow_frame(fb, video_frames, environment_list[env_index], list_of_simulated_objects_base);

                if ( environment_list[env_index] == "none" ) { // store the stimulated objects from the ground run.

                    for ( auto obj_count = 0; obj_count < list_of_simulated_objects.size(); obj_count++ ) {
                        list_of_simulated_objects_base.push_back(list_of_simulated_objects.at(obj_count));
                    }
                    ptr_list_of_simulated_objects_base = ptr_list_of_simulated_objects;

                }

                for ( ushort i = 0; i < list_of_simulated_objects.size(); i++) {
                    // m_obj_extrapolated_stencil_pixel_point_pixel_displacement
                    list_of_simulated_objects.at(i)
                            .generate_obj_extrapolated_mean_pixel_centroid_pixel_displacement(MAX_SKIPS, list_of_simulated_objects.at(i).get_obj_extrapolated_stencil_pixel_point_pixel_displacement(), list_of_simulated_objects.at(i).get_obj_extrapolated_shape_visibility());
                    list_of_simulated_objects.at(i).generate_obj_line_parameters(MAX_SKIPS);
                }

                fback.generate_collision_points_mean();
                fback.generate_shape_points();
                fback.visualiseStencil();

                if ( (cpp_dataset.plot && cpp_dataset.execute) || (vires_dataset.plot && vires_dataset.execute )) {

                    PixelRobustness pixelRobustness(fs);
                    VectorRobustness vectorRobustness(fs);

                    vectorRobustness.generateVectorRobustness(fback);
                    pixelRobustness.generatePixelRobustness(fback);

                    vectorRobustness.make_video_from_png(fback.getImageAbholOrt());

                }
            }
        }
    }
    fs.release();


/* MATLAB_DATASET ------------- */

    {
        if ( matlab_dataset.execute ) {

            cv::Size_<unsigned> frame_size(1242, 375);
            std::string input = "data/stereo_flow/image_02";
            Dataset::fillDataset(frame_size, depth, cn, MATLAB_DATASET_PATH, input, "results");
            //AlgorithmFlow algo();

        }
    }

/* KITTI_FLOW_DATASET------------- */

    {
        if ( kitti_flow_dataset.execute ) {

            cv::Size_<unsigned> frame_size(1242, 375);
            Dataset::fillDataset(frame_size, depth, cn, KITTI_FLOW_DATASET_PATH, "data/stereo_flow/image_02",
                                 "results");
            //AlgorithmFlow algo;
            // The ground truth generate_flow_frame and image is already available from the base dataset. Hence only results can be
            // calculated here.

            //algo.generate_flow_frame(fb, continous_frames, no_noise);

            //make_video_from_png((boost::filesystem::path)KITTI_FLOW_DATASET_PATH, "data/stereo_flow/image_02/");
            //make_video_from_png((boost::filesystem::path)KITTI_RAW_DATASET_PATH,"data/2011_09_28_drive_0016_sync/image_02/data/");
            //make_video_from_png((boost::filesystem::path)CPP_DATASET_PATH, "data/stereo_flow/image_02/");
        }
    }

    std::cout << "End of Program\n";
}

