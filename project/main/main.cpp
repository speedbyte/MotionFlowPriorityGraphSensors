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
#include <chrono>

#include "GridLayout.h"
#include "datasets.h"
#include "GroundTruthFlow.h"
#include "AlgorithmFlow.h"
#include "GroundTruthScene.h"
#include "GroundTruthObjects.h"
#include "RobustnessIndex.h"

#include "Sensors.h"
#include "Utils.h"


using namespace std::chrono;

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


    /*
    cv::Mat image = cv::imread("/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/results/stereo_flow/two/results_FB_none/stencil/000003_10_color_algo_3.png", cv::IMREAD_ANYCOLOR);

    cv::namedWindow("dd", CV_WINDOW_AUTOSIZE);
    cv::imshow("dd", image);
    cv::waitKey(0);

    cv::Mat image_mod = image.clone();
    std::cout << image_mod.channels();
    cv::Vec3b color = {255, 255, 255};
    cv::Vec3b color_edge = {255, 0, 0};
    for ( ushort x = 0; x < image_mod.cols; x++) {
        for ( ushort y = 0; y < image_mod.rows; y++) {
            if  (image_mod.at<cv::Vec3b>(y,x)[0] == 0 && image_mod.at<cv::Vec3b>(y,x)[1] == 0 && image_mod.at<cv::Vec3b>(y,x)[2] == 0) {
                image_mod.at<cv::Vec3b>(y,x) = color;
            }
            else {
                //image_mod.at<cv::Vec3b>(y,x) = color_edge;
            }
        }
    }
    cv::namedWindow("dd", CV_WINDOW_AUTOSIZE);
    cv::imshow("dd", image_mod);
    cv::imwrite("/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/results/stereo_flow/two/results_FB_none/stencil/000003_10_color_algo_3_mod.png", image_mod);
    cv::waitKey(0);
    return  0;
    */


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
        ushort start;
        ushort stop;
        std::string path;
        bool execute;
        bool gt;
        bool analyse;
        bool video;
        std::map<std::string, bool> dataprocessing_map;
        std::map<std::string, ushort> algorithm_map;
    } CONFIG_FILE_DATA;


    CONFIG_FILE_DATA
            cpp_dataset,
            matlab_dataset,
            kitti_flow_dataset,
            vkitti_flow_dataset,
            kitti_raw_dataset,
            vkitti_raw_dataset,
            vires_dataset,
            radar_dataset;

    cv::FileStorage fs_configfile;
    fs_configfile.open("../input.yml", cv::FileStorage::READ);
    if (!fs_configfile.isOpened()) {
        cerr << "Failed to open " << endl;
        return 1;
    }

    std::map<std::string, CONFIG_FILE_DATA *> map_object;

    map_object["CPP_DATASET"] = &cpp_dataset;
    map_object["MATLAB_DATASET"] = &matlab_dataset;
    map_object["KITTI_FLOW_DATASET"] = &kitti_flow_dataset;
    map_object["KITTI_RAW_DATASET"] = &kitti_raw_dataset;
    map_object["VKITTI_FLOW_DATASET"] = &vkitti_flow_dataset;
    map_object["VKITTI_RAW_DATASET"] = &vkitti_raw_dataset;
    map_object["RADAR_DATASET"] = &radar_dataset;
    map_object["VIRES_DATASET"] = &vires_dataset;

    cv::FileNode node;
    for (std::map<std::string, CONFIG_FILE_DATA *>::iterator it = map_object.begin(); it != map_object.end(); ++it) {

        std::cout << it->first << '\n';
        node = fs_configfile[(*it).first];
        if (node.isNone() || node.empty()) {
            std::cout << (*it).first << " cannot be found" << std::endl;
        }
        it->second->start = (int) node["START"];
        it->second->stop = (int) node["STOP"];
        it->second->path = node["PATH"].string();
        it->second->execute = (bool) (int) node["EXECUTE"];
        it->second->gt = (bool) (int) (node["GT"]);
        it->second->analyse = (bool) (int) (node["ANALYSE"]);
        it->second->video = (bool) (int) (node["VIDEO"]);

        for ( cv::FileNodeIterator iterator_subNode = node.begin(); iterator_subNode!= node.end(); iterator_subNode++) {

            if ((*iterator_subNode).name() == "DATAPROCESSING") {
                it->second->dataprocessing_map["NoAlgorithm"] = (bool)(int) (*iterator_subNode)["NoAlgorithm"];
                it->second->dataprocessing_map["SimpleAverage"] = (bool) (int) (*iterator_subNode)["SimpleAverage"];
                it->second->dataprocessing_map["MovingAverage"] = (bool) (int) (*iterator_subNode)["MovingAverage"];
                it->second->dataprocessing_map["VotedMean"] = (bool) (int) (*iterator_subNode)["VotedMean"];
                it->second->dataprocessing_map["RankedMean"] = (bool) (int) (*iterator_subNode)["RankedMean"];
            }
            else if ((*iterator_subNode).name() == "ALGORITHMS") {
                it->second->algorithm_map["FB"] = (ushort) (int) (*iterator_subNode)["FB"];
                it->second->algorithm_map["LK"] = (ushort) (int) (*iterator_subNode)["LK"];
                it->second->algorithm_map["SF"] = (ushort) (int) (*iterator_subNode)["SF"];
            }
        }
         //map_input_txt_to_main.push_back(map_object);
        std::cout << it->second->path << " " << it->second->execute << " " << it->second->gt << std::endl;

    }


/*

#define ITERATION_START_POINT 27
#define ITERATION_END_POINT 39
#define MAX_ITERATION_RESULTS (ITERATION_END_POINT - ITERATION_START_POINT) // 60 generate result. this cannot be more than vector

#define MAX_ITERATION_DATASET MAX_ITERATION_RESULTS // 60 generate result. this cannot be more than vector
#define MAX_ITERATION_GT_SCENE_GENERATION_DATASET (MAX_ITERATION_DATASET)*IMAGE_SKIP_FACTOR_DYNAMIC + IMAGE_SKIP_FACTOR_DYNAMIC*20  // generate always twenty images more than required.
#define IMAGE_SKIP_FACTOR_DYNAMIC 1  // 10

 */

    ushort ITERATION_START_POINT;
    ushort ITERATION_END_POINT;
    ushort MAX_ITERATION_RESULTS;
    ushort MAX_ITERATION_DATASET;
    ushort MAX_ITERATION_GT_SCENE_GENERATION_DATASET;

    /*
D     * novel real-to-virtual cloning method. Photo realistic synthetic dataaset consisting of 50 high resolution monocular
     * videos ( 21260 frames ) for five different virtual worlds in urban settings under different imaging and weather c
     * conditions.
     */
    if (vkitti_raw_dataset.execute) {

    }

    if (radar_dataset.execute) {

    }

    if (kitti_raw_dataset.execute) {
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

    std::map<std::string, double> time_map;


    std::vector<float> time;
    double sum_time = 0;

    ushort depth = CV_8U;
    ushort cn = 3;
    //assert(MAX_ITERATION_RESULTS <= MAX_ITERATION_GT_SCENE_GENERATION_DATASET);

    const std::vector<std::string> scenarios_list = {"two"};
    //const std::vector < std::string> environment_list = {"blue_sky", "light_snow", "rain_low"};
    //std::vector < std::string> environment_list = {"blue_sky", "night"};
    //const std::vector < std::string> environment_list = {"blue_sky", "light_snow", "mild_snow", "heavy_snow"};
    //const std::vector<std::string> environment_list = {"blue_sky", "heavy_snow"};
    const std::vector<std::string> environment_list = {"blue_sky"};

    auto tic_all = steady_clock::now();
    auto tic = steady_clock::now();

    cv::FileStorage fs;

    std::vector<GroundTruthObjects> list_of_gt_objects_base;
    std::vector<Sensors> list_of_gt_sensors_base;

    std::vector<Objects *> ptr_list_of_gt_objects_base;
    std::vector<std::unique_ptr<Objects>> ptr_list_of_simulated_objects_base;

    PixelRobustness pixelRobustness(fs);
    VectorRobustness vectorRobustness(fs);
    SensorFusionRobustness sensorFusionRobustness(fs);

    std::vector<ushort> generation_list, evaluation_list;

    for (ushort env_index = 0; env_index < environment_list.size(); env_index++) {

        if (cpp_dataset.execute || vires_dataset.execute) {

            std::vector<GroundTruthObjects> list_of_gt_objects;
            std::vector<Objects *> ptr_list_of_gt_objects;
            GroundTruthObjects::groundTruthObjectTotalCount = 0;

            cv::Size_<unsigned> frame_size(IMAGE_WIDTH, IMAGE_HEIGHT);
            std::string input = "data/stereo_flow/" + scenarios_list[0] + "/";
            std::string output = "results/stereo_flow/" + scenarios_list[0] + "/";

            GroundTruthScene *base_ptr_gt_scene;

            generation_list = {0};
            evaluation_list = {0};

            if (vires_dataset.execute) {

                Dataset::fillDataset(frame_size, depth, cn, VIRES_DATASET_PATH, input, output, vires_dataset.start, vires_dataset.stop, vires_dataset.dataprocessing_map,
                vires_dataset.algorithm_map);
                // The first iteration "blue_sky" will fil the objects_base and the ptr_objects_base and thereafter it is simply visible
                // through out the life cycle of the program.


                GroundTruthSceneExternal gt_scene(generation_list, evaluation_list, scenarios_list[0], environment_list[env_index],
                                                  list_of_gt_objects_base, list_of_gt_sensors_base,
                                                  vires_dataset.gt);
                base_ptr_gt_scene = &gt_scene;
                for ( ushort sensor_group_index = 0; sensor_group_index < generation_list.size(); sensor_group_index++ ) {
                    base_ptr_gt_scene->prepare_directories(generation_list.at(sensor_group_index));
                }
                base_ptr_gt_scene->generate_gt_scene();
                base_ptr_gt_scene->generate_bird_view();

                if ((env_index == environment_list.size() - 1) && vires_dataset.gt) {
                    //base_ptr_gt_scene->stopSimulation();
                    // Hack the images and the position_file
                    //system("python ../quicky_1.py 1");
                    exit(0);
                }

            } else if (cpp_dataset.execute) {

                Dataset::fillDataset(frame_size, depth, cn, CPP_DATASET_PATH, input, output, cpp_dataset.start, cpp_dataset.stop, cpp_dataset.dataprocessing_map, cpp_dataset.algorithm_map);

                GroundTruthSceneInternal gt_scene(generation_list, evaluation_list, scenarios_list[0], environment_list[env_index],
                                                  list_of_gt_objects_base, list_of_gt_sensors_base, cpp_dataset.gt);
                base_ptr_gt_scene = &gt_scene;
                for ( ushort sensor_group_index = 0; sensor_group_index < generation_list.size(); sensor_group_index++ ) {
                    base_ptr_gt_scene->prepare_directories(generation_list.at(sensor_group_index));
                }

                base_ptr_gt_scene->generate_gt_scene();
                base_ptr_gt_scene->generate_bird_view();
                if (cpp_dataset.gt) {
                    exit(0);
                }

            }

            if (environment_list[env_index] == "blue_sky") {
                for (auto obj_index = 0; obj_index < list_of_gt_objects_base.size(); obj_index++) {
                    ptr_list_of_gt_objects_base.push_back(&(list_of_gt_objects_base.at(obj_index)));
                }
            }

            list_of_gt_objects = list_of_gt_objects_base;
            ptr_list_of_gt_objects = ptr_list_of_gt_objects_base;

            GroundTruthFlow gt_flow(evaluation_list, environ[env_index], ptr_list_of_gt_objects_base, ptr_list_of_simulated_objects_base, ptr_list_of_gt_objects_base);

            // Generate Groundtruth data flow --------------------------------------
            if (environment_list[env_index] == "blue_sky" && !vires_dataset.gt) {

// in case i want to store values.yml in the groundtruth path, otherwise store simply in the project path
//                    fs.open((Dataset::m_dataset_gtpath.string() + "/values.yml"), cv::FileStorage::WRITE);
                fs.open(("../values.yml"), cv::FileStorage::WRITE);

                /// the following snippet prepares the ground truth edge, depth etc.
                gt_flow.prepare_directories((ushort)(evaluation_list.size() + 1%evaluation_list.size()), "", 0, 0);
                gt_flow.generate_edge_images((ushort)(evaluation_list.size() + 1%evaluation_list.size()));
                gt_flow.generate_depth_images((ushort)(evaluation_list.size() + 1%evaluation_list.size()));

                /// the following snippet generates mean centroid displacement for various data processing algorithms
                gt_flow.generate_flow_vector((ushort)(evaluation_list.size() + 1%evaluation_list.size()));
                //gt_flow.save_flow_vector((ushort)(evaluation_list.size() + 1%evaluation_list.size()));
                gt_flow.find_ground_truth_flow_occlusion_boundary((ushort)(evaluation_list.size() + 1%evaluation_list.size()));

                for (ushort obj_index = 0; obj_index < list_of_gt_objects_base.size(); obj_index++) {
                    ptr_list_of_gt_objects.at(obj_index)->generate_object_mean_centroid_displacement((ushort)(evaluation_list.size() + 1%evaluation_list.size()),
                            "ground_truth");
                }

                gt_flow.generate_collision_points((ushort)(evaluation_list.size() + 1%evaluation_list.size()));
                gt_flow.generate_metrics_optical_flow_algorithm((ushort)(evaluation_list.size() + 1%evaluation_list.size())); // this is to just create Jaccard Index  =  1
                //gt_flow.analyse_stencil();

            }

            if (environment_list[env_index] == "blue_sky" && !vires_dataset.gt) {

                time_map["groundtruth_flow"] = duration_cast<milliseconds>(steady_clock::now() - tic).count();
                tic = steady_clock::now();


                std::vector<AlgorithmFlow> dummy;

                if ((cpp_dataset.analyse && cpp_dataset.execute) || (vires_dataset.analyse && vires_dataset.execute)) {

                    pixelRobustness.generatePixelRobustness((ushort)(evaluation_list.size()+1%evaluation_list.size()), gt_flow, gt_flow);
                    vectorRobustness.generateVectorRobustness((ushort)(evaluation_list.size() + 1%evaluation_list.size()), gt_flow, gt_flow);
                }

                time_map["robustness_gt_flow"] = duration_cast<milliseconds>(steady_clock::now() - tic).count();
                tic = steady_clock::now();

            }
        }
    }


    ushort fps = 30;

    for (ushort algorithm_index = 0; algorithm_index < Dataset::m_algorithm_map.size() ; algorithm_index++) {

        std::vector<std::unique_ptr<AlgorithmFlow>> list_of_ptr_of_environment_OFalgorithm;
        std::map<std::string, std::unique_ptr<AlgorithmFlow>> map_string_to_OFalgorithm;

        for (ushort stepSize = 1; stepSize <= 1; stepSize += 4) {
            ptr_list_of_simulated_objects_base.clear();
            std::vector<SimulatedObjects> list_of_simulated_objects_base;
            // Generate Algorithm data flow --------------------------------------
            for (ushort env_index = 0; env_index < environment_list.size(); env_index++) {

                std::vector<Objects *> ptr_list_of_simulated_objects;
                bool found = false;

                map_string_to_OFalgorithm["LK"] = std::make_unique<LukasKanade>(evaluation_list, environment_list[env_index], lk, "lk", ptr_list_of_gt_objects_base, ptr_list_of_simulated_objects_base, ptr_list_of_simulated_objects, stepSize);
                map_string_to_OFalgorithm["FB"] = std::make_unique<Farneback>(evaluation_list, environment_list[env_index], fb, "fback", ptr_list_of_gt_objects_base, ptr_list_of_simulated_objects_base, ptr_list_of_simulated_objects, stepSize);
                map_string_to_OFalgorithm["SF"] = std::make_unique<Farneback>(evaluation_list, environment_list[env_index], fb, "simple_flow", ptr_list_of_gt_objects_base, ptr_list_of_simulated_objects_base, ptr_list_of_simulated_objects, stepSize);

                if ( Dataset::m_algorithm_map["LK"]) {

                    list_of_ptr_of_environment_OFalgorithm.push_back(std::move(map_string_to_OFalgorithm["LK"]));
                    found = true;
                    if ( env_index == (environment_list.size()-1)) {
                        Dataset::m_algorithm_map["LK"] = 0;
                    }

                }
                else if ( Dataset::m_algorithm_map["FB"] ) {

                    list_of_ptr_of_environment_OFalgorithm.push_back(std::move(map_string_to_OFalgorithm["FB"]));
                    found = true;
                    if ( env_index == (environment_list.size()-1)) {
                        Dataset::m_algorithm_map["FB"] = 0;
                    }
                }

                else if ( Dataset::m_algorithm_map["SF"]) {

                    //The Simple Flow algorithm attempts to establish a local flow vector for each point that best explains the motion of the neighborhood around that point. It does this by computing the (integer) flow vector that optimizes an energy function. his energy function is essentially a sum over terms for each pixel in the neighborhood in which the energy grows quadratically with the difference between the intensities of the pixel in the neighborhood at time t and the corresponding pixel (i.e., displaced by the flow vector) at time t + 1.
                    list_of_ptr_of_environment_OFalgorithm.push_back(std::move(map_string_to_OFalgorithm["SF"]));
                    found = true;
                    if ( env_index == (environment_list.size()-1)) {
                        Dataset::m_algorithm_map["SF"] = 0;
                    }

                }

                if ( !found ) {
                    break;
                }

                std::vector<SimulatedObjects> list_of_simulated_objects;
                // just to be sure, all lists are empty
                list_of_simulated_objects.clear();
                ptr_list_of_simulated_objects.clear();

                SimulatedObjects::simulatedObjectTotalCount = 0; // start from 0 for each list_of_algorithm

                for (ushort obj_index = 0; obj_index < list_of_gt_objects_base.size(); obj_index++) {
                    //two objects
                    std::vector<std::vector<bool> > extrapolated_visibility = list_of_gt_objects_base.at(
                            obj_index).get_object_extrapolated_visibility();

                    SimulatedObjects objects(
                            "simulated_" + list_of_gt_objects_base.at(obj_index).getObjectName(), extrapolated_visibility);
                    list_of_simulated_objects.push_back(objects);  // mke new objects
                }

                // push the objects into the pointer. The pointer here will contain two elements.
                for (auto obj_index = 0; obj_index < list_of_simulated_objects.size(); obj_index++) {
                    ptr_list_of_simulated_objects.push_back(&list_of_simulated_objects.at(obj_index));
                }

                list_of_ptr_of_environment_OFalgorithm[env_index]->prepare_directories((ushort)(evaluation_list.size() + 1%evaluation_list.size()), environment_list[env_index], fps, stepSize);
                if ((Dataset::m_execute_algorithm && cpp_dataset.execute) || (Dataset::m_execute_algorithm && vires_dataset.execute)) {

                    list_of_ptr_of_environment_OFalgorithm[env_index]->prepare_directories((ushort)(evaluation_list.size() + 1%evaluation_list.size()), environment_list[env_index], fps, stepSize);
                    // TODO - do something for stepSize.. its redundant here.
                    /// run optical flow algorithm
                    list_of_ptr_of_environment_OFalgorithm[env_index]->run_optical_flow_algorithm(evaluation_list, video_frames, fps);
                    /// combine sensor data
                    if ( (evaluation_list.size() + 1%evaluation_list.size()) > 1 ) {
                        list_of_ptr_of_environment_OFalgorithm[env_index]->combine_sensor_data();
                    }
                    if (environment_list[env_index] == "blue_sky") { // store the stimulated objects from the ground run.
                        for (auto obj_index = 0; obj_index < list_of_simulated_objects.size(); obj_index++) {
                            list_of_simulated_objects_base.push_back(list_of_simulated_objects.at(obj_index));
                            ptr_list_of_simulated_objects_base.push_back(std::make_unique<SimulatedObjects>((list_of_simulated_objects_base.at(obj_index))));
                            assert(ptr_list_of_simulated_objects_base.at(obj_index)->getObjectId() == obj_index);
                        }
                    }
                    for (auto obj_index = 0; obj_index < list_of_simulated_objects.size(); obj_index++) {
                        assert(list_of_simulated_objects_base.at(obj_index).getObjectId() == obj_index);
                        assert(ptr_list_of_simulated_objects_base.at(obj_index)->getObjectId() == obj_index);
                    }

                    /// generate and save flow vector
                    for (ushort i = 0; i < list_of_simulated_objects.size(); i++) {
                        list_of_simulated_objects.at(i).generate_object_mean_centroid_displacement((ushort)(evaluation_list.size() + 1%evaluation_list.size()), "algorithm");
                    }
                    //list_of_ptr_of_environment_OFalgorithm[env_index]->save_flow_vector((ushort)(evaluation_list.size() + 1%evaluation_list.size()));

                    /// analysis and metrics
                    list_of_ptr_of_environment_OFalgorithm[env_index]->generate_collision_points((ushort)(evaluation_list.size() + 1%evaluation_list.size()));
                    //list_of_ptr_of_environment_OFalgorithm[env_index]->analyse_stencil();
                    list_of_ptr_of_environment_OFalgorithm[env_index]->generate_metrics_optical_flow_algorithm((ushort)(evaluation_list.size() + 1%evaluation_list.size()));

                }

                auto position = list_of_ptr_of_environment_OFalgorithm.at(env_index)->getResultOrdner().find('/');
                std::string suffix = list_of_ptr_of_environment_OFalgorithm.at(env_index)->getResultOrdner().replace(position, 1, "_");
                time_map["algorithm_flow_" + suffix] = (duration_cast<milliseconds>( steady_clock::now() - tic).count());
                tic = steady_clock::now();

                if ((Dataset::m_execute_algorithm && cpp_dataset.analyse && cpp_dataset.execute) || (Dataset::m_execute_algorithm && vires_dataset.analyse && vires_dataset.execute)) {

                    pixelRobustness.generatePixelRobustness((ushort)(evaluation_list.size() + 1%evaluation_list.size()), *list_of_ptr_of_environment_OFalgorithm[0], *list_of_ptr_of_environment_OFalgorithm[env_index]);
                    vectorRobustness.generateVectorRobustness((ushort)(evaluation_list.size() + 1%evaluation_list.size()), *list_of_ptr_of_environment_OFalgorithm[env_index], *list_of_ptr_of_environment_OFalgorithm[0]);

                    time_map["robustness_" + suffix] = (duration_cast<milliseconds>(steady_clock::now() - tic).count());
                    tic = steady_clock::now();

                }
            }

            //ffmpeg -framerate 10 -pattern_type glob -i '*.png' -r 30 -pix_fmt yuv420p movement.avi

            if ((cpp_dataset.video && cpp_dataset.execute) || (vires_dataset.video && vires_dataset.execute)) {
                for (ushort env_index = 0; env_index < environment_list.size(); env_index++) {
                    for (int sensors = 0; sensors < (ushort)(evaluation_list.size() + 1%evaluation_list.size()); sensors++) {
                        Utils::make_video_from_regex(
                                Dataset::m_dataset_gtpath.string() + '/' + environment_list[env_index] + '_' +
                                std::to_string(sensors));
                        Utils::make_video_from_regex(
                                Dataset::m_dataset_resultpath.string() + "/results_FB_" + environment_list[env_index] +
                                '_' + std::to_string(fps) + "_" + std::to_string(stepSize) + "/position_occ_0" +
                                std::to_string(sensors) + '/');
                    }
                }
            }
        }

        //system("python ../../main_python/motionflow_graphs.py");

    }

    fs << "time_map" << "[";
    int total = 0;
    for (auto &n : time_map) {
        fs << "{:" << n.first << n.second << "}";
        std::cout << n.first << " " << n.second << std::endl;
        total += n.second;
    }

    time_map["total"] = duration_cast<milliseconds>(steady_clock::now() - tic_all).count();
    fs << "{:" << "total" << time_map["total"] << "}";
    fs << "]";
    std::cout << "unaccounted time = " << time_map["total"] - total << std::endl;

    fs.release();


    /* MATLAB_DATASET ------------- */

    {
        if ( matlab_dataset.execute ) {

            cv::Size_<unsigned> frame_size(1242, 375);
            std::string input = "data/stereo_flow/image_02";
            //Dataset::fillDataset(frame_size, depth, cn, MATLAB_DATASET_PATH, input, "results", matlab_dataset.start, matlab_dataset.stop);
            //AlgorithmFlow algo();

        }
    }

/* KITTI_FLOW_DATASET------------- */

    {
        cv::Size_<unsigned> frame_size(IMAGE_WIDTH, IMAGE_HEIGHT);
        std::string input = "data/stereo_flow/" + scenarios_list[0] + "/";
        std::string output = "results/stereo_flow/" + scenarios_list[0] + "/";

        if ( kitti_flow_dataset.execute ) {

            //Dataset::fillDataset(frame_size, depth, cn, VIRES_DATASET_PATH, input, output, kitti_flow_dataset.start, kitti_flow_dataset.stop);

            std::vector<Objects *> ptr_list_of_gt_objects_base;
            std::vector<std::unique_ptr<Objects>> ptr_list_of_simulated_objects_base;
            std::vector<Objects *> ptr_list_of_simulated_objects;

            Farneback fback(evaluation_list, "blue_sky", fb, "fback", ptr_list_of_gt_objects_base, ptr_list_of_simulated_objects_base, ptr_list_of_simulated_objects, 1);
            AlgorithmFlow *algo = &fback;
            // The ground truth generate_flow_frame and image is already available from the base dataset. Hence only results can be
            // calculated here.

            algo->run_optical_flow_algorithm(evaluation_list, video_frames, 30);

            //make_video_from_regex((boost::filesystem::path)KITTI_FLOW_DATASET_PATH, "data/stereo_flow/image_02/");
            //make_video_from_regex((boost::filesystem::path)KITTI_RAW_DATASET_PATH,"data/2011_09_28_drive_0016_sync/image_02/data/");
            //make_video_from_regex((boost::filesystem::path)CPP_DATASET_PATH, "data/stereo_flow/image_02/");
        }
    }

    std::cout << "End of Program\n";
}

