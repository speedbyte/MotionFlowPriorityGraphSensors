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
#include "Dataset.h"
#include "GroundTruthFlow.h"
#include "AlgorithmFlow.h"
#include "GenerateGroundTruthScene.h"
#include "GroundTruthObjects.h"
#include "RobustnessIndex.h"

#include "Sensors.h"
#include "Utils.h"

//TODO

// all python plot data first and then mix them as per requirement
// stich images python plot automatically - DONE
// quantify noise. rain or static noise is not enough. need to know how many original pixels are corrputed by this noise.
// interpolate using splash mechanism ( bilateral filter )
// class interpolate Data
// why does vires generates one image more than the ground truth information. That means the last ground truth informtion is bogus. Hence while reading the ground truth, the max should be
// frame_count in position file - 1.

//DONE
// check if depth is correct - depth is taken from the last value. the sensor position does not matter for the depth image. it is always taken from the middle of the rear axle.
// find values of algorithm displacement at sroi - DONE.
// quality of value of algorithm displacement at sroi - DONE
// why is cumulative error of sroi more than eroi???? - DONE
// gnuplot show interpolated values - DONE
// why are there two signs in one gnuplot - DONE, by creating a legend
// stich images - DONE
// masking problem. cpp and vires - i wasnt converting RGB to BGR and hence the problem arised.
// fix sroi analysis l1, l2 and ma cumulative error and total pixels - added gt_displacement when gt_sroi is created
// parameter-extended : total algo sroi pixels / total algo pixels AND total algo sroi pixel error / total algo pixel error - DOME
// why is stencil size in LK 0, why cant we start FB and LK at the same time? - DONE. added further variables. Now all algorithm will start

// Presentation:
// the gradient matters. If the error is in the same direction, then its fine. but if the error is distributed, then its a problem
// So, what is the error covariance matrix?
// How reliable is the data - compare L2_good_pixel and MA_good_pixel error. The lesser the distance the better it is.
// Evaluation models has Set of aögorithms - OF, Inter, Dataprocessing and lasstly Sensor Fusion.
// Further it induces quantified noise
// Analysis - Health of the algorithms by analysing overall pixels and pixels at SROI
// Visual representation of the analysis of a single frame
// All frames using graphs


// Frame number 37 has a peak, although the objects arent so close to each other. SROI is less, but the pixels in good pixel region are at the boundary.
// Total pixels in other frames are much higer



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
    //#define CV_MAKETYPE(depth,cn) (CV_MAT_DEPTH(depth) + (((cn)-1) << CV_CN_SHIFT))
    ushort channels = image_mod.channels();   // how many channels
    ushort depth = image_mod.depth();  // if CV_8U or CV_16S or CV_32F
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
        ushort max_frames_dataset;
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

        node = fs_configfile[(*it).first];
        if (node.isNone() || node.empty()) {
            std::cout << (*it).first << " cannot be found" << std::endl;
            continue;
        }
        std::cout << node.name() << '\n';

        for ( cv::FileNodeIterator iterator_subNode = node.begin(); iterator_subNode!= node.end(); iterator_subNode++) {

            if ((*iterator_subNode).name() == "START") {
                it->second->start = (int) node[(*iterator_subNode).name()];
            }
            else if ((*iterator_subNode).name() == "STOP") {
                it->second->stop = (int) node[(*iterator_subNode).name()];
            }
            else if ((*iterator_subNode).name() == "MAX_GENERATION_DATASET") {
                it->second->max_frames_dataset = (int) node[(*iterator_subNode).name()];
            }
            else if ((*iterator_subNode).name() == "PATH") {
                it->second->path = node[(*iterator_subNode).name()].string();
            }
            else if ((*iterator_subNode).name() == "EXECUTE") {
                it->second->execute = (bool) (int) node[(*iterator_subNode).name()];
            }
            else if ((*iterator_subNode).name() == "GT") {
                it->second->gt = (bool) (int) (node[(*iterator_subNode).name()]);
            }
            else if ((*iterator_subNode).name() == "ANALYSE") {
                it->second->analyse = (bool) (int) (node[(*iterator_subNode).name()]);
            }
            else if ((*iterator_subNode).name() == "VIDEO") {
                it->second->video = (bool) (int) (node[(*iterator_subNode).name()]);
            }
            else if ((*iterator_subNode).name() == "DATAPROCESSING") {
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
                it->second->algorithm_map["TVL"] = (ushort) (int) (*iterator_subNode)["TVL"];
            }
        }
        // map_input_txt_to_main.push_back(map_object);
        std::cout << it->second->path << " " << it->second->execute << " " << it->second->gt << std::endl;

    }

    ushort ITERATION_START_POINT;
    ushort ITERATION_END_POINT;
    ushort MAX_ITERATION_RESULTS;
    ushort MAX_GENERATION_DATASET;
    ushort MAX_ITERATION_GT_SCENE_GENERATION_DATASET;

    /*
D     * novel real-to-virtual cloning method. Photo realistic synthetic dataaset consisting of 50 high resolution monocular
     * videos ( 21260 frames ) for five different virtual worlds in urban settings under different imaging and noise c
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
    //const std::vector < std::string> noise_list = {"blue_sky", "light_snow", "rain_low"};
    //std::vector < std::string> noise_list = {"blue_sky", "night"};
    //const std::vector < std::string> noise_list = {"blue_sky", "light_snow", "mild_snow", "heavy_snow"};
    const std::vector<std::string> noise_list = {"blue_sky", "heavy_snow"};
    //const std::vector<std::string> noise_list = {"blue_sky"};

    auto tic_all = steady_clock::now();
    auto tic = steady_clock::now();

    cv::FileStorage fs_ground_truth;
    cv::FileStorage fs_algorithm;

    std::vector<GroundTruthObjects> list_of_gt_objects_base;
    std::vector<Sensors> list_of_gt_sensors_base;

    std::vector<GroundTruthObjects *> ptr_list_of_gt_objects_base;

    PixelRobustness pixelRobustness;
    VectorRobustness vectorRobustness;
    SensorFusionRobustness sensorFusionRobustness;

    const std::vector<ushort> generation_list = {0}, evaluation_list = {0};

    cv::Size_<unsigned> frame_size(IMAGE_WIDTH, IMAGE_HEIGHT);
    std::string input = "data/stereo_flow/" + scenarios_list[0] + "/";
    std::string output = "results/stereo_flow/" + scenarios_list[0] + "/";

    bool LK = true;
    bool FB = true;
    bool SF = true;
    bool TVL = true;

    if (cpp_dataset.execute || vires_dataset.execute) {

        if (vires_dataset.execute) {

            Dataset::fillDataset(frame_size, depth, cn, VIRES_DATASET_PATH, input, output, vires_dataset.gt,
                                 vires_dataset.start, vires_dataset.stop, vires_dataset.max_frames_dataset,
                                 vires_dataset.dataprocessing_map, vires_dataset.algorithm_map, evaluation_list);

        } else if ( cpp_dataset.execute ) {

            Dataset::fillDataset(frame_size, depth, cn, CPP_DATASET_PATH, input, output, cpp_dataset.gt,
                                 cpp_dataset.start, cpp_dataset.stop, cpp_dataset.max_frames_dataset,
                                 cpp_dataset.dataprocessing_map, cpp_dataset.algorithm_map, evaluation_list);
        }


        std::shared_ptr<GroundTruthFlow> ptr_gt_flow;

        for (ushort noise_index = 0; noise_index < noise_list.size(); noise_index++) {  // blue_sky, heavy_snow

            std::unique_ptr<GroundTruthScene> base_ptr_gt_scene;
            std::unique_ptr<Noise> noisePointer;
            if (vires_dataset.execute) {

                base_ptr_gt_scene = std::make_unique<GroundTruthSceneExternal>(generation_list, evaluation_list, scenarios_list[0], noise_list[noise_index]);

            } else if (cpp_dataset.execute) {

                base_ptr_gt_scene = std::make_unique<GroundTruthSceneInternal>(generation_list, evaluation_list, scenarios_list[0], noise_list[noise_index]);
                noisePointer = std::make_unique<ColorfulNoise>();

            }

            for ( ushort sensor_group_index = 0; sensor_group_index < generation_list.size(); sensor_group_index++ ) {
                if (noise_list[noise_index] == "blue_sky" ) {

                    base_ptr_gt_scene->prepare_scene_directories_others(generation_list.at(sensor_group_index));
                    base_ptr_gt_scene->prepare_scene_directories_blue_sky(generation_list.at(sensor_group_index));

                } else {

                    base_ptr_gt_scene->prepare_scene_directories_others(generation_list.at(sensor_group_index));
                }
                noisePointer = std::make_unique<NoNoise>();
            }

            if ( Dataset::GENERATE ) {
                base_ptr_gt_scene->generate_gt_scene();
                base_ptr_gt_scene->generate_bird_view();
            }

            // Generate Groundtruth data flow --------------------------------------
            if (noise_list[noise_index] == "blue_sky" ) {

                if ( Dataset::GENERATE ) {
                    base_ptr_gt_scene->write_gt_scene_data();  // writePositionInYAML; framdifference, edge images
                } else {
                    base_ptr_gt_scene->read_gt_scene_data();
                }
                base_ptr_gt_scene->convert_sensor_image_to_object_level(noisePointer, list_of_gt_objects_base, list_of_gt_sensors_base); // transfer data to list_of_objects.
                // starting here,  the job of GroundTruthScene is complete

                // The first iteration "blue_sky" will fil the objects_base and the ptr_objects_base and thereafter it is simply visible
                // through out the life cycle of the program.
                for (auto obj_index = 0; obj_index < list_of_gt_objects_base.size(); obj_index++) {
                    ptr_list_of_gt_objects_base.push_back(&(list_of_gt_objects_base.at(obj_index)));
                }

                GroundTruthFlow gt_flow(evaluation_list, environ[noise_index], list_of_gt_sensors_base,
                                           ptr_list_of_gt_objects_base);

                // in case i want to store values.yml in the groundtruth path, otherwise store simply in the project path
                // fs.open((Dataset::m_dataset_gtpath.string() + "/values.yml"), cv::FileStorage::WRITE);
                /// the following snippet prepares the ground truth edge, depth etc.
                gt_flow.prepare_groundtruth_flow_directories("blue_sky", 0, 0);
                /// the following snippet generates mean centroid displacement for various data processing algorithms
                gt_flow.generate_flow_vector();

                if ( Dataset::GENERATE ) {
                    gt_flow.save_flow_vector();
                } else {

                    gt_flow.rerun_optical_flow_algorithm_interpolated();
                    gt_flow.find_ground_truth_object_special_region_of_interest();
                    fs_ground_truth.open((gt_flow.getGeneratePath() + std::string("/values_ground_truth.yml")) , cv::FileStorage::WRITE);

                    for (ushort obj_index = 0; obj_index < list_of_gt_objects_base.size(); obj_index++) {
                        ptr_list_of_gt_objects_base.at(obj_index)->generate_object_mean_centroid_displacement(
                                "ground_truth");
                    }

                    gt_flow.generate_collision_points();
                    gt_flow.generate_metrics_optical_flow_algorithm(); // this is to just create Jaccard Index  =  1

                    time_map["prepare_ground_truth"] = duration_cast<milliseconds>(steady_clock::now() - tic).count();
                    tic = steady_clock::now();

                    if ((cpp_dataset.analyse && cpp_dataset.execute) || (vires_dataset.analyse && vires_dataset.execute)) {

                        pixelRobustness.generatePixelRobustness(gt_flow, gt_flow, fs_ground_truth);
                        vectorRobustness.generateVectorRobustness( gt_flow, gt_flow, fs_ground_truth);
                    }

                    time_map["robustness_gt_flow"] = duration_cast<milliseconds>(steady_clock::now() - tic).count();
                    tic = steady_clock::now();
                }
                ptr_gt_flow = std::make_unique<GroundTruthFlow>(gt_flow);
            }
            else {

                if (Dataset::GENERATE) {

                }
            }

             if ((noise_index == noise_list.size() - 1) && Dataset::GENERATE) {
                //base_ptr_gt_scene->stopSimulation();
                // Hack the images and the position_file
                //system("python ../quicky_1.py 1");
                exit(0);
            }
        }

        ushort fps = 30;

        for (ushort algorithm_index = 0; algorithm_index < Dataset::m_algorithm_map.size() ; algorithm_index++) {
        //for (ushort algorithm_index = 0; algorithm_index < 1 ; algorithm_index++) {

            std::unique_ptr<AlgorithmFlow> ptr_OF_algorithm;

            for (ushort stepSize = 1; stepSize <= 1; stepSize += 4) {

                // the base changes when the ground truth with respect to this cycle changes.
                // If the step size changes, then it means that the ground truth also changes.
                std::vector<SimulatedObjects> list_of_simulated_objects_base;
                std::vector<SimulatedObjects *> ptr_list_of_simulated_objects_base;

                Dataset::m_algorithm_map = Dataset::m_algorithm_map_original;
                // Generate Algorithm data flow --------------------------------------
                for (ushort noise_index = 0; noise_index < noise_list.size(); noise_index++) {

                    std::vector<SimulatedObjects *> ptr_list_of_simulated_objects;
                    bool found = false;

                    //continue;
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
                                "simulated_" + list_of_gt_objects_base.at(obj_index).getObjectName(),
                                extrapolated_visibility);
                        list_of_simulated_objects.push_back(objects);  // mke new objects
                    }

                    // push the objects into the pointer. The pointer here will contain two elements.
                    for (auto obj_index = 0; obj_index < list_of_simulated_objects.size(); obj_index++) {
                        ptr_list_of_simulated_objects.push_back(&list_of_simulated_objects.at(obj_index));
                    }

                    std::string path;
                    if (Dataset::m_algorithm_map["LK"] && LK) {

                        ptr_OF_algorithm = std::make_unique<LukasKanade>(evaluation_list,
                                noise_list[noise_index], lk,
                                "LK", list_of_gt_sensors_base,
                                ptr_list_of_gt_objects_base,
                                ptr_list_of_simulated_objects_base,
                                ptr_list_of_simulated_objects,
                                stepSize, ptr_gt_flow);
                        found = true;
                        if (noise_index == (noise_list.size() - 1)) {
                            //reset. We are done wth this algorithm
                            LK = false;
                        }
                    } else if (Dataset::m_algorithm_map["FB"] && FB) {

                        ptr_OF_algorithm = std::make_unique<Farneback>(evaluation_list,
                                noise_list[noise_index], fb,
                                "FB", list_of_gt_sensors_base,
                                ptr_list_of_gt_objects_base,
                                ptr_list_of_simulated_objects_base,
                                ptr_list_of_simulated_objects,
                                stepSize, ptr_gt_flow);
                        found = true;
                        if (noise_index == (noise_list.size() - 1)) {
                            //reset. We are done wth this algorithm
                            FB = false;
                        }
                    } else if (Dataset::m_algorithm_map["TVL"] && TVL) {

                        ptr_OF_algorithm = std::make_unique<DualTVLFlow>(evaluation_list,
                                noise_list[noise_index], tvl,
                                "TVL",
                                list_of_gt_sensors_base,
                                ptr_list_of_gt_objects_base,
                                ptr_list_of_simulated_objects_base,
                                ptr_list_of_simulated_objects,
                                stepSize, ptr_gt_flow);
                        found = true;
                        if (noise_index == (noise_list.size() - 1)) {
                            //reset. We are done wth this algorithm
                            TVL = false;
                        }
                    } else if (Dataset::m_algorithm_map["SF"] && SF) {

                        //The Simple Flow algorithm attempts to establish a local flow vector for each point that best explains the motion of the neighborhood around that point. It does this by computing the (integer) flow vector that optimizes an energy function. his energy function is essentially a sum over terms for each pixel in the neighborhood in which the energy grows quadratically with the difference between the intensities of the pixel in the neighborhood at time t and the corresponding pixel (i.e., displaced by the flow vector) at time t + 1.
                        //ptr_OF_algorithm.push_back(std::move(pair_string_to_OFalgorithm["SF"]));
                        found = true;
                        if (noise_index == (noise_list.size() - 1)) {
                            //reset. We are done wth this algorithm
                            SF = false;
                        }
                    }

                    if (!found) {
                        break;
                    }


                    if ((Dataset::m_execute_algorithm && cpp_dataset.execute) ||
                        (Dataset::m_execute_algorithm && vires_dataset.execute)) {

                        ptr_OF_algorithm->prepare_algorithm_flow_directories(
                                noise_list[noise_index], fps, stepSize);

                        path = ptr_OF_algorithm->getGeneratePath() + std::string("/values_") +
                                ptr_OF_algorithm->getOpticalFlowName() + std::string(".yml");
                        fs_algorithm.open(path, cv::FileStorage::WRITE);

                        // TODO - do something for stepSize.. its redundant here.
                        /// run optical flow algorithm
                        ptr_OF_algorithm->run_optical_flow_algorithm(evaluation_list,
                                                                                                      video_frames,
                                                                                                      fps);
                        /// combine sensor data
                        if ((evaluation_list.size() + 1 % evaluation_list.size()) > 1) {
                            ptr_OF_algorithm->combine_sensor_data();
                        }
                        ptr_OF_algorithm->save_flow_vector();
                        ptr_OF_algorithm->rerun_optical_flow_algorithm_interpolated();
                        ptr_OF_algorithm->generate_sroi_intersections();

                        /// generate and save flow vector
                        for (ushort i = 0; i < list_of_simulated_objects.size(); i++) {
                            list_of_simulated_objects.at(i).generate_object_mean_centroid_displacement("algorithm");
                        }

                        /// analysis and metrics
                        ptr_OF_algorithm->generate_collision_points();
                        ptr_OF_algorithm->generate_metrics_optical_flow_algorithm();

                        /// stiching images
                        ptr_OF_algorithm->stich_gnuplots(); // gnuplots
                        //ptr_OF_algorithm[noise_index]->stich_gnuplots(); // optical flow algorithm interpolated data

                    }

                    auto position = ptr_OF_algorithm->getResultOrdner().find('/');
                    std::string suffix = ptr_OF_algorithm->getResultOrdner().replace(position, 1, "_");
                    time_map["algorithm_flow_" + suffix] = (duration_cast<milliseconds>(
                            steady_clock::now() - tic).count());
                    tic = steady_clock::now();

                    if ((Dataset::m_execute_algorithm && cpp_dataset.analyse && cpp_dataset.execute) ||
                        (Dataset::m_execute_algorithm && vires_dataset.analyse && vires_dataset.execute)) {

                        pixelRobustness.generatePixelRobustness(*ptr_OF_algorithm,
                                                                *ptr_OF_algorithm, fs_algorithm);
                        vectorRobustness.generateVectorRobustness(*ptr_OF_algorithm,
                                                                  *ptr_OF_algorithm, fs_algorithm);

                        time_map["robustness_" + suffix] = (duration_cast<milliseconds>(
                                steady_clock::now() - tic).count());
                        tic = steady_clock::now();

                    }

                    if (noise_list[noise_index] ==
                            "blue_sky") { // store the stimulated objects from the ground run.
                        list_of_simulated_objects_base = list_of_simulated_objects;
                        // push the objects into the pointer. The pointer here will contain two elements.
                        for (auto obj_index = 0; obj_index < list_of_simulated_objects.size(); obj_index++) {
                            ptr_list_of_simulated_objects_base.push_back(&list_of_simulated_objects_base.at(obj_index));
                        }
                    }
                    for (auto obj_index = 0; obj_index < list_of_simulated_objects.size(); obj_index++) {
                        assert(ptr_list_of_simulated_objects.at(obj_index)->getObjectId() == obj_index);
                        assert(ptr_list_of_simulated_objects_base.at(obj_index)->getObjectId() == obj_index);
                    }

                }

                //ffmpeg -framerate 10 -pattern_type glob -i '*.png' -r 30 -pix_fmt yuv420p movement.avi

                if ((cpp_dataset.video && cpp_dataset.execute) || (vires_dataset.video && vires_dataset.execute)) {
                    for (ushort noise_index = 0; noise_index < noise_list.size(); noise_index++) {
                        for (int sensors = 0; sensors < Dataset::SENSOR_COUNT; sensors++) {
                            Utils::make_video_from_regex(
                                    Dataset::m_dataset_gtpath.string() + '/' + noise_list[noise_index] + '_' +
                                    std::to_string(sensors));
                            Utils::make_video_from_regex(
                                    Dataset::m_dataset_resultpath.string() + "/results_FB_" +
                                    noise_list[noise_index] +
                                    '_' + std::to_string(fps) + "_" + std::to_string(stepSize) + "/position_occ_0" +
                                    std::to_string(sensors) + '/');
                        }
                    }
                }
            }
        }
        //system("python ../../main_python/motionflow_graphs.py");
    }

    fs_ground_truth << "time_map" << "[";
    int total = 0;
    for (auto &n : time_map) {
        fs_ground_truth << "{:" << n.first << n.second << "}";
        std::cout << n.first << " " << n.second << std::endl;
        total += n.second;
    }

    time_map["total"] = duration_cast<milliseconds>(steady_clock::now() - tic_all).count();
    fs_ground_truth << "{:" << "total" << time_map["total"] << "}";
    fs_ground_truth << "]";
    std::cout << "unaccounted time = " << time_map["total"] - total << std::endl;

    fs_ground_truth.release();
    fs_algorithm.release();


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

            std::shared_ptr<GroundTruthFlow> ptr_gt_flow;

            //Dataset::fillDataset(frame_size, depth, cn, VIRES_DATASET_PATH, input, output, kitti_flow_dataset.start, kitti_flow_dataset.stop);

            std::vector<GroundTruthObjects *> ptr_list_of_gt_objects_base;
            std::vector<SimulatedObjects *> ptr_list_of_simulated_objects_base;
            std::vector<SimulatedObjects *> ptr_list_of_simulated_objects;

            Farneback fback(evaluation_list, "blue_sky", fb, "fback", list_of_gt_sensors_base, ptr_list_of_gt_objects_base, ptr_list_of_simulated_objects_base, ptr_list_of_simulated_objects, 1, ptr_gt_flow);
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

