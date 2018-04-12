//
// Created by veikas on 27.01.18.
//

#include <iostream>
#include <boost/filesystem/operations.hpp>
#include <opencv2/core/mat.hpp>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <png++/rgb_pixel.hpp>
#include <png++/image.hpp>
#include <vires-interface/Common/viRDBIcd.h>
#include <sys/time.h>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv/cv.hpp>
#include "GroundTruthScene.h"
#include "kbhit.h"
#include "ViresObjects.h"
#include "ObjectMetaData.h"

using namespace std::chrono;

void GroundTruthScene::visualiseBoundingBox(void) {

    std::cout << "visualise boudning box at " << m_generatepath.string() + "stencil/" << std::endl;

    char file_name_image[50], file_name_image_output[50];

    cv::Mat image_data_and_shape;

    const ushort max_frame_skip = 1; // image is generated only once irrespective of skips.
    cv::Mat tempGroundTruthImage(Dataset::getFrameSize(), CV_8UC3);

    for (int frame_skip = 1; frame_skip <= max_frame_skip; frame_skip++) {

        for (ushort frame_count = 1; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_IMAGES; frame_count++)
        {
            //ushort frame_count = 3;

            std::cout << "frame_count " << frame_count << std::endl;

            std::string output_image_file_with_path;
            /*---------------------------------------------------------------------------------*/
            tempGroundTruthImage = cv::Scalar::all(255);

            sprintf(file_name_image, "000%03d_10.png", frame_count * frame_skip);
            std::string input_image_file_with_path = m_generatepath.string() + file_name_image;

            sprintf(file_name_image_output, "000%03d_10_bb.png", frame_count * frame_skip);
            output_image_file_with_path = m_generatepath.string() + "stencil/" + file_name_image_output;

            cv::Mat tempGroundTruthImageBase = cv::imread(input_image_file_with_path, CV_LOAD_IMAGE_ANYCOLOR);
            tempGroundTruthImage = cv::Scalar::all(255);
            tempGroundTruthImage = tempGroundTruthImageBase.clone();

            for (unsigned obj_index = 0; obj_index < m_list_gt_objects.size(); obj_index++) {

                if ((m_list_gt_objects.at(obj_index).get_obj_base_visibility().at(frame_count))
                        ) {

                    //cv::Rect boundingbox =  cv::Rect(cvRound(m_list_gt_objects.at(obj_index).get_obj_base_pixel_position_pixel_displacement().at(frame_count).first.x - (cvRound(m_list_gt_objects.at(obj_index).getExtrapolatedGroundTruthDetails().at(frame_count).m_object_dimensions_px.dim_length_m/2))),
                    cv::Rect boundingbox = cv::Rect(
                            cvRound(m_list_gt_objects.at(obj_index).getExtrapolatedGroundTruthDetails().at(
                                    frame_skip - 1).at(frame_count).m_object_location_px.location_x_m-
                                    cvRound(m_list_gt_objects.at(obj_index).getExtrapolatedGroundTruthDetails().at(
                                            frame_skip - 1).at(frame_count).m_object_dimensions_px.dim_width_m)),
                            cvRound(m_list_gt_objects.at(obj_index).getExtrapolatedGroundTruthDetails().at(
                                    frame_skip - 1).at(frame_count).m_object_location_px.location_y_m),
                            cvRound(m_list_gt_objects.at(obj_index).getExtrapolatedGroundTruthDetails().at(
                                    frame_skip - 1).at(frame_count).m_object_dimensions_px.dim_width_m),
                            cvRound(m_list_gt_objects.at(obj_index).getExtrapolatedGroundTruthDetails().at(
                                    frame_skip - 1).at(frame_count).m_object_dimensions_px.dim_height_m));

                    cv::rectangle(tempGroundTruthImage, boundingbox, cv::Scalar(0, 255, 0), 1, 8, 0);

                }
            }
            cv::namedWindow("BB", CV_WINDOW_AUTOSIZE);
            cv::imshow("BB", tempGroundTruthImage);
            cv::waitKey(0);
            //cv::imwrite(output_image_file_with_path, tempGroundTruthImage);
            /*---------------------------------------------------------------------------------*/
        }
    }
    cv::destroyAllWindows();
}

void GroundTruthScene::prepare_directories() {

    m_groundtruthpath = Dataset::getGroundTruthPath(); // data/stereo_flow

    m_generatepath = m_groundtruthpath.string() + "/" +  m_environment + "/";

    if ( m_regenerate_yaml_file ) {
        if (!m_datasetpath.string().compare(CPP_DATASET_PATH) || !m_datasetpath.string().compare(VIRES_DATASET_PATH)) {

            std::cout << "prepare gt_scene directories" << std::endl;

            if (boost::filesystem::exists(m_generatepath)) {
                system(("rm -rf " + m_generatepath.string()).c_str());
            }
            boost::filesystem::create_directories(m_generatepath);

            char char_dir_append[20];
            boost::filesystem::path path;

            for (int i = 0; i < m_list_gt_objects.size(); i++) {

                sprintf(char_dir_append, "%02d", i);
                m_position_obj_path = m_generatepath.string() + "position_obj_";
                path = m_position_obj_path.string() + char_dir_append;
                boost::filesystem::create_directories(path);

            }
        }
    }
    else {
        // post processing step
        boost::filesystem::path bbox_dir = m_generatepath.string() + "bounding_box/";
        if (boost::filesystem::exists(m_generatepath)) {
            system(("rm -rf " + bbox_dir.string()).c_str());
        }
        boost::filesystem::create_directories(bbox_dir);
    }

}

void GroundTruthScene::writePositionInYaml(std::string suffix) {

    cv::FileStorage write_fs;
    write_fs.open("../position_"+suffix+".yml", cv::FileStorage::WRITE);

    for ( unsigned frame_skip = 1; frame_skip < MAX_SKIPS ; frame_skip++ ) {

        std::cout << "write yaml file for frame_skip  " << (frame_skip-1) << std::endl;

        char temp_str_fs[20];
        sprintf (temp_str_fs, "frame_skip_%03d", frame_skip);
        //write_fs << temp_str_fs << "[";

        unsigned long FRAME_COUNT = m_list_gt_objects.at(0).get_obj_extrapolated_shape_pixel_point_pixel_displacement().at(frame_skip-1).size();
        assert(FRAME_COUNT>0);

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {
            char temp_str_fc[20];
            sprintf (temp_str_fc, "frame_count_%03d", frame_count);
            write_fs << temp_str_fc << "[";
            for ( int i = 0; i< m_list_gt_objects.size(); i++) {
                write_fs
                        << "{:" << "name" << m_list_gt_objects.at(i).getObjectName()
                        << "visible" << m_list_gt_objects.at(i).get_obj_base_visibility().at(frame_count)
                        << "x_camera" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_location_px.location_x_m
                        << "y_camera" << m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_location_px.location_y_m
                        << "z_camera" << m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_location_px.location_z_m
                        << "x_usk" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_location_m.location_x_m
                        << "y_usk" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_location_m.location_y_m
                        << "z_usk" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_location_m.location_z_m
                        << "x_inertial" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_location_inertial_m.location_x_m
                        << "y_inertial" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_location_inertial_m.location_y_m
                        << "z_inertial" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_location_inertial_m.location_z_m
                        << "dim_x_camera" << m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_width_m
                        << "dim_y_camera" << m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_height_m
                        << "dim_x_realworld" << m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_realworld_dim_m.dim_width_m
                        << "dim_y_realworld" << m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_realworld_dim_m.dim_height_m
                        << "dim_z_realworld" << m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_realworld_dim_m.dim_length_m
                        << "speed_x" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_speed.x
                        << "speed_y" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_speed.y
                        << "speed_x_inertial" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_speed_inertial.x
                        << "speed_y_inertial" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_speed_inertial.y
                        << "speed_z_inertial" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_speed_inertial.z
                        << "off_x" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_offset.offset_x
                        << "off_y" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_offset.offset_y
                        << "off_z" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_offset.offset_z
                        << "h_usk" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_rotation_rad.rotation_ry_yaw_rad
                        << "p_usk" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_rotation_rad.rotation_rx_pitch_rad
                        << "r_usk" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_rotation_rad.rotation_rz_roll_rad
                        << "h_inertial" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_rotation_inertial_rad.rotation_ry_yaw_rad
                        << "p_inertial" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_rotation_inertial_rad.rotation_rx_pitch_rad
                        << "r_inertial" <<  m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_rotation_inertial_rad.rotation_rz_roll_rad
                        << "}";
            }
            write_fs << "]";
        }
    }
    //write_fs << "]";
    write_fs.release();
}

void GroundTruthScene::readPositionFromFile(std::string positionFileName) {

    cv::FileStorage fs(positionFileName, cv::FileStorage::READ);

    cv::Point2f offset_pixel, speed_usk, speed_inertial;
    cv::Point2f dimension_pixel;
    cv::Point3f position_inertial, position_usk, position_pixel, dimension_realworld;
    cv::Point3f position_inertial_pre, position_usk_pre, position_pixel_pre;

    cv::Point3f orientation_inertial, orientation_usk;


    cv::FileNode file_node;
    cv::FileNodeIterator file_node_iterator_begin, file_node_iterator_end, file_node_iterator;

    for ( unsigned frame_skip = 1; frame_skip < MAX_SKIPS ; frame_skip++ ) {

        //std::string temp_str = "frame_skip" + frame_skip;
        char temp_str_fs[20];
        sprintf (temp_str_fs, "frame_skip_%03d", frame_skip);
        std::cout << "read yaml file for frame_skip " << (frame_skip-1) << std::endl;
        //unsigned long FRAME_COUNT = m_list_gt_objects.at(0).get_obj_extrapolated_shape_pixel_point_pixel_displacement().at(frame_skip-1).size();
        unsigned long FRAME_COUNT = MAX_ITERATION_GT_SCENE_GENERATION_VECTOR;
        assert(FRAME_COUNT>0);

        for (ushort frame_count = 0; frame_count < FRAME_COUNT; frame_count++) {

            char temp_str_fc[20];
            sprintf(temp_str_fc, "frame_count_%03d", frame_count);
            file_node = fs[temp_str_fc];
            if ( file_node.isNone() || file_node.empty() ) {
                std::cout << temp_str_fc << " cannot be found" << std::endl;
            }
            else {

                //std::cout << file_node.size() << " found" << std::endl;
                file_node_iterator_begin = file_node.begin();
                file_node_iterator_end = file_node.end();

                for ( file_node_iterator = file_node_iterator_begin; file_node_iterator != file_node_iterator_end;
                        file_node_iterator++) {

                    if ( m_mapObjectNameToObjectMetaData.count((*file_node_iterator)["name"].string()) == 0 ) {
                        m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(m_objectCount));
                        m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()] = m_ptr_customObjectMetaDataList.at(m_objectCount);
                        m_ptr_customObjectMetaDataList.at(m_objectCount)->setObjectName((*file_node_iterator)["name"].string());
                        Rectangle rectangle(Dataset::getFrameSize().width, Dataset::getFrameSize().height); // width, height
                        m_ptr_customObjectMetaDataList.at(m_objectCount)->setObjectShape(rectangle);
                        m_objectCount+=1;
                    }

                    std::cout << (*file_node_iterator)["name"].string() << " " << (double)(*file_node_iterator)["x_camera"] << " " << (double)(*file_node_iterator)["y_camera"] << std::endl;

                    position_pixel = cv::Point3f((double)(*file_node_iterator)["x_camera"], (double)(*file_node_iterator)["y_camera"], (double)(*file_node_iterator)["z_camera"]);
                    position_usk = cv::Point3f((double)(*file_node_iterator)["x_usk"], (double)(*file_node_iterator)["y_usk"], (double)(*file_node_iterator)["z_usk"]);
                    position_inertial = cv::Point3f((double)(*file_node_iterator)["x_inertial"], (double)(*file_node_iterator)["y_inertial"], (double)(*file_node_iterator)["z_inertial"]);

                    offset_pixel = cv::Point2f((double)(*file_node_iterator)["off_x"], (double)(*file_node_iterator)["off_y"]);

                    orientation_usk = cv::Point3f((double)(*file_node_iterator)["h_usk"], (double)(*file_node_iterator)["p_usk"], (double)(*file_node_iterator)["r_usk"]);

                    orientation_inertial = cv::Point3f((double)(*file_node_iterator)["h_inertial"], (double)(*file_node_iterator)["p_inertial"], (double)(*file_node_iterator)["r_inertial"]);

                    dimension_pixel = cv::Point2f((int)(*file_node_iterator)["dim_x_camera"], (int)(*file_node_iterator)["dim_y_camera"]);
                    dimension_realworld = cv::Point3f((double)(*file_node_iterator)["dim_x_realworld"], (double)(*file_node_iterator)["dim_y_realworld"], (double)(*file_node_iterator)["dim_z_realworld"]);

                    speed_usk = cv::Point2f((int)(*file_node_iterator)["speed_x"], (int)(*file_node_iterator)["speed_y"]);

                    (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberCameraSensor(frame_count, position_pixel, offset_pixel, dimension_pixel);
                    (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberPerfectSensor(frame_count, position_usk, orientation_usk, dimension_realworld, speed_usk);
                    (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberPerfectSensorInertial(frame_count, position_inertial, orientation_inertial, dimension_realworld, speed_inertial);
                    (m_mapObjectNameToObjectMetaData[(*file_node_iterator)["name"].string()])->atFrameNumberVisibility(frame_count, (int)(*file_node_iterator)["visible"]);

                    position_inertial_pre = position_inertial;
                    position_pixel_pre = position_pixel;
                    position_usk_pre = position_usk;

                }
            }
        }
    }
    fs.release();
}

void GroundTruthSceneInternal::generate_gt_scene(void) {


    /*
    cv::RNG rng(-1);
    for ( unsigned i = 0 ; i < MAX_ITERATION_THETA; i++ ) {
        float a        = (float) rng.uniform(100., 1000.);
        float b        = (float) rng.uniform(100., 300.);
        cv::Point2f points(a,b);
    }
*/


    ColorfulNoise colorfulNoise;

    if ( m_environment == "none") {


        if ( !m_regenerate_yaml_file  ) { // dont generate, just read

            readPositionFromFile("../position_cpp.yml");

            ushort map_pair_count = 0;
            for ( const auto &myPair : m_mapObjectNameToObjectMetaData ) {
                std::cout << myPair.first << "\n";
                map_pair_count++;
            }
        }
        else { // genreate yaml file
            boost::filesystem::remove("../position_cpp.yml");

            Rectangle rectangle(Dataset::getFrameSize().width, Dataset::getFrameSize().height); // width, height

            Achterbahn achterbahn;

            achterbahn = Achterbahn(rectangle, "rectangle_long", 60);
            achterbahn.process(Dataset::getFrameSize());
            objectMetaDataList.at(0) = achterbahn;
            m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(0));

            achterbahn = Achterbahn(rectangle, "random_object", 120);
            achterbahn.process(Dataset::getFrameSize());
            objectMetaDataList.at(1) = achterbahn;
            m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(1));

        }

        for ( auto i = 0; i < m_ptr_customObjectMetaDataList.size() ; i++) {

            GroundTruthObjects gt_obj(m_ptr_customObjectMetaDataList.at(i)->getObjectShape(), *m_ptr_customObjectMetaDataList.at(i), m_ptr_customObjectMetaDataList.at(i)->getObjectStartPoint(), colorfulNoise, m_ptr_customObjectMetaDataList.at(i)->getObjectName());
            m_list_gt_objects.push_back(gt_obj);

        }

        if ( m_regenerate_yaml_file  ) {
            writePositionInYaml("cpp");
        }

        //visualiseBoundingBox();

    }

    cv::Mat tempGroundTruthImage, tempGroundTruthImageBase;
    tempGroundTruthImage.create(Dataset::getFrameSize(), CV_32FC3);
    assert(tempGroundTruthImage.channels() == 3);

    cv::Mat tempGroundTruthPosition;
    tempGroundTruthPosition.create(Dataset::getFrameSize(), CV_32FC3);
    assert(tempGroundTruthPosition.channels() == 3);
    tempGroundTruthPosition = cv::Scalar::all(255);

    cv::Mat tempGroundTruthPosition_2;
    tempGroundTruthPosition_2.create(Dataset::getFrameSize(), CV_32FC3);
    assert(tempGroundTruthPosition_2.channels() == 3);
    tempGroundTruthPosition_2 = cv::Scalar::all(255);


    std::map<std::string, double> time_map = {{"generate_single_scene_image",0},{"generate_all_scene_image", 0}};

    std::cout << "generate_gt_scene at " << m_groundtruthpath.string() << std::endl;

    char file_name_image[50];

    cv::Mat image_data_and_shape;
    cv::Mat positionShape;

    const ushort frame_skip = 1; // image is generated only once irrespective of skips.

    auto tic_all = steady_clock::now();

    // apply black noise in case of night
    if ( m_environment == "night") {
        BlackNoise noise;
        ObjectImageShapeData newCanvas = m_canvas.getCanvasShapeAndData();
        noise.apply(newCanvas);
        tempGroundTruthImageBase = newCanvas.get().clone();
    }
    else {
        tempGroundTruthImageBase = m_canvas.getCanvasShapeAndData().get().clone();
    }

    for (ushort frame_count = 0; frame_count < MAX_ITERATION_GT_SCENE_GENERATION_IMAGES; frame_count++) {

        auto tic = steady_clock::now();

        sprintf(file_name_image, "000%03d_10.png", frame_count*frame_skip);
        std::string input_image_file_with_path = m_generatepath.string() + file_name_image;


        //draw new ground truth image.
        tempGroundTruthImage = tempGroundTruthImageBase.clone();

        char frame_skip_folder_suffix[50];

        for ( unsigned  i = 0; i < m_list_gt_objects.size(); i++ ) {

            sprintf(frame_skip_folder_suffix, "%02d", m_list_gt_objects.at(i).getObjectId());
            std::string position_image_file_with_path = m_position_obj_path.string() +
                    frame_skip_folder_suffix + "/" + file_name_image;

            image_data_and_shape = m_list_gt_objects.at(i).getImageShapeAndData().get().clone();
            image_data_and_shape = image_data_and_shape.rowRange(0, cvRound(m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_height_m)).colRange(0,cvRound(m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_width_m));
            positionShape = m_list_gt_objects.at(i).getImageShapeAndData().get().clone();
            positionShape = positionShape.rowRange(0, cvRound(m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_height_m)).colRange(0,cvRound(m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_width_m));

            if ( ( m_list_gt_objects.at(i).get_obj_base_visibility().at(frame_count))
                    ) {

                image_data_and_shape.copyTo(tempGroundTruthImage(
                        cv::Rect(cvRound(m_list_gt_objects.at(i).get_obj_base_pixel_position_pixel_displacement().at(frame_count).first.x),
                                cvRound(m_list_gt_objects.at(i).get_obj_base_pixel_position_pixel_displacement().at(frame_count).first.y),
                                cvRound(m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_width_m),
                                cvRound(m_list_gt_objects.at(i).getExtrapolatedGroundTruthDetails().at(frame_skip-1).at(frame_count).m_object_dimensions_px.dim_height_m))));

            }
        }

        cv::imwrite(input_image_file_with_path, tempGroundTruthImage);
        auto toc = steady_clock::now();
        time_map["generate_single_scene_image"] = duration_cast<milliseconds>(toc - tic).count();

    }

    auto toc_all = steady_clock::now();
    time_map["generate_all_scene_image"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth scene generation time - " << time_map["generate_all_scene_image"] << "ms" << std::endl;

}

void GroundTruthScene::generate_bird_view() {
    // the bird view needs the range information of each object
    // Assuming the camera is mounted on the floor.
    /*
        for ( auto position_index = 0;  position_index <  MAX_ITERATION_RESULTS; position_index++ ) {

            cv::Mat birdview_frame(Dataset::getFrameSize(), CV_32FC1);
            for ( auto object_index= 0; object_index < m_list_gt_objects.size(); object_index++ ) {
                birdview_frame.at(m_list_gt_objects.at(object_index).get_obj_base_pixel_position_pixel_displacement().at(position_index).first.x, m_list_gt_objects.at(object_index).get_obj_range().at(position_index)) = 100;
                cv::Mat roi_objects;
                roi_objects = birdview_frame.rowRange(m_list_gt_objects.at(object_index).get_obj_range().at(position_index), m_list_gt_objects.at(object_index).get_obj_dimension().at(position_index).z_offset )
                        .colRange(m_list_gt_objects.at(object_index).get_obj_range().at(position_index), m_list_gt_objects.at(object_index).get_obj_dimension().at(position_index).x_offset);

            }
            cv::imwrite("filename", birdview_frame);
            // time to collide with the object.

        }
        */
}

void GroundTruthSceneExternal::generate_gt_scene() {

    Noise noNoise;

    if ( m_environment == "none") {

        if ( !m_regenerate_yaml_file  ) { // dont generate, just read

            readPositionFromFile("../position_vires.yml");

            ushort map_pair_count = 0;
            for ( const auto &myPair : m_mapObjectNameToObjectMetaData ) {
                std::cout << myPair.first << "\n";
                map_pair_count++;
            }
        }
    }

    if (m_regenerate_yaml_file) { // call VIRES only at the time of generating the files

        char command[1024];

        std::string project = "Movement";

        std::vector<std::string> list_of_scenarios = {"carTypesComplete.xml", "crossing8Demo.xml",
                "crossing8DualExt.xml",
                "crossing8Static.xml", "HighwayPulk.xml", "invisibleCar.xml", "ParkPerp.xml",
                "RouteAndPathShapeSCP.xml",
                "staticCar.xml", "TownActionsPath.xml", "TownPathLong.xml", "traffic_demo2Ext.xml",
                "trafficDemoClosePath.xml", "trafficDemoPath.xml", "trafficDemoPed.xml", "traffic_demoReverse.xml",
                "trafficDemoTrailer.xml", "trafficDemoUK.xml", "traffic_demo.xml"
                        "car.xml", "moving_car_near.xml", "moving_car.xml", "moving_truck.xml", "moving.xml", "one.xml",
                "truck.xml", "two.xml"};

        sleep(5); // Wait before starting vtd again.

        if ( m_environment == "none") {
            sprintf(command, "cd %s../../ ; bash vtdSendandReceive.sh %s", (m_datasetpath.string()).c_str(),
                    project.c_str());
            std::cout << command << std::endl;
            system(command);
            std::cout << " I am out of bash" << std::endl;
        }

        sleep(5); // Give some time before you send SCP commands.

        // std::string m_server;
        boost::filesystem::path m_ts_gt_out_dir;

        int initCounter = 6;

        // initalize the server variable
        std::string serverName = "127.0.0.1";

        setServer(serverName.c_str());

        fprintf(stderr, "ValidateArgs: key = 0x%x, checkMask = 0x%x, mForceBuffer = %d\n", mShmKey, getCheckMask(),
                getForceBuffer());

        bool connected_trigger_port = false;
        bool connected_module_manager_port = false;
        bool connected_scp_port = false;

        //if ( m_environment == "none" ) {
        m_scpSocket = openNetwork(SCP_DEFAULT_PORT);
        //}
        std::cout << "scp socket - " << m_scpSocket << std::endl;
        if (m_scpSocket != -1) { // this is blocking until the network has been opened
            connected_scp_port = true;
        }

        sleep(1); // Give some time before you send the next SCP command.

        sendSCPMessage(m_scpSocket, apply.c_str());

        sleep(5); // This is very important !! Mimimum 5 seconds of wait, till you start the simulation

        sendSCPMessage(m_scpSocket, project_name.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, rdbtrigger_portnumber.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, scenario_name.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, module_manager_libModuleCameraSensor.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, module_manager_libModulePerfectSensor.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, module_manager_libModulePerfectSensorInertial.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, view_parameters_sensorpoint_intrinsicparams.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, display_parameters.c_str());

        sleep(2);

        sendSCPMessage(m_scpSocket, m_environment_scp_message.c_str());

        sleep(1);

        sendSCPMessage(m_scpSocket, message_scp.c_str());

        sleep(1);

        //sendSCPMessage(m_scpSocket, popup_scp.c_str());

        //sleep(1);

        sendSCPMessage(m_scpSocket, eyepoint.c_str());

        sleep(1);


        sendSCPMessage(m_scpSocket, elevation.c_str());

        sleep(1);

        //sendSCPMessage(m_scpSocket, bbox.c_str());

        sleep(1);

        sprintf(command, "cd %s../../ ; bash vtdRunScp.sh", (m_datasetpath.string()).c_str());
        std::cout << command << std::endl;
        system(command);

        sleep(10);  // Give some time before you start the trigger and module manager ports.

        //readScpNetwork(m_scpSocket);

        //readScpNetwork(m_scpSocket);

        //sleep(1);


        // open the network connection to the taskControl (so triggers may be sent)
        fprintf(stderr, "creating network connection....\n");
        //if ( m_environment == "none") {
        m_triggerSocket = openNetwork(DEFAULT_PORT);
        //}
        std::cout << "trigger socket - " << m_triggerSocket << std::endl;
        if (m_triggerSocket != -1) { // this is blocking until the network has been opened
            connected_trigger_port = true;
        }

        //if ( m_environment == "none") {
        m_moduleManagerSocket_Camera = openNetwork(DEFAULT_RX_PORT);
        m_moduleManagerSocket_Perfect = openNetwork(DEFAULT_RX_PORT_PERFECT);
        m_moduleManagerSocket_PerfectInertial = openNetwork(DEFAULT_RX_PORT_PERFECT_INERTIAL);
        //}

        std::cout << "mm socket - " << m_moduleManagerSocket_Camera << std::endl;
        if (m_moduleManagerSocket_Camera != -1) {
            connected_module_manager_port = true;
        }

        if (connected_trigger_port && connected_module_manager_port && connected_scp_port) {

            // open the shared memory for IG image output (try to attach without creating a new segment)
            fprintf(stderr, "openCommunication: attaching to shared memory (IG image output) 0x%x....\n", mShmKey);

            while (!getShmPtr()) {
                openShm(mShmKey);
                usleep(1000);     // do not overload the CPU
            }

            // now check the SHM for the time being
            bool breaking = false;
            int count = 0;

            int lastSimFrame = -1; // parseEndOfFrame
            try {
                while (1) {

                    if (breaking) {
                        break;
                    }

                    if (mSimFrame > MAX_ITERATION_GT_SCENE_GENERATION_DYNAMIC) {
                        breaking = true;
                    }

                    readNetwork(m_moduleManagerSocket_Camera);  // this calls parseRDBMessage() in vires_common.cpp

                    readNetwork(m_moduleManagerSocket_Perfect);  // this calls parseRDBMessage() in vires_common.cpp

                    readNetwork(m_moduleManagerSocket_PerfectInertial);  // this calls parseRDBMessage() in vires_common.cpp

                    bool haveNewFrame = false;

                    if (mLastNetworkFrame < 0) {
                        std::cerr << "Flushing images in shared memory" << std::endl;
                        checkShm();  //empty IG buffer of spurious images
                    } else {
                        haveNewFrame = (lastSimFrame != mLastNetworkFrame);
                    }

                    lastSimFrame = mLastNetworkFrame;

                    if (!mHaveFirstImage || mHaveImage || haveNewFrame || !mHaveFirstFrame) {
                        // do not initialize too fast
                        if (!mHaveFirstImage || !mHaveFirstFrame) {
                            // only in the beginning.
                            usleep(100000);
                        }

                        //bool requestImage = (mLastNetworkFrame >= (mLastIGTriggerFrame + IMAGE_SKIP_FACTOR_DYNAMIC));
                        bool requestImage = ((mLastNetworkFrame % IMAGE_SKIP_FACTOR_DYNAMIC*1000) == 0);
                        mLastNetworkFrame++;

                        if (requestImage) {
                            //std::cout << mLastNetworkFrame << " " << mLastIGTriggerFrame << std::endl;
                            mLastIGTriggerFrame = mLastNetworkFrame;
                            mCheckForImage = true;
                            fprintf( stderr, "sendRDBTrigger: sending trigger, deltaT = %.4lf, requestImage = %s at simFrame %d\n", mDeltaTime, requestImage ? "true" : "false", mSimFrame );
                            sendRDBTrigger(m_triggerSocket, mSimTime, mSimFrame, requestImage, 0.1);
                        }

                        // calculate the timing statistics
                        if (mHaveImage) {
                            //calcStatistics();
                        }
                    }

                    while (mCheckForImage) {

                        checkShm();

                        mCheckForImage = !mHaveImage;

                        if (!mCheckForImage) {
                            fprintf( stderr, "main: got it! at %d\n", mSimFrame );
                            mHaveImage = false;
                        }

                        usleep(10);
                    }

                    // increase internal counters
                    if (haveNewFrame ) {
                        mSimTime += mDeltaTime;
                        mSimFrame++;
                        haveNewFrame = false;
                        mHaveFirstFrame = true;

                    }
                    // has an image arrived or do the first frames need to be triggered
                    //(first image will arrive with a certain image_02_frame delay only)

                    usleep(10000); // sleep for 10 ms
                    //std::cout << "getting data from VIRES\n";
                }
            }
            catch (...) {
                stopSimulation();
                return;
            };

            configVires();
        }
    }
    try {
        if ( m_environment == "none") {

            for ( auto i = 0; i < m_ptr_customObjectMetaDataList.size() ; i++) {

                GroundTruthObjects gt_obj(m_ptr_customObjectMetaDataList.at(i)->getObjectShape(), *m_ptr_customObjectMetaDataList.at(i), m_ptr_customObjectMetaDataList.at(i)->getObjectStartPoint(), noNoise, m_ptr_customObjectMetaDataList.at(i)->getObjectName());
                m_list_gt_objects.push_back(gt_obj);

            }

            if ( m_regenerate_yaml_file  ) {
                writePositionInYaml("vires");
            }

            //visualiseBoundingBox();

        }
    }
    catch (...) {
        stopSimulation();
    }
}


void GroundTruthSceneExternal::parseEntry( RDB_TRIGGER_t *data, const double & simTime, const unsigned int &
simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId,
        const unsigned int & totalElem )
{
    fprintf(stderr, "RDBTrigger answer = %.3f, simFrame = %d\n", simTime, simFrame);
}


void GroundTruthSceneExternal::parseStartOfFrame(const double &simTime, const unsigned int &simFrame) {
    //fprintf(stderr, "I am in GroundTruthFlow %d\n,", RDB_PKG_ID_START_OF_FRAME);
    //mHaveFirstFrame = true;
    //fprintf(stderr, "RDBHandler::parseStartOfFrame: simTime = %.3f, simFrame = %d\n", simTime, simFrame);
}

void GroundTruthSceneExternal::parseEndOfFrame(const double &simTime, const unsigned int &simFrame) {

    //mLastNetworkFrame = simFrame;
    fprintf(stderr, "RDBHandler::parseEndOfFrame: simTime = %.3f, simFrame = %d\n", simTime, simFrame);
}

/** ------ state of an object (may be extended by the next structure) ------- */
typedef struct
{
    uint32_t            id;                         /**< unique object ID                                              @unit _                                   */
    uint8_t             category;                   /**< object category                                               @unit @link RDB_OBJECT_CATEGORY @endlink  */
    uint8_t             type;                       /**< object type                                                   @unit @link RDB_OBJECT_TYPE     @endlink  */
    uint16_t            visMask;                    /**< visibility mask                                               @unit @link RDB_OBJECT_VIS_FLAG @endlink  */
    char                name[RDB_SIZE_OBJECT_NAME]; /**< symbolic name                                                 @unit _                                   */
    RDB_GEOMETRY_t      geo;                        /**< info about object's geometry                                  @unit m,m,m,m,m,m                         */
    RDB_COORD_t         pos;                        /**< position and orientation of object's reference point          @unit m,m,m,rad,rad,rad                   */
    uint32_t            parent;                     /**< unique ID of parent object                                    @unit _                                   */
    uint16_t            cfgFlags;                   /**< configuration flags                                           @unit @link RDB_OBJECT_CFG_FLAG @endlink  */
    int16_t             cfgModelId;                 /**< visual model ID (configuration parameter)                     @unit _                                   */
} RDB_OBJECT_STATE_BASE_DUMMY_t;

/** ------ extended object data (e.g. for dynamic objects) ------- */
typedef struct
{
    RDB_COORD_t         speed;                      /**< speed and rates                                               @unit m/s,m/s,m/s,rad/s,rad/s,rad/s           */
    RDB_COORD_t         accel;                      /**< acceleration                                                  @unit m/s2,m/s2,m/s2,rad/s2,rad/s2/rad/s2     */
    float               traveledDist;               /**< traveled distance                                             @unit m                                      a */
    uint32_t            spare[3];                   /**< reserved for future use                                       @unit _                                       */
} RDB_OBJECT_STATE_EXT_DUMMY_t;

/** ------ sensor definition and state ------ */
typedef struct
{
    uint32_t    id;                          /**< id of the sensor                                      @unit _                                      */
    uint8_t     type;                        /**< type of the sensor                                    @unit @link RDB_SENSOR_TYPE     @endlink     */
    uint8_t     hostCategory;                /**< category of the object hosting the sensor             @unit @link RDB_OBJECT_CATEGORY @endlink     */
    uint16_t    spare0;                      /**< for future use                                        @unit _                                      */
    uint32_t    hostId;                      /**< unique id of the sensor's host                        @unit _                                      */
    char        name[RDB_SIZE_OBJECT_NAME];  /**< name of the sensor                                    @unit _                                      */
    float       fovHV[2];                    /**< field-of-view (horizontal/vertical)                   @unit rad,rad                                */
    float       clipNF[2];                   /**< clipping ranges (near/far)                            @unit m,m                                    */
    RDB_COORD_t pos;                         /**< position and orientation of sensor's reference point  @unit m,m,m,rad,rad,rad                      */
    RDB_COORD_t originCoordSys;              /**< position and orientation of sensor's coord origin     @unit m,m,m,rad,rad,rad                      */
    float       fovOffHV[2];                 /**< field-of-view offset (horizontal/vertical)            @unit rad, rad                              B */
    int32_t     spare[2];                    /**< for future use                                        @unit _                                      */
} RDB_SENSOR_STATE_DUMMY_t;

/** ------ information about an object registered within a sensor ------ */
typedef struct
{
    uint8_t     category;    /**< object category                                                                @unit @link RDB_OBJECT_CATEGORY    @endlink   */
    uint8_t     type;        /**< object type                                                                    @unit @link RDB_OBJECT_TYPE        @endlink   */
    uint16_t    flags;       /**< sensor object flags                                                            @unit @link RDB_SENSOR_OBJECT_FLAG @endlink   */
    uint32_t    id;          /**< id of the object                                                               @unit _                                       */
    uint32_t    sensorId;    /**< id of the detecting sensor                                                     @unit _                                       */
    double      dist;        /**< distance between object and referring device                                   @unit m                                       */
    RDB_COORD_t sensorPos;   /**< position and orientation of object in sensor coord                             @unit m,m,m,rad,rad,rad                       */
    int8_t      occlusion;   /**< degree of occlusion for viewer (-1 = not valid, 0..127 = 0..100% occlusion)    @unit [-1, 0..127]                            */
    uint8_t     spare0[3];   /**< for future use                                                                 @unit _                                       */
    uint32_t    spare[3];    /**< for future use                                                                 @unit _                                       */
} RDB_SENSOR_OBJECT_DUMMY_t;

void GroundTruthSceneExternal::parseEntry(RDB_OBJECT_CFG_t *data, const double &simTime, const unsigned int &
simFrame, const
unsigned short &pkgId, const unsigned short &flags,
        const unsigned int &elemId, const unsigned int &totalElem) {

    RDB_OBJECT_CFG_t *object = reinterpret_cast<RDB_OBJECT_CFG_t *>(data); /// raw image data
    std::cout << object->type;


}

void GroundTruthSceneExternal::parseEntry(RDB_OBJECT_STATE_t *data, const double &simTime, const unsigned int &
simFrame, const
unsigned
short &pkgId, const unsigned short &flags, const unsigned int &elemId,
        const unsigned int &totalElem) {

    cv::Point3f position_inertial, position_usk, position_pixel, dimension_realworld;
    cv::Point2f dimension_pixel, offset_pixel;
    cv::Point2f speed_inertial, speed_usk;
    cv::Point3f orientation_usk, orientation_inertial;

    if ( m_environment == "none") {

        if ( data->base.type == RDB_OBJECT_TYPE_PLAYER_PEDESTRIAN || data->base.type == RDB_OBJECT_TYPE_PLAYER_CAR ) {
            if ( mHaveFirstImage ) {

                fprintf(stderr, "%s: %d %.3lf %.3lf %.3lf %.3lf \n",
                        data->base.name, simFrame, data->base.pos.x, data->base.pos.y, data->base.geo.dimX, data->base
                                .geo.dimY);

                if (m_mapObjectNameToObjectMetaData.count(data->base.name) == 0) {

                    m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(m_objectCount));
                    Rectangle rectangle((int)(data->base.geo.dimX),(int)(data->base.geo.dimY)); // width, height
                    m_mapObjectNameToObjectMetaData[data->base.name] = m_ptr_customObjectMetaDataList.at(m_objectCount);
                    m_ptr_customObjectMetaDataList.at(m_objectCount)->setObjectShape(rectangle);
                    m_ptr_customObjectMetaDataList.at(m_objectCount)->setObjectName(data->base.name);
                    m_objectCount += 1;
                }
                if ( data->base.pos.type == RDB_COORD_TYPE_WINDOW )
                {
                    position_pixel = cv::Point3f((float) data->base.pos.x, (float) data->base.pos.y, float(data->base.pos.z));
                    dimension_pixel = cv::Point2f((float) data->base.geo.dimX, (float) data->base.geo.dimY);
                    offset_pixel = cv::Point2f((float) data->base.geo.offX, (float) data->base.geo.offY);
                    m_mapObjectNameToObjectMetaData[data->base.name]->atFrameNumberCameraSensor((ushort)(simFrame/IMAGE_SKIP_FACTOR_DYNAMIC), position_pixel, offset_pixel, dimension_pixel);
                    m_mapObjectNameToObjectMetaData[data->base.name]->atFrameNumberVisibility((ushort)(simFrame/IMAGE_SKIP_FACTOR_DYNAMIC), true);
                }
                else if ( data->base.pos.type == RDB_COORD_TYPE_USK ) {
                    position_usk = cv::Point3f((float) data->base.pos.x, (float) data->base.pos.y, (float) data->base.pos.z);
                    orientation_usk = cv::Point3f((float) data->base.pos.h, (float) data->base.pos.p, (float) data->base.pos.r);
                    dimension_realworld = cv::Point3f((float) data->base.geo.dimX, (float) data->base.geo.dimY, (float) data->base.geo.dimZ);
                    speed_usk = cv::Point2f((float) data->ext.speed.x, (float) data->ext.speed.y);
                    m_mapObjectNameToObjectMetaData[data->base.name]->atFrameNumberPerfectSensor((ushort)(simFrame/IMAGE_SKIP_FACTOR_DYNAMIC), position_usk, orientation_usk, dimension_realworld, speed_usk);
                    m_mapObjectNameToObjectMetaData[data->base.name]->atFrameNumberVisibility((ushort)(simFrame/IMAGE_SKIP_FACTOR_DYNAMIC), true);
                }
                else if ( data->base.pos.type == RDB_COORD_TYPE_INERTIAL ) {
                    position_inertial = cv::Point3f((float) data->base.pos.x, (float) data->base.pos.y, (float) data->base.pos.z);
                    orientation_inertial = cv::Point3f((float) data->base.pos.h, (float) data->base.pos.p, (float) data->base.pos.r);
                    dimension_realworld = cv::Point3f((float) data->base.geo.dimX, (float) data->base.geo.dimY, (float) data->base.geo.dimZ);
                    speed_inertial = cv::Point2f((float) data->ext.speed.x, (float) data->ext.speed.y);
                    m_mapObjectNameToObjectMetaData[data->base.name]->atFrameNumberPerfectSensorInertial((ushort)(simFrame/IMAGE_SKIP_FACTOR_DYNAMIC), position_inertial, orientation_inertial, dimension_realworld, speed_inertial);
                    m_mapObjectNameToObjectMetaData[data->base.name]->atFrameNumberVisibility((ushort)(simFrame/IMAGE_SKIP_FACTOR_DYNAMIC), true);
                }
            } else {
                //std::cout << data->base.type << std::endl;
            }
        }
    }
}

void GroundTruthSceneExternal::parseEntry(RDB_IMAGE_t *data, const double &simTime, const unsigned int &simFrame,
        const
        unsigned short &pkgId, const unsigned short &flags,
        const unsigned int &elemId, const unsigned int &totalElem) {

    if (!data)
        return;
    fprintf(stderr, "handleRDBitem: image at simFrame %d\n", simFrame);
    //fprintf(stderr, "    simTime = %.3lf, simFrame = %d, mLastShmFrame = %d\n", simTime, simFrame, getLastShmFrame());
    //fprintf(stderr, "    width / height = %d / %d\n", data->width, data->height);
    //fprintf(stderr, "    dataSize = %d\n", data->imgSize);

    // ok, I have an image, but it might be the first one

    //analyzeImage(  data  , simFrame, 0 );


    if ( mHaveFirstImage  ) { // always ignore the first image.

        char *image_data_ = NULL;
        RDB_IMAGE_t *image = reinterpret_cast<RDB_IMAGE_t *>(data); /// raw image data


        /// RDB image information of \see image_data_
        RDB_IMAGE_t image_info_;
        memcpy(&image_info_, image, sizeof(RDB_IMAGE_t));

        if (NULL == image_data_) {
            image_data_ = reinterpret_cast<char *>(malloc(image_info_.imgSize));
        } else {
            image_data_ = reinterpret_cast<char *>(realloc(image_data_, image_info_.imgSize));
        }
        // jump data header
        memcpy(image_data_, reinterpret_cast<char *>(image) + sizeof(RDB_IMAGE_t), image_info_.imgSize);

        if (image_info_.imgSize == image_info_.width * image_info_.height * 3) {
            png::image<png::rgb_pixel> save_image(image_info_.width, image_info_.height);
            unsigned int count = 0;
            for (int32_t v = 0; v < image_info_.height; v++) {
                for (int32_t u = 0; u < image_info_.width; u++) {
                    png::rgb_pixel val;
                    val.red = (unsigned char) image_data_[count++];
                    val.green = (unsigned char) image_data_[count++];
                    val.blue = (unsigned char) image_data_[count++];
                    //val.alpha = (unsigned char)image_data_[count++];
                    save_image.set_pixel(u, v, val);
                }
            }

            //fprintf(stderr, "got a RGB image with %d channels\n", image_info_.imgSize / (image_info_.width * image_info_
            //.height));

            char file_name_image[50];

            fprintf( stderr, "------------------------------------------------------------------------------------\n");
            fprintf( stderr, "saving image at simFrame = %d, simTime = %.3f, dataSize = %d with image id %d\n", simFrame, mSimTime, data->imgSize, data->id);
            fprintf( stderr, "------------------------------------------------------------------------------------\n");
            sprintf(file_name_image, "000%03d_10.png", mImageCount);
            std::string input_image_file_with_path = m_generatepath.string() + file_name_image;
            save_image.write(input_image_file_with_path);
            mImageCount++;

        }
        else {
            fprintf(stderr, "ignoring file with %d channels\n", image_info_.imgSize / (image_info_.width * image_info_.height));
        }
        mHaveImage      = true;
    }
    else {
            if ( mFirstIgnoredFrame == simFrame ) {
                mHaveFirstImage = true;
            }
            mFirstIgnoredFrame = simFrame;
            std::cerr << "ignoring images from first frame" << std::endl;
    }
}


/**
* handle driver control input and compute vehicle dynamics output
*/
void GroundTruthSceneExternal::parseEntry(RDB_DRIVER_CTRL_t *data, const double &simTime, const unsigned int &simFrame,
        const unsigned short &pkgId, const unsigned short &flags,
        const unsigned int &elemId, const unsigned int &totalElem) {

    if (!data)
        return;

    static bool         sVerbose     = true;
    static bool         sShowMessage = false;
    static unsigned int sMyPlayerId  = 1;             // this may also be determined from incoming OBJECT_CFG messages
    static double       sLastSimTime = -1.0;

    fprintf( stderr, "handleRDBitem: handling driver control for player %d\n", data->playerId );

    // is this a new message?
    //if ( simTime == sLastSimTime )
    //    return;

    // is this message for me?
    if ( data->playerId != sMyPlayerId )
        return;

    // check for valid inputs (only some may be valid)
    float mdSteeringAngleRequest = ( data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_STEERING_WHEEL ) ? data->steeringWheel / 19.0 : 0.0;
    float mdThrottlePedal        = ( data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_THROTTLE )       ? data->throttlePedal        : 0.0;
    float mdBrakePedal           = ( data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_BRAKE )          ? data->brakePedal           : 0.0;
    float mInputAccel            = ( data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL )      ? data->accelTgt             : 0.0;
    float mInputSteering         = ( data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING )   ? data->steeringTgt          : 0.0;
    float mdSteeringRequest      = ( data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING )   ? data->steeringTgt          : 0.0;
    float mdAccRequest           = ( data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL )      ? data->accelTgt             : 0.0;
    int   mInputGear             = 0;

    // check the input validity
    unsigned int validFlagsLat  = RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING | RDB_DRIVER_INPUT_VALIDITY_STEERING_WHEEL;
    unsigned int validFlagsLong = RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL | RDB_DRIVER_INPUT_VALIDITY_THROTTLE | RDB_DRIVER_INPUT_VALIDITY_BRAKE;
    unsigned int checkFlags     = data->validityFlags & 0x00000fff;

    if ( checkFlags )
    {
        if ( ( checkFlags & validFlagsLat ) && ( checkFlags & validFlagsLong ) )
            sShowMessage = false;
        else if ( checkFlags != RDB_DRIVER_INPUT_VALIDITY_GEAR ) // "gear only" is also fine
        {
            if ( !sShowMessage )
                fprintf( stderr, "Invalid driver input for vehicle dynamics" );

            sShowMessage = true;
        }
    }

    // use pedals/wheel or targets?
    bool mUseSteeringTarget = ( ( data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING ) != 0 );
    bool mUseAccelTarget    = ( ( data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL ) != 0 );

    if ( data->validityFlags & RDB_DRIVER_INPUT_VALIDITY_GEAR )
    {
        if ( data->gear == RDB_GEAR_BOX_POS_R )
            mInputGear = -1;
        else if ( data->gear == RDB_GEAR_BOX_POS_N )
            mInputGear = 0;
        else if ( data->gear == RDB_GEAR_BOX_POS_D )
            mInputGear = 1;
        else
            mInputGear = 1;
    }

    // now, depending on the inputs, select the control mode and compute outputs
    if ( mUseSteeringTarget && mUseAccelTarget )
    {
        fprintf( stderr, "Compute new vehicle position from acceleration target and steering target.\n" );

        // call your methods here
    }
    else if ( !mUseSteeringTarget && !mUseAccelTarget )
    {
        fprintf( stderr, "Compute new vehicle position from brake pedal, throttle pedal and steering wheel angle.\n" );

        // call your methods here
    }
    else
    {
        fprintf( stderr, "Compute new vehicle position from a mix of targets and pedals / steering wheel angle.\n" );

        // call your methods here
    }

    bool useDummy = true;

    RDB_OBJECT_STATE_t sOwnObjectState;

    // the following assignments are for dummy purposes only
    // vehicle moves along x-axis with given speed
    // ignore first message
    if ( useDummy && ( sLastSimTime >= 0.0 ) )
    {
        double speedX = 5.0;    // m/s
        double speedY = 0.0;    // m/s
        double speedZ = 0.0;    // m/s
        double dt     = simTime - sLastSimTime;

        sOwnObjectState.base.id       = sMyPlayerId;
        sOwnObjectState.base.category = RDB_OBJECT_CATEGORY_PLAYER;
        sOwnObjectState.base.type     = RDB_OBJECT_TYPE_PLAYER_CAR;
        strcpy( sOwnObjectState.base.name, "Ego" );

        // dimensions of own vehicle
        sOwnObjectState.base.geo.dimX = 4.60;
        sOwnObjectState.base.geo.dimY = 1.86;
        sOwnObjectState.base.geo.dimZ = 1.60;

        // offset between reference point and center of geometry
        sOwnObjectState.base.geo.offX = 0.80;
        sOwnObjectState.base.geo.offY = 0.00;
        sOwnObjectState.base.geo.offZ = 0.30;

        sOwnObjectState.base.pos.x     += dt * speedX;
        sOwnObjectState.base.pos.y     += dt * speedY;
        sOwnObjectState.base.pos.z     += dt * speedZ;
        sOwnObjectState.base.pos.h     = 0.0;
        sOwnObjectState.base.pos.p     = 0.0;
        sOwnObjectState.base.pos.r     = 0.0;
        sOwnObjectState.base.pos.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;

        sOwnObjectState.ext.speed.x     = speedX;
        sOwnObjectState.ext.speed.y     = speedY;
        sOwnObjectState.ext.speed.z     = speedZ;
        sOwnObjectState.ext.speed.h     = 0.0;
        sOwnObjectState.ext.speed.p     = 0.0;
        sOwnObjectState.ext.speed.r     = 0.0;
        sOwnObjectState.ext.speed.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;

        sOwnObjectState.ext.accel.x     = 0.0;
        sOwnObjectState.ext.accel.y     = 0.0;
        sOwnObjectState.ext.accel.z     = 0.0;
        sOwnObjectState.ext.accel.flags = RDB_COORD_FLAG_POINT_VALID;

        sOwnObjectState.base.visMask    =  RDB_OBJECT_VIS_FLAG_TRAFFIC | RDB_OBJECT_VIS_FLAG_RECORDER;
    }

    // ok, I have a new object state, so let's send the data
    sendOwnObjectState( sOwnObjectState, m_triggerSocket, simTime, simFrame);

    // remember last simulation time
    sLastSimTime = simTime;
}

double GroundTruthSceneExternal::getTime()
{
    struct timeval tme;
    gettimeofday(&tme, 0);

    double now = tme.tv_sec + 1.0e-6 * tme.tv_usec;

    if ( mStartTime < 0.0 )
        mStartTime = now;

    return now;
}


void GroundTruthSceneExternal::calcStatistics()
{
    double now = getTime();

    double dt = now - mStartTime;

    if ( dt < 1.e-6 )
        return;

    fprintf( stderr, "calcStatistics: received %d/%d images in %.3lf seconds (i.e. %.3lf/%.3lf images per second ), total number of errors = %d\n",
            mTotalNoImages, dt, mTotalNoImages / dt, mTotalErrorCount );
}

void GroundTruthSceneExternal::analyzeImage( RDB_IMAGE_t* img, const unsigned int & simFrame, unsigned int index )
{
    static unsigned int sLastImgSimFrame =  0;

    if ( !img || ( index > 1 ) )
        return;

    if ( img->id == 0 )
        return;

    fprintf( stderr, "analyzeImage: simFrame = %d, index = %d: have image no. %d, size = %d bytes, pixelFormat = %d\n",
            simFrame, index, img->id, img->imgSize, img->pixelFormat );

    if ( img->pixelFormat == RDB_PIX_FORMAT_RGB32F )		// some analysis
    {
        float *imgData = ( float* ) ( ( ( char* ) img ) + sizeof( RDB_IMAGE_t ) );

        for ( int i = 0; i < 10; i++ )	// first 10 pixels
        {
            fprintf( stderr, "r / g / b = %.3f / %.3f / %.3f\n", imgData[0], imgData[1], imgData[2] );
            imgData += 3;
        }
    }
    else if ( img->pixelFormat == RDB_PIX_FORMAT_RGB8 )		// some analysis
    {
        unsigned char *imgData = ( unsigned char* ) ( ( ( char* ) img ) + sizeof( RDB_IMAGE_t ) );

        for ( int i = 0; i < 10; i++ )	// first 10 pixels
        {
            fprintf( stderr, "r / g / b = %d / %d / %d\n", imgData[0], imgData[1], imgData[2] );
            imgData += 3;
        }
    }


    //if ( ( myImg->id > 3 ) && ( ( myImg->id - mLastImageId ) != IMAGE_SKIP_FACTOR_DYNAMIC ) )
    if ( ( simFrame!= sLastImgSimFrame ) && ( img->id > 3 ) && ( ( simFrame- sLastImgSimFrame ) != IMAGE_SKIP_FACTOR_DYNAMIC ) )
    {
        fprintf( stderr, "WARNING: parseRDBMessageEntry: index = %d, delta of image ID out of bounds: delta = %d\n", index, simFrame- sLastImgSimFrame );
        mTotalErrorCount++;
    }

    mLastImageId    = img->id;
    mTotalNoImages++;

    sLastImgSimFrame = simFrame;
}



/**
 * @brief calcBBFrom3DPosition
 *
 * For the camera sensor the GT data from VTD is slightly shifted. Therefore we try to get better
 * bounding boxes by calculating the bounding box on the image from the 3D data delivered by the perfect
 * sensor.
 * @param screen_width      width of VTD display
 * @param screen_height     height of VTD display
 * @param cam_pos           position of camera relative to vehicle point of origin
 * @param fov_v             vertical fov of the VTD camera
 * @param pixSize           pixel size
 */

