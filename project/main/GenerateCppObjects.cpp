//
// Created by veikas on 26.06.18.
//


#include <memory>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>
#include <algorithm>
#include "Noise.h"
#include "ObjectMetaData.h"
#include "GenerateCppObjects.h"
#include "GroundTruthScene.h"

void CppObjects::process(std::unique_ptr<Noise> &noise, ushort sensor_index) {


    std::string sensor_name = "Sensor_MM_" + std::to_string(sensor_index);
    if (m_mapSensorNameToSensorMetaData.count(sensor_name) == 0) {

        //m_mapSensorIdToSensorName[data->id] = sensor_name;
        m_ptr_customSensorMetaDataList.push_back(&sensorMetaDataList.at(m_sensorCount));
        m_mapSensorNameToSensorMetaData[sensor_name] = m_ptr_customSensorMetaDataList.at(m_sensorCount);
        m_ptr_customSensorMetaDataList.at(m_sensorCount)->setSensorName(sensor_name);
        m_ptr_customSensorMetaDataList.at(m_sensorCount)->setStartPoint(0);
        m_sensorCount += 1;

    }


    std::unique_ptr<Noise> objectNoise = std::make_unique<ColorfulNoise>();
    std::unique_ptr<Noise> objectNoise_noNoise = std::make_unique<NoNoise>();

    Rectangle rectangle_obj1(30, 70, objectNoise, 200); // width, height
    Circle circle_obj1(std::max(30,70), objectNoise_noNoise, 200); // width, height

    Rectangle rectangle_obj2(30, 70, objectNoise, 11); // width, height
    Circle circle_obj2(std::max(30,70), objectNoise_noNoise, 11); // width, height

    Achterbahn achterbahn;

    ushort start_point_1 = 100; // 100
    ushort start_point_2 = 220; // 220

    Canvas canvas((ushort)Dataset::m_frame_size.width, (ushort)Dataset::m_frame_size.height, noise);
    Canvas background((ushort)Dataset::m_frame_size.width, (ushort)Dataset::m_frame_size.height, objectNoise);

    achterbahn = Achterbahn("rectangle_long", start_point_1);
    achterbahn.process(Dataset::m_frame_size);

    Ramp ramp = Ramp("rectangle_long", start_point_1);
    ramp.process(Dataset::m_frame_size);

    objectMetaDataList.at(0) = achterbahn;
    objectMetaDataList.at(0).setObjectShape(circle_obj1);
    objectMetaDataList.at(0).setCppData();
    m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(0));

    achterbahn = Achterbahn("random_object", start_point_2);
    achterbahn.process(Dataset::m_frame_size);

    NegativeRamp negativeRamp = NegativeRamp("random_object", start_point_2);
    negativeRamp.process(Dataset::m_frame_size);

    objectMetaDataList.at(1) = achterbahn;
    objectMetaDataList.at(1).setObjectShape(circle_obj2);
    objectMetaDataList.at(1).setCppData();
    m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(1));

    cv::Mat tempGroundTruthPosition;
    tempGroundTruthPosition.create(Dataset::m_frame_size, CV_32FC3);
    assert(tempGroundTruthPosition.channels() == 3);
    tempGroundTruthPosition = cv::Scalar::all(255);

    cv::Mat tempGroundTruthPosition_2;
    tempGroundTruthPosition_2.create(Dataset::m_frame_size, CV_32FC3);
    assert(tempGroundTruthPosition_2.channels() == 3);
    tempGroundTruthPosition_2 = cv::Scalar::all(255);


    std::map<std::string, double> time_map = {{"generate_single_scene_image", 0},
                                              {"generate_all_scene_image",    0}};


    cv::Mat image_data_and_shape;
    cv::Mat depth_data_and_shape;

    for (ushort current_frame_index = 0; current_frame_index < Dataset::MAX_GENERATION_DATASET; current_frame_index++) {

        // TODO - when the variable overflows.

        //draw new ground truth image.
        cv::Mat tempGroundTruthImage = canvas.getImage().clone();
        cv::Mat tempGroundTruthDepthImage(Dataset::m_frame_size, CV_8UC1, cv::Scalar(255));
        cv::Mat base_image_for_circles(Dataset::m_frame_size, CV_8UC3);
        cv::randu(base_image_for_circles, 0, 0);
        //tempGroundTruthDepthImage = cv::Mat::zeros(Dataset::m_frame_size, CV_8UC1);

        assert((tempGroundTruthImage.channels() == 3 ) && ( tempGroundTruthDepthImage.channels() == 1 ));

        char file_name_image[50], file_name_depth_image[50], sensor_index_folder_suffix[50];
        sprintf(sensor_index_folder_suffix, "%02d", m_sensorGroupCount);
        sprintf(file_name_image, "000%03d_10.png", current_frame_index);
        sprintf(file_name_depth_image, "depth_000%03d_10.png", current_frame_index);
        std::basic_string<char> base_image_file_with_path = GroundTruthScene::m_ground_truth_generate_path.string() + "/" + file_name_image; //+ file_name_image;
        std::basic_string<char> input_image_file_with_path = GroundTruthScene::m_ground_truth_generate_path.string() + "_" + sensor_index_folder_suffix + "/" + file_name_image; //+ file_name_image;
        std::basic_string<char> input_image_depth_file_with_path = GroundTruthScene::m_ground_truth_generate_path.string() + "_" + sensor_index_folder_suffix + "/" + file_name_depth_image; //+ "/" +  file_name_depth;

        cv::Mat result(Dataset::m_frame_size, CV_8UC3);
        result = cv::Scalar::all(0);

        for (unsigned obj_index = 0; obj_index < m_ptr_customObjectMetaDataList.size(); obj_index++) {


            std::string position_image_file_with_path = GroundTruthScene::m_ground_truth_generate_path.string() + '_' + std::to_string(m_sensorGroupCount)+ "/" + file_name_image;

            image_data_and_shape = m_ptr_customObjectMetaDataList.at(obj_index)->getObjectShape().getImage().clone();
            depth_data_and_shape = m_ptr_customObjectMetaDataList.at(obj_index)->getObjectShape().getDepthImage().clone();

            //cv::imshow("con", image_data_and_shape);
            //cv::waitKey(0);

            cv::Mat binary_image, gray_image;
            std::vector<std::vector<cv::Point> > contours;
            cv::cvtColor(image_data_and_shape, gray_image, CV_BGR2GRAY);
            cv::threshold(gray_image, binary_image, 128, 255, CV_THRESH_BINARY); // invert and see what happens
            cv::findContours(binary_image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE,
                             cv::Point(cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                     current_frame_index).m_object_location_camera_px.location_x_px),
                                       cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                               current_frame_index).m_object_location_camera_px.location_y_px)));

            //cv::imshow("con", binary_image);
            //cv::waitKey(0);
            //Draw the contours
            cv::Mat contourImage(binary_image.size(), CV_8UC3, cv::Scalar(0,0,0));
            for (size_t idx = 0; idx < contours.size(); idx++) {
                //cv::drawContours(contourImage, contours, idx, cv::Scalar(255,255,255));
                break;
            }
            //cv::imshow("con", contourImage);
            //cv::waitKey(0);
            if ((!m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).occluded )) {

                for (size_t idx = 0; idx < contours.size(); idx++) {
                    //cv::drawContours(tempGroundTruthImage, contours, idx, cv::Scalar(0,0,0));
                    break;
                }
                cv::Scalar color;
                unsigned char depth;
                //cv::fillConvexPoly(tempGroundTruthDepthImage, contours.at(0), cv::Scalar(255,0,0));
                if ( obj_index == 0 ) {
                    color = cv::Scalar(1,1,1);
                    depth = 200;
                } else {
                    color = cv::Scalar(1,1,1);
                    depth = 11;
                }

                cv::circle(result, cv::Point(cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_location_camera_px.cog_px.x),cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_location_camera_px.cog_px.y)), 30, color, CV_FILLED);

                /*
                image_data_and_shape.copyTo(tempGroundTruthImage(
                        cv::Rect(cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                current_frame_index).m_object_location_camera_px.location_x_px),
                                 cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                         current_frame_index).m_object_location_camera_px.location_y_px),
                                 cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_dimension_camera_px.width_px),
                                 cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_dimension_camera_px.height_px))));


                 depth_data_and_shape.copyTo(tempGroundTruthDepthImage(
                        cv::Rect(cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                current_frame_index).m_object_location_camera_px.location_x_px),
                                 cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                         current_frame_index).m_object_location_camera_px.location_y_px),
                                 cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_dimension_camera_px.width_px),
                                 cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_dimension_camera_px.height_px))));
                */
                cv::circle(tempGroundTruthDepthImage, cv::Point(cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_location_camera_px.cog_px.x),cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_location_camera_px.cog_px.y)), 30, cv::Scalar_<unsigned char>(depth), CV_FILLED);

                printf("! WRITE %u %u %u !\n", tempGroundTruthDepthImage.at<unsigned char>(cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_location_camera_px.cog_px.y),cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_location_camera_px.cog_px.x)), cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_location_camera_px.cog_px.x), cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_location_camera_px.cog_px.y));

            }
            else {
                std::cout << "unable to generate, since object is occluded" << std::endl;
            }
        }

        cv::Mat final = tempGroundTruthImage.clone();
        tempGroundTruthImage = background.getImage().mul(result);
        tempGroundTruthImage.copyTo(final, result);

        //cv::namedWindow("render", CV_WINDOW_AUTOSIZE);
        //cv::imshow("render", tempGroundTruthImage);
        //cv::waitKey(0);
        cv::imwrite(input_image_file_with_path, final);
        cv::imwrite(input_image_depth_file_with_path, tempGroundTruthDepthImage);

        // Validate
        cv::Mat depth_02_frame = cv::imread(input_image_depth_file_with_path, CV_LOAD_IMAGE_GRAYSCALE);

        for (unsigned obj_index = 0; obj_index < m_ptr_customObjectMetaDataList.size(); obj_index++) {
            printf("! READ %u %u %u !\n", depth_02_frame.at < unsigned
            char > (cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_location_camera_px.cog_px.y), cvRound(
                    m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                            current_frame_index).m_object_location_camera_px.cog_px.x)), cvRound(
                    m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                            current_frame_index).m_object_location_camera_px.cog_px.x), cvRound(
                    m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                            current_frame_index).m_object_location_camera_px.cog_px.y));
        }
    }

    std::cout << std::endl;

}
