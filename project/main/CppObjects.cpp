//
// Created by veikas on 26.06.18.
//


#include <memory>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>
#include "Noise.h"
#include "ObjectMetaData.h"
#include "CppObjects.h"

void CppObjects::process(std::unique_ptr<Noise> &noise) {

    Achterbahn achterbahn;

    std::unique_ptr<Noise> objectNoise = std::make_unique<ColorfulNoise>();
    cv::Mat tempGroundTruthImageBase;

    Canvas canvas((ushort)Dataset::getFrameSize().width, (ushort)Dataset::getFrameSize().height, noise);

    tempGroundTruthImageBase = canvas.getImage().clone();

    achterbahn = Achterbahn("rectangle_long", 100);
    achterbahn.process(Dataset::getFrameSize());

    Ramp ramp = Ramp("rectangle_long", 100);
    ramp.process(Dataset::getFrameSize());

    objectMetaDataList.at(0) = achterbahn;
    m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(0));


    achterbahn = Achterbahn("random_object", 220);
    achterbahn.process(Dataset::getFrameSize());

    NegativeRamp negativeRamp = NegativeRamp("random_object", 100);
    negativeRamp.process(Dataset::getFrameSize());

    objectMetaDataList.at(1) = achterbahn;
    m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(1));


    cv::Mat tempGroundTruthPosition;
    tempGroundTruthPosition.create(Dataset::getFrameSize(), CV_32FC3);
    assert(tempGroundTruthPosition.channels() == 3);
    tempGroundTruthPosition = cv::Scalar::all(255);

    cv::Mat tempGroundTruthPosition_2;
    tempGroundTruthPosition_2.create(Dataset::getFrameSize(), CV_32FC3);
    assert(tempGroundTruthPosition_2.channels() == 3);
    tempGroundTruthPosition_2 = cv::Scalar::all(255);


    std::map<std::string, double> time_map = {{"generate_single_scene_image", 0},
                                              {"generate_all_scene_image",    0}};


    cv::Mat image_data_and_shape;
    cv::Mat depth_data_and_shape;

    //draw new ground truth image.
    cv::Mat tempGroundTruthImage;
    cv::Mat tempGroundTruthDepthImage;

    tempGroundTruthImage.create(Dataset::getFrameSize(), CV_32FC3);
    tempGroundTruthDepthImage.create(Dataset::getFrameSize(), CV_8UC1);

    assert((tempGroundTruthImage.channels() == 3 ) && ( tempGroundTruthDepthImage.channels() == 1 ));

    for (ushort current_frame_index = 0; current_frame_index < MAX_ITERATION_GT_SCENE_GENERATION_DATASET; current_frame_index++) {

        // TO DO - when the variable overflows.

        char file_name_image[50], file_name_depth_image[50], sensor_index_folder_suffix[50];
        sprintf(sensor_index_folder_suffix, "%02d", m_sensorGroupCount);
        sprintf(file_name_image, "000%03d_10.png", current_frame_index);
        sprintf(file_name_depth_image, "depth_000%03d_10.png", current_frame_index);
        std::basic_string<char> input_image_file_with_path = m_generatepath.string() + "_" + sensor_index_folder_suffix + "/" + file_name_image; //+ file_name_image;
        std::basic_string<char> input_image_depth_file_with_path = m_generatepath.string() + "_" + sensor_index_folder_suffix + "/" + file_name_depth_image; //+ "/" +  file_name_depth;

        tempGroundTruthImage = tempGroundTruthImageBase.clone();
        cv::cvtColor(tempGroundTruthImageBase.clone(), tempGroundTruthDepthImage, CV_BGR2GRAY);


        for (unsigned obj_index = 0; obj_index < m_ptr_customObjectMetaDataList.size(); obj_index++) {

            if ( current_frame_index == MAX_ITERATION_GT_SCENE_GENERATION_DATASET ) {
                current_frame_index = 0;
            }
            if ( obj_index == 0 ) {

                Rectangle rectangle((ushort)m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_dimension_camera_px.width_px,
                                    (ushort)m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_dimension_camera_px.height_px, objectNoise, 11); // width, height
                m_ptr_customObjectMetaDataList.at(obj_index)->setObjectShape(rectangle);

            }
            else if ( obj_index == 1 ){

                Rectangle rectangle((ushort)m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_dimension_camera_px.width_px,
                                    (ushort)m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_dimension_camera_px.height_px, objectNoise, 200); // width, height
                m_ptr_customObjectMetaDataList.at(obj_index)->setObjectShape(rectangle);

            }

            std::string position_image_file_with_path = m_generatepath.string() + '_' + std::to_string(m_sensorGroupCount)+ "/" + file_name_image;

            image_data_and_shape = m_ptr_customObjectMetaDataList.at(obj_index)->getObjectShape().getImage().clone();
            depth_data_and_shape = m_ptr_customObjectMetaDataList.at(obj_index)->getObjectShape().getDepth().clone();

            if ((!m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).occluded )) {

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

            }
        }

        //cv::namedWindow("render", CV_WINDOW_AUTOSIZE);
        //cv::imshow("render", tempGroundTruthImage);
        //cv::waitKey(0);
        cv::imwrite(input_image_file_with_path, tempGroundTruthImage);
        cv::imwrite(input_image_depth_file_with_path, tempGroundTruthDepthImage);

    }

}
