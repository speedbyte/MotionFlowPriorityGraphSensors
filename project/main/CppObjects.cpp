//
// Created by veikas on 26.06.18.
//


#include <memory>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include "Noise.h"
#include "ObjectMetaData.h"
#include "CppObjects.h"

void CppObjects::process(std::unique_ptr<Noise> &noise) {

    Achterbahn achterbahn;
    ColorfulNoise colorfulNoise;

    std::unique_ptr<Noise> objectNoise = std::make_unique<ColorfulNoise>();
    cv::Mat tempGroundTruthImageBase;

    Canvas canvas((ushort)Dataset::getFrameSize().width, (ushort)Dataset::getFrameSize().height, noise);
    Rectangle rectangle((ushort)Dataset::getFrameSize().width, (ushort)Dataset::getFrameSize().height, objectNoise); // width, height

    tempGroundTruthImageBase = canvas.get().clone();

    achterbahn = Achterbahn(rectangle, "rectangle_long", 60);
    achterbahn.process(Dataset::getFrameSize());
    objectMetaDataList.at(0) = achterbahn;
    m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(0));

    m_ptr_customObjectMetaDataList.at(0)->setObjectShape(rectangle);

    achterbahn = Achterbahn(rectangle, "random_object", 120);
    achterbahn.process(Dataset::getFrameSize());
    objectMetaDataList.at(1) = achterbahn;
    m_ptr_customObjectMetaDataList.push_back(&objectMetaDataList.at(1));

    m_ptr_customObjectMetaDataList.at(1)->setObjectShape(rectangle);

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

    char file_name_image[50];

    cv::Mat image_data_and_shape;

    //draw new ground truth image.
    cv::Mat tempGroundTruthImage;
    tempGroundTruthImage.create(Dataset::getFrameSize(), CV_32FC3);
    assert(tempGroundTruthImage.channels() == 3);

    for (ushort current_frame_index = 0; current_frame_index < MAX_ITERATION_RESULTS; current_frame_index++) {

        sprintf(file_name_image, "000%03d_10.png", current_frame_index);
        std::string input_image_file_with_path = m_generatepath.string() + "_" + std::to_string(m_sensorGroupCount) + "/" + file_name_image; //+ file_name_image;

        tempGroundTruthImage = tempGroundTruthImageBase.clone();

        for (unsigned obj_index = 0; obj_index < m_ptr_customObjectMetaDataList.size(); obj_index++) {

            std::string position_image_file_with_path = m_generatepath.string() + '_' + std::to_string(m_sensorGroupCount)+ "/" + file_name_image;

            image_data_and_shape = m_ptr_customObjectMetaDataList.at(obj_index)->getObjectShape().get().clone();
            image_data_and_shape = image_data_and_shape.rowRange(0, cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_dimension_camera_px.height_px)).colRange(0, cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                    current_frame_index).m_object_dimension_camera_px.width_px));


            image_data_and_shape.copyTo(tempGroundTruthImage(
                    cv::Rect(cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                            current_frame_index).m_object_location_camera_px.cog_px.x),
                             cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                     current_frame_index).m_object_location_camera_px.cog_px.y),
                             cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_dimension_camera_px.width_px),
                             cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_dimension_camera_px.height_px))));


            if ((!m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).occluded )) {

                image_data_and_shape.copyTo(tempGroundTruthImage(
                        cv::Rect(cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                current_frame_index).m_object_location_camera_px.cog_px.x),
                                 cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(
                                         current_frame_index).m_object_location_camera_px.cog_px.y),
                                 cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_dimension_camera_px.width_px),
                                 cvRound(m_ptr_customObjectMetaDataList.at(obj_index)->getAll().at(current_frame_index).m_object_dimension_camera_px.height_px))));

            }
        }

        cv::imwrite(input_image_file_with_path, tempGroundTruthImage);

    }

}
