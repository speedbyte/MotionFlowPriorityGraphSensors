


#include "GroundTruthScene.h"

void GroundTruthSceneInternal::generate_gt_scene(void) {

    /*
    cv::RNG rng(-1);
    for ( unsigned i = 0 ; i < MAX_ITERATION_THETA; i++ ) {
        float a        = (float) rng.uniform(100., 1000.);
        float b        = (float) rng.uniform(100., 300.);
        cv::Point2f points(a,b);
    }
*/
    std::unique_ptr<Noise> colorfulNoise = std::make_unique<ColorfulNoise>();

    if (m_environment == "blue_sky") {

        cppObjects.push_back(CppObjects(0));
        cppObjects.push_back(CppObjects(1));

        if (m_regenerate_yaml_file) {

            for (ushort sensor_group_index = 0; sensor_group_index < m_generation_sensor_list.size(); sensor_group_index++ ) {

                std::cout << "generate_gt_scene at " << m_groundtruthpath.string() << " for " << sensor_group_index << std::endl;

                BlackNoise blackNoise;
                std::unique_ptr<Noise> noise;
                if (m_environment == "night") {
                    BlackNoise blackNoise_;
                    noise = std::make_unique<BlackNoise>(blackNoise_);
                } else {
                    WhiteNoise whiteNoise_;
                    noise = std::make_unique<WhiteNoise>(whiteNoise_);
                }

                //noise = std::make_unique<BlackNoise>(blackNoise);
                cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).process(noise, sensor_group_index);

            }

            //save_groundtruth_data

        } else { // do not genreate yaml file
            for (ushort sensor_group_index = 0; sensor_group_index < m_generation_sensor_list.size(); sensor_group_index++ ) {

                cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).readPositionFromFile("cpp_");
                cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).calcBBFrom3DPosition("cpp_");
            }

            startEvaluating(colorfulNoise);
        }

    }

}


void GroundTruthSceneInternal::save_gt_scene_data() {

    try {

        for ( ushort sensor_group_index = 0 ; sensor_group_index < m_generation_sensor_list.size() ; sensor_group_index++) {

            //cppObjects.at(m_generation_sensor_list.at(sensor_group_index)).closeAllFileHandles();

            //cppObjects.at(m_generation_sensor_list.at(sensor_group_index)).readObjectData();
            //cppObjects.at(m_generation_sensor_list.at(sensor_group_index)).readSensorData();

            // writePosition deletes the file before generating yaml file
            cppObjects.at(m_generation_sensor_list.at(sensor_group_index)).writePositionInYaml("cpp_");
            //system("diff ../position_vires_original_15_65.yml ../position_vires_0.yml");

            cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).validate_depth_images();

            cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).generateFrameDifferenceImage(m_ground_truth_generate_path, m_ground_truth_framedifference_path);
            cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).generate_edge_images(m_ground_truth_generate_path);


        }

        std::cout << "Validate ground truth generation completed" << std::endl;

    }
    catch (...) {
        std::cerr << "VTD Generation complete, but error in generating images" << std::endl;
        //stopSimulation();
    }
}


void GroundTruthSceneInternal::startEvaluating(std::unique_ptr<Noise> &noise) {


    for (ushort obj_index = 0; obj_index < cppObjects.at(m_evaluation_sensor_list.at(0)).get_ptr_customObjectMetaDataList().size(); obj_index++) {

        GroundTruthObjects gt_obj;

        // how many objects were found. The assumption is that in each frame, the number of objects would be constant.

        for (ushort sensor_group_index = 0; sensor_group_index < m_evaluation_sensor_list.size(); sensor_group_index++ ) {

            std::cout << "send object name " << cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index)->getObjectName() << std::endl;

            if ( sensor_group_index == 0 ) {
                gt_obj = GroundTruthObjects(cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index)->getObjectShape(),
                                            cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index)->getObjectStartPoint(), noise,
                                            cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index)->getObjectName());
                m_list_gt_objects.push_back(gt_obj);
            }
            // each object is monitored by n sensor group.
            m_list_gt_objects.at(obj_index).beginGroundTruthGeneration(sensor_group_index, *cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index));

        }
        if ( m_evaluation_sensor_list.size() > 1 ) {
            m_list_gt_objects.at(obj_index).generate_combined_sensor_data();
        }

    }

    for (ushort sen_index = 0; sen_index < cppObjects.at(m_evaluation_sensor_list.at(0)).get_ptr_customSensorMetaDataList().size(); sen_index++) {

        // how many sensors were found. The assumption is that in each frame, the number of sensors would be constant.

        for (ushort sensor_group_index = 0; sensor_group_index < m_evaluation_sensor_list.size(); sensor_group_index++ ) {

            Sensors gt_sen(cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customSensorMetaDataList().at(0)->getSensorStartPoint(), noise,
                           cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customSensorMetaDataList().at(0)->getSensorName());
            m_list_gt_sensors.push_back(gt_sen);
            // each sensor is monitored by n sensor group.

            m_list_gt_sensors.at(sen_index).beginGroundTruthGeneration(*cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customSensorMetaDataList().at(sen_index));

        }

    }

}

