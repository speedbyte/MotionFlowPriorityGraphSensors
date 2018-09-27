


#include "GenerateGroundTruthScene.h"

void GroundTruthSceneInternal::generate_gt_scene(void) {

    /*
    cv::RNG rng(-1);
    for ( unsigned i = 0 ; i < MAX_ITERATION_THETA; i++ ) {
        float a        = (float) rng.uniform(100., 1000.);
        float b        = (float) rng.uniform(100., 300.);
        cv::Point2f points(a,b);
    }
*/

    cppObjects.push_back(CppObjects(0));
    cppObjects.push_back(CppObjects(1));

    for (ushort sensor_group_index = 0; sensor_group_index < m_generation_sensor_list.size(); sensor_group_index++ ) {

        std::cout << "generate_gt_scene at " << m_groundtruthpath.string() << " for " << sensor_group_index << std::endl;

        std::unique_ptr<Noise> background_noise;
        if (m_environment == "night") {
            StaticNoise<0> blackStaticNoise;
            background_noise = std::make_unique<StaticNoise<0>>(blackStaticNoise);
        }  else if (m_environment == "blue_sky") {
            StaticNoise<255> whiteStaticNoise;
            background_noise = std::make_unique<StaticNoise<255>>(whiteStaticNoise);
        }

        cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).process(background_noise, sensor_group_index);
    }

}


void GroundTruthSceneInternal::read_gt_scene_data() {

    std::unique_ptr<Noise> noNoise = std::make_unique<NoNoise>();

    cppObjects.push_back(CppObjects(0));
    cppObjects.push_back(CppObjects(1));

    for (ushort sensor_group_index = 0; sensor_group_index < m_generation_sensor_list.size(); sensor_group_index++ ) {

        cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).readPositionFromFile("cpp_");
        cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).calcBBFrom3DPosition("cpp_");
    }

}


void GroundTruthSceneInternal::write_gt_scene_data() {

    try {

        for ( ushort sensor_group_index = 0 ; sensor_group_index < m_generation_sensor_list.size() ; sensor_group_index++) {

            cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).validate_depth_images();
            cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).generate_frame_difference_images();
            cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).generate_edge_images();

            //cppObjects.at(m_generation_sensor_list.at(sensor_group_index)).closeAllFileHandles();

            //cppObjects.at(m_generation_sensor_list.at(sensor_group_index)).readObjectData();
            //cppObjects.at(m_generation_sensor_list.at(sensor_group_index)).readSensorData();

            // writePosition deletes the file before generating yaml file
            cppObjects.at(m_generation_sensor_list.at(sensor_group_index)).writePositionInYaml("cpp_");
            //system("diff ../position_vires_original_15_65.yml ../position_vires_0.yml");



        }

        std::cout << "Validate ground truth generation completed" << std::endl;

    }
    catch (...) {
        std::cerr << "VTD Generation complete, but error in generating images" << std::endl;
        exit(-1);
    }
}


void GroundTruthSceneInternal::convert_sensor_image_to_object_level(std::unique_ptr<Noise> &noise, std::vector<GroundTruthObjects> &list_of_gt_objects_base, std::vector<Sensors> &list_of_gt_sensors_base) {


    for (ushort obj_index = 0; obj_index < cppObjects.at(m_evaluation_sensor_list.at(0)).get_ptr_customObjectMetaDataList().size(); obj_index++) {

        GroundTruthObjects gt_obj;

        // how many objects were found. The assumption is that in each frame, the number of objects would be constant.

        for (ushort sensor_group_index = 0; sensor_group_index < m_evaluation_sensor_list.size(); sensor_group_index++ ) {

            std::cout << "send object name " << cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index)->getObjectName() << std::endl;

            if ( sensor_group_index == 0 ) {
                gt_obj = GroundTruthObjects(
                                            cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index)->getObjectStartPoint(), noise,
                                            cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index)->getObjectName());
                list_of_gt_objects_base.push_back(gt_obj);
            }
            // each object is monitored by n sensor group.
            list_of_gt_objects_base.at(obj_index).beginGroundTruthGeneration(sensor_group_index, *cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customObjectMetaDataList().at(obj_index));

        }
        if ( m_evaluation_sensor_list.size() > 1 ) {
            list_of_gt_objects_base.at(obj_index).generate_combined_sensor_data();
        }

    }

    for (ushort sen_index = 0; sen_index < cppObjects.at(m_evaluation_sensor_list.at(0)).get_ptr_customSensorMetaDataList().size(); sen_index++) {

        // how many sensors were found. The assumption is that in each frame, the number of sensors would be constant.

        for (ushort sensor_group_index = 0; sensor_group_index < m_evaluation_sensor_list.size(); sensor_group_index++ ) {

            Sensors gt_sen(cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customSensorMetaDataList().at(0)->getSensorStartPoint(), noise,
                           cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customSensorMetaDataList().at(0)->getSensorName());
            list_of_gt_sensors_base.push_back(gt_sen);
            // each sensor is monitored by n sensor group.

            list_of_gt_sensors_base.at(sen_index).beginGroundTruthGeneration(*cppObjects.at(m_evaluation_sensor_list.at(sensor_group_index)).get_ptr_customSensorMetaDataList().at(sen_index));

        }

    }

}

