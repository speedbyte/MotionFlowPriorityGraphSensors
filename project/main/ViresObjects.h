//
// Created by veikas on 10.02.18.
//

#ifndef MAIN_VIRESOBJECTS_H
#define MAIN_VIRESOBJECTS_H

#include <iostream>
#include <unistd.h>
#include <boost/tuple/tuple.hpp>

class ViresObjects {

private:
    std::vector<boost::tuple<std::string, std::string, ushort > > sensor_group;

    unsigned current_frame_index;

    unsigned int mShmKey;      // key of the SHM segment


public:

    ViresObjects(unsigned int shmKey): mShmKey(shmKey) {}

    ViresObjects() {}

    void configureSensor(const int port_number_camera_sensor_data, const int port_number_usk_sensor_data, const int port_number_inertial_sensor_data,
    std::string module_manager_libModuleSensor_CameraTemplate, std::string module_manager_libModuleSensor_PerfectTemplate, std::string module_manager_libModuleSensor_PerfectInertialTemplate) {


        std::string module_manager_libModuleCameraSensor;
        std::string module_manager_libModulePerfectSensor;
        std::string module_manager_libModulePerfectSensorInertial;

        std::string to_replace;

        std::string::size_type position;

        ///Start sensor
        ///--------------------------
        module_manager_libModuleCameraSensor = module_manager_libModuleSensor_CameraTemplate;

        to_replace = std::to_string(65535);
        position = module_manager_libModuleCameraSensor.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModuleCameraSensor.replace(position, to_replace.length(), std::to_string(port_number_camera_sensor_data));
        }

        sensor_group.push_back(boost::make_tuple(module_manager_libModuleCameraSensor, "suffix", port_number_camera_sensor_data));

        ///--------------------------

        module_manager_libModulePerfectSensor = module_manager_libModuleSensor_PerfectTemplate;
        //port number
        to_replace = std::to_string(65535);
        position = module_manager_libModulePerfectSensor.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModulePerfectSensor.replace(position, to_replace.length(), std::to_string(port_number_usk_sensor_data));
        }

        //sensorlibrary
        to_replace = "libModuleCameraSensor";
        position = module_manager_libModulePerfectSensor.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModulePerfectSensor.replace(position, to_replace.length(), "libModulePerfectSensor");
        }

        //sensor name
        to_replace = "Sensor_MM";
        position = module_manager_libModulePerfectSensor.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModulePerfectSensor.replace(position, to_replace.length(), "Sensor_MM_Perfect");
        }

        sensor_group.push_back(boost::make_tuple(module_manager_libModulePerfectSensor, "suffix", port_number_usk_sensor_data));

        ///--------------------------

        module_manager_libModulePerfectSensorInertial = module_manager_libModuleSensor_PerfectInertialTemplate;
        //port number
        to_replace = std::to_string(65535);
        position = module_manager_libModulePerfectSensorInertial.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModulePerfectSensorInertial.replace(position, to_replace.length(), std::to_string(port_number_inertial_sensor_data));
        }

        //sensorlibrary
        to_replace = "libModuleCameraSensor";
        position = module_manager_libModulePerfectSensorInertial.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModulePerfectSensorInertial.replace(position, to_replace.length(), "libModulePerfectSensor");
        }

        //sensor name
        to_replace = "Sensor_MM";
        position = module_manager_libModulePerfectSensorInertial.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModulePerfectSensorInertial.replace(position, to_replace.length(), "Sensor_MM_PerfectInertial");
        }

        //sensor coordinate
        to_replace = "usk";
        position = module_manager_libModulePerfectSensorInertial.find(to_replace);
        if ( position != std::string::npos) {
            module_manager_libModulePerfectSensorInertial.replace(position, to_replace.length(), "inertial");
        }

        sensor_group.push_back(boost::make_tuple(module_manager_libModulePerfectSensorInertial, "suffix", port_number_inertial_sensor_data));

        ///End sensor

    }

    const std::vector<boost::tuple< std::string, std::string, ushort > >  &getSensorConfiguration() {
        return sensor_group;
    }

    const unsigned int &getShmKey() {
        return mShmKey;
    }

};


#endif //MAIN_VIRESOBJECTS_H
