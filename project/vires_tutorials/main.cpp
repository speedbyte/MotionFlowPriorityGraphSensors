//#include "ImageUtils.h"
#include <boost/bind.hpp>
#include "RDBHandler.hh"

#include <sys/ipc.h>
#include <sys/shm.h>

#include <boost/filesystem.hpp>

#include <iostream>

int main(int argc, char *argv[]) {
    std::cout << "start my program";
    std::string m_server;
    int m_port;
    int m_sensor_port;
    bool m_triggers;
    int m_frames_to_read;
    bool m_write_ts_gt;
    boost::filesystem::path m_ts_gt_out_dir;


    return 0;
}


