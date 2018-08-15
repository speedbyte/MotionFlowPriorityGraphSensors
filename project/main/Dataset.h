//
// Created by veikas on 26.01.18.
//

#ifndef MAIN_DATASET_H
#define MAIN_DATASET_H

#include <opencv2/core/types.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <map>



class Dataset {

public:

    static cv::Size_<unsigned> m_frame_size;
    static ushort m_depth;
    static ushort m_cn;

    static ushort ITERATION_START_POINT;
    static ushort ITERATION_END_POINT;
    static ushort MAX_ITERATION_RESULTS;
    static ushort MAX_ITERATION_DATASET;
    static ushort MAX_ITERATION_GT_SCENE_GENERATION_DATASET;

    static boost::filesystem::path m_dataset_basepath;
    static boost::filesystem::path  m_dataset_gtpath;
    static boost::filesystem::path  m_dataset_resultpath;

    static std::map<std::string, bool> m_dataprocessing_map;

    static void fillDataset(cv::Size_<unsigned> frame_size, ushort depth, ushort cn, std::string dataset_path,
            std::string unterordner, std::string resultordner, ushort start, ushort stop, std::map<std::string, bool> dataprocessing_map);
    static const cv::Size_<unsigned> getFrameSize()  ;

    static const ushort getMakeType();

};


#endif //MAIN_DATASET_H
