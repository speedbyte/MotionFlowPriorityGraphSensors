//
// Created by veikas on 26.01.18.
//

#ifndef MAIN_DATASET_H
#define MAIN_DATASET_H


#include <opencv2/core/types.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

typedef struct {


    cv::Size_<unsigned> m_frame_size;

    ushort m_depth;

    ushort m_cn;

    boost::filesystem::path m_dataset_basepath;

    boost::filesystem::path  m_directory_path_gt;

    boost::filesystem::path  m_directory_path_result;


} DATASET_STRUCT;

static DATASET_STRUCT  datasetStruct;

class Dataset {


private:

    Dataset() {}

public:

    static void fillDataset(cv::Size_<unsigned> frame_size, ushort depth, ushort cn, std::string dataset_path,
            std::string unterordner, std::string resultordner) ;

    const static cv::Size_<unsigned> getFrameSize()  ;

    const static boost::filesystem::path getDatasetPath() ;

    const static boost::filesystem::path getGroundTruthPath()  ;

    const static boost::filesystem::path getResultPath();

    const static ushort getDepth();

    const static ushort getChannel();

    const static ushort getMakeType();




};


#endif //MAIN_DATASET_H
