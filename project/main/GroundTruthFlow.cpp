

#include <vector>
#include <cmath>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>
#include <map>
#include <chrono>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/tuple/tuple.hpp>
#include <png++/png.hpp>

#include "kitti/log_colormap.h"
#include <kitti/mail.h>
#include <kitti/io_flow.h>
#include <vires/vires_common.h>

#include "datasets.h"
#include "GroundTruthFlow.h"
#include "kbhit.h"

#include <unordered_map>
#include <bits/unordered_map.h>

#include "ObjectTrajectory.h"
#include "Dataset.h"


//Creating a movement path. The path is stored in a x and y vector

using namespace std::chrono;

GroundTruthScene::GroundTruthScene(Dataset dataset) {

    m_dataset = dataset;

}

GroundTruthFlow::GroundTruthFlow(Dataset dataset) {

    m_dataset = dataset;

}


void GroundTruthScene::prepare_directories_gt_scene() {

    if (!m_dataset.getBasePath().compare(CPP_DATASET_PATH) || !m_dataset.getBasePath().compare(VIRES_DATASET_PATH) ) {

        // delete ground truth image and ground truth flow directories
        if ( boost::filesystem::exists(m_dataset.getInputPath()) ) {
            system(("rm -rf " + m_dataset.getInputPath().string()).c_str()); // data/stereo_flow/
        }

        // create base directories
        boost::filesystem::create_directories(m_dataset.getInputPath().string());
        std::cout << "Creating GT Scene directories" << std::endl;
        boost::filesystem::create_directories(m_dataset.getInputPath().string() + "/image_02");

        std::cout << "Ending GT Scene directories" << std::endl;
    }
}

void GroundTruthFlow::prepare_directories_gt_flow() {

    char char_dir_append[20];

    if (!m_dataset.getBasePath().compare(CPP_DATASET_PATH) || !m_dataset.getBasePath().compare(VIRES_DATASET_PATH) ) {

        std::cout << "Creating GT Flow directories" << std::endl;
        // create flow directories
        for (int i = 1; i < 10; ++i) {
            sprintf(char_dir_append, "%02d", i);
            boost::filesystem::create_directories(m_dataset.getInputPath().string() + "/flow_occ_" + char_dir_append);
        }
        std::cout << "Ending GT Flow directories" << std::endl;
    }
}


void GroundTruthScene::generate_gt_scene(void) {

    prepare_directories_gt_scene();

    /*
     * First create an object with an shape
     * Then define the object trajectory
     * Then copy the object shape on the object trajectory points
     * Then store the image in the ground truth image folder
     *
     * Then extrapolate the object trajectory with the above shape
     *
     * Then store the flow information in the flow folder
     */

    cv::Mat tempGroundTruthImage;
    tempGroundTruthImage.create(m_dataset.getFrameSize(), CV_8UC3);
    assert(tempGroundTruthImage.channels() == 3);

    const ushort start=60;

    std::map<std::string, double> time_map = {{"generate",0}, {"ground truth", 0}};

    std::cout << "ground truth images will be stored in " << m_dataset.getInputPath().string() << std::endl;

    auto tic = steady_clock::now();
    auto toc = steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();

    //Initialization
    ushort current_index = 0;
    //for moving the objects later
    char folder_name_flow[50];
    char file_name_image[50];

    current_index = start;



    Rectangle rectangle;
    Achterbahn achterbahn;

    ObjectProperties shape1 = ObjectProperties(m_dataset, rectangle, achterbahn);

    std::vector<cv::Point2i> trajectory_points = shape1.getTrajectoryPoints();
    cv::Mat shape = shape1.getShape();

    for (ushort frame_count=0; frame_count < MAX_ITERATION_GT; frame_count++) {

        sprintf(file_name_image, "000%03d_10.png", frame_count);
        std::string input_image_file_with_path = m_dataset.getInputPath().string() + "/image_02/" + file_name_image;

        tempGroundTruthImage = cv::Scalar::all(0);

        //draw new ground truth image.

        shape.copyTo(tempGroundTruthImage(
                cv::Rect(trajectory_points.at(current_index).x, trajectory_points.at
                                 (current_index).y, object_width,
                         object_height)));
        toc = steady_clock::now();
        time_map["generate"] = duration_cast<milliseconds>(toc - tic).count();
        cv::imwrite(input_image_file_with_path, tempGroundTruthImage);
        current_index++;
        if ((current_index) >= trajectory_points.size() ) {
            current_index = 0;
        }
    }

    current_index = start;
    cv::Point2i l_pixel_position, l_pixel_movement;

    std::vector<std::pair<cv::Point2i, cv::Point2i> > m_flowvector_with_coordinate_gt;

    for (ushort frame_count=0; frame_count < MAX_ITERATION_GT; frame_count++) {
        // The first frame is the reference frame, hence it is skipped
        if ( frame_count > 0 ) {
            //If we are at the end of the path vector, we need to reset our iterators
            if ((current_index) >= trajectory_points.size()) {
                current_index = 0;
                l_pixel_movement.x = trajectory_points.at(current_index).x - trajectory_points.at(trajectory_points.size() - 1).x;
                l_pixel_movement.y = trajectory_points.at(current_index).y - trajectory_points.at(trajectory_points.size() - 1).y;
                l_pixel_position = trajectory_points.at(current_index);
            } else {
                l_pixel_movement.x = trajectory_points.at(current_index).x - trajectory_points.at(current_index - (ushort) 1).x;
                l_pixel_movement.y = trajectory_points.at(current_index).y - trajectory_points.at(current_index - (ushort) 1).y;
                l_pixel_position = trajectory_points.at(current_index);
            }

            printf("%u, %u , %u, %u, %u, %d, %d\n", frame_count, start, current_index, l_pixel_position.x, l_pixel_position.y,
                   l_pixel_movement.x, l_pixel_movement.y);

            // make m_flowvector_with_coordinate_gt with smallest resolution.
            m_flowvector_with_coordinate_gt.push_back(std::make_pair(l_pixel_position, l_pixel_movement));
        }
        else {
            m_flowvector_with_coordinate_gt.push_back(std::make_pair(cv::Point2i(0,0), cv::Point2i(0,0)));
        }
        current_index++;
    }

    for ( int frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++ ){
        int temp_flow_x = 0, temp_flow_y = 0 ;
        sprintf(folder_name_flow, "flow_occ_%02d", frame_skip);
        cv::FileStorage fs;
        fs.open(m_dataset.getInputPath().string() + "/" + folder_name_flow + "/" + "gt_flow.yaml",
                cv::FileStorage::WRITE);
        std::cout << "creating flow files for frame_skip " << frame_skip << std::endl;
        for (ushort frame_count=1; frame_count < MAX_ITERATION_GT; frame_count++) {
            // The first frame is the reference frame.
            //the below code has to go through consecutive frames
            if ( frame_count%frame_skip != 0 ) {
                temp_flow_x += m_flowvector_with_coordinate_gt.at(frame_count).second.x;
                temp_flow_y += m_flowvector_with_coordinate_gt.at(frame_count).second.y;
                continue;
            }
            temp_flow_x += m_flowvector_with_coordinate_gt.at(frame_count).second.x;
            temp_flow_y += m_flowvector_with_coordinate_gt.at(frame_count).second.y;
            fs << "frame_count" << frame_count;
            sprintf(file_name_image, "000%03d_10.png", frame_count);
            std::string temp_gt_flow_image_path = m_dataset.getInputPath().string() + "/" + folder_name_flow + "/"
                                                  + file_name_image;
            ObjectFlow point(m_dataset);
            point.extrapolate_flowpoints( fs, cv::Point2i(m_flowvector_with_coordinate_gt.at(frame_count).first.x,
                                                          m_flowvector_with_coordinate_gt.at
                                                                  (frame_count).first.y),
                                          object_width, object_height, temp_flow_x, temp_flow_y, temp_gt_flow_image_path );
            temp_flow_x = 0, temp_flow_y = 0 ;
        }
        fs.release();
    }

    // plotVectorField (F_gt_write,m_base_directory_path_image_out.parent_path().string(),file_name);
    toc_all = steady_clock::now();
    time_map["ground truth"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth generation time - " << time_map["ground truth"]  << "ms" << std::endl;

}


void GroundTruthFlow::generate_gt_flow(void) {

    prepare_directories_gt_flow();

    /*
     * First create an object with an shape
     * Then define the object trajectory
     * Then copy the object shape on the object trajectory points
     * Then store the image in the ground truth image folder
     *
     * Then extrapolate the object trajectory with the above shape
     *
     * Then store the flow information in the flow folder
     */

    cv::Mat tempGroundTruthImage;
    tempGroundTruthImage.create(m_dataset.getFrameSize(), CV_8UC3);
    assert(tempGroundTruthImage.channels() == 3);

    const ushort start=60;

    std::map<std::string, double> time_map = {{"generate",0}, {"ground truth", 0}};

    std::cout << "ground truth images will be stored in " << m_dataset.getInputPath().string() << std::endl;

    auto tic = steady_clock::now();
    auto toc = steady_clock::now();
    auto tic_all = steady_clock::now();
    auto toc_all = steady_clock::now();

    //Initialization
    ushort current_index = 0;
    //for moving the objects later
    char folder_name_flow[50];
    char file_name_image[50];

    current_index = start;



    Rectangle rectangle;
    Achterbahn achterbahn;

    ObjectProperties shape1 = ObjectProperties(m_dataset, rectangle, achterbahn);

    std::vector<cv::Point2i> trajectory_points = shape1.getTrajectoryPoints();
    cv::Mat shape = shape1.getShape();

    for (ushort frame_count=0; frame_count < MAX_ITERATION_GT; frame_count++) {

        sprintf(file_name_image, "000%03d_10.png", frame_count);
        std::string input_image_file_with_path = m_dataset.getInputPath().string() + "/image_02/" + file_name_image;

        tempGroundTruthImage = cv::Scalar::all(0);

        //draw new ground truth image.

        shape.copyTo(tempGroundTruthImage(
                cv::Rect(trajectory_points.at(current_index).x, trajectory_points.at
                                 (current_index).y, object_width,
                         object_height)));
        toc = steady_clock::now();
        time_map["generate"] = duration_cast<milliseconds>(toc - tic).count();
        cv::imwrite(input_image_file_with_path, tempGroundTruthImage);
        current_index++;
        if ((current_index) >= trajectory_points.size() ) {
            current_index = 0;
        }
    }

    current_index = start;
    cv::Point2i l_pixel_position, l_pixel_movement;

    std::vector<std::pair<cv::Point2i, cv::Point2i> > m_flowvector_with_coordinate_gt;

    for (ushort frame_count=0; frame_count < MAX_ITERATION_GT; frame_count++) {
        // The first frame is the reference frame, hence it is skipped
        if ( frame_count > 0 ) {
            //If we are at the end of the path vector, we need to reset our iterators
            if ((current_index) >= trajectory_points.size()) {
                current_index = 0;
                l_pixel_movement.x = trajectory_points.at(current_index).x - trajectory_points.at(trajectory_points.size() - 1).x;
                l_pixel_movement.y = trajectory_points.at(current_index).y - trajectory_points.at(trajectory_points.size() - 1).y;
                l_pixel_position = trajectory_points.at(current_index);
            } else {
                l_pixel_movement.x = trajectory_points.at(current_index).x - trajectory_points.at(current_index - (ushort) 1).x;
                l_pixel_movement.y = trajectory_points.at(current_index).y - trajectory_points.at(current_index - (ushort) 1).y;
                l_pixel_position = trajectory_points.at(current_index);
            }

            printf("%u, %u , %u, %u, %u, %d, %d\n", frame_count, start, current_index, l_pixel_position.x, l_pixel_position.y,
                   l_pixel_movement.x, l_pixel_movement.y);

            // make m_flowvector_with_coordinate_gt with smallest resolution.
            m_flowvector_with_coordinate_gt.push_back(std::make_pair(l_pixel_position, l_pixel_movement));
        }
        else {
            m_flowvector_with_coordinate_gt.push_back(std::make_pair(cv::Point2i(0,0), cv::Point2i(0,0)));
        }
        current_index++;
    }

    for ( int frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++ ){
        int temp_flow_x = 0, temp_flow_y = 0 ;
        sprintf(folder_name_flow, "flow_occ_%02d", frame_skip);
        cv::FileStorage fs;
        fs.open(m_dataset.getInputPath().string() + "/" + folder_name_flow + "/" + "gt_flow.yaml",
                cv::FileStorage::WRITE);
        std::cout << "creating flow files for frame_skip " << frame_skip << std::endl;
        for (ushort frame_count=1; frame_count < MAX_ITERATION_GT; frame_count++) {
            // The first frame is the reference frame.
                //the below code has to go through consecutive frames
            if ( frame_count%frame_skip != 0 ) {
                temp_flow_x += m_flowvector_with_coordinate_gt.at(frame_count).second.x;
                temp_flow_y += m_flowvector_with_coordinate_gt.at(frame_count).second.y;
                continue;
            }
            temp_flow_x += m_flowvector_with_coordinate_gt.at(frame_count).second.x;
            temp_flow_y += m_flowvector_with_coordinate_gt.at(frame_count).second.y;
            fs << "frame_count" << frame_count;
            sprintf(file_name_image, "000%03d_10.png", frame_count);
            std::string temp_gt_flow_image_path = m_dataset.getInputPath().string() + "/" + folder_name_flow + "/"
                                         + file_name_image;
            ObjectFlow point(m_dataset);
            point.extrapolate_flowpoints( fs, cv::Point2i(m_flowvector_with_coordinate_gt.at(frame_count).first.x,
                                              m_flowvector_with_coordinate_gt.at
                                         (frame_count).first.y),
                                 object_width, object_height, temp_flow_x, temp_flow_y, temp_gt_flow_image_path );
            temp_flow_x = 0, temp_flow_y = 0 ;
        }
        fs.release();
    }

    // plotVectorField (F_gt_write,m_base_directory_path_image_out.parent_path().string(),file_name);
    toc_all = steady_clock::now();
    time_map["ground truth"] = duration_cast<milliseconds>(toc_all - tic_all).count();
    std::cout << "ground truth generation time - " << time_map["ground truth"]  << "ms" << std::endl;

}


void GroundTruthFlow::generate_gt_image_and_gt_flow_vires() {

    prepare_directories_gt_flow();

    char command[1024];


    sprintf(command,"cd %s; %s",(m_dataset.getBasePath().string() + std::string("../../")).c_str(),"bash "
            "vtdSendandReceive.sh");
    std::cout << command << std::endl;
    system(command);

    std::cout << " I am out of bash" << std::endl;

    //Framework::ViresInterface vi;
    std::string m_server;
    boost::filesystem::path m_ts_gt_out_dir;

    int initCounter = 6;

    // initalize the server variable
    std::string serverName = "127.0.0.1";

    setServer(serverName.c_str());

    fprintf(stderr, "ValidateArgs: key = 0x%x, checkMask = 0x%x, mForceBuffer = %d\n",
            getShmKey(), getCheckMask(), getForceBuffer());

    // open the network connection to the taskControl (so triggers may be sent)
    fprintf(stderr, "creating network connection....\n");
    openNetwork();  // this is blocking until the network has been opened
    openNetwork_GT();



    // now: open the shared memory (try to attach without creating a new segment)
    fprintf(stderr, "attaching to shared memory 0x%x....\n", getShmKey());

    while (!getShmPtr()) {
        openShm();
        usleep(1000);     // do not overload the CPU
    }

    fprintf(stderr, "...attached! Reading now...\n");

    // now check the SHM for the time being
    bool breaking = false;
    int count = 0;
    while (1) {

        // Break out of the loop if the user presses the Esc key
        int c =  kbhit();

        switch (c) {
            case 9:
                breaking = true;
                break;
            default:
                break;
        }

        if ( breaking ) {
            break;
        }

        if ( getSimFrame() > MAX_ITERATION_GT ) {
            breaking = true;
        }

        readNetwork();  // this calls parseRDBMessage() in vires_common.cpp

        if (initCounter <= 0)
            checkShm();

        // has an image arrived or do the first frames need to be triggered
        //(first image will arrive with a certain image_02_frame delay only)
        if (getHaveImage() || (initCounter-- > 0)) {
            sendRDBTrigger();
            std::cout << getSimFrame() << std::endl;
        }
        // ok, reset image indicator
        setHaveImage(0);

        usleep(10000); // sleep for 10 ms
        std::cout << "getting data from VIRES\n";
    }

    sprintf(command,"cd %s; %s",(m_dataset.getBasePath().string() + std::string("../../")).c_str(),"bash vtdStop.sh");
    std::cout << command << std::endl;
    system(command);
}


void GroundTruthFlow::parseStartOfFrame(const double &simTime, const unsigned int &simFrame) {
    fprintf( stderr, "I am in GroundTruthFlow %d\n,", RDB_PKG_ID_START_OF_FRAME );
    fprintf( stderr, "RDBHandler::parseStartOfFrame: simTime = %.3f, simFrame = %d\n", simTime, simFrame );
}

void GroundTruthFlow::parseEndOfFrame( const double & simTime, const unsigned int & simFrame )
{
    fprintf( stderr, "headers %d\n,", RDB_PKG_ID_END_OF_FRAME );
    fprintf( stderr, "RDBHandler::parseEndOfFrame: simTime = %.3f, simFrame = %d\n", simTime, simFrame );
}

void GroundTruthFlow::parseEntry( RDB_OBJECT_CFG_t *data, const double & simTime, const unsigned int & simFrame, const
unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {

    RDB_OBJECT_CFG_t* object = reinterpret_cast<RDB_OBJECT_CFG_t*>(data); /// raw image data
    std::cout << object->type;

}

void GroundTruthFlow::parseEntry( RDB_OBJECT_STATE_t *data, const double & simTime, const unsigned int & simFrame, const
unsigned
short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {

    RDB_OBJECT_STATE_t* object = reinterpret_cast<RDB_OBJECT_STATE_t*>(data); /// raw image data
//        fprintf( stderr, "handleRDBitem: handling object state\n" );
//        fprintf( stderr, "    simTime = %.3lf, simFrame = %d\n", simTime, simFrame );
//        fprintf( stderr, "    object = %s, id = %d\n", data->base.name, data->base.id );
//        fprintf( stderr, "    position = %.3lf / %.3lf / %.3lf\n", data->base.pos.x, data->base.pos.y, data->base
//                .pos.z );
    if ( strcmp(data->base.name, "New Character") == 0) {

        fprintf( stderr, "INDICATOR: %d %.3lf %.3lf %.3lf %.3lf \n" ,
                 simFrame,data->base.pos.x,
                 data->base.pos.y, data->base.geo.dimX, data->base.geo.dimY );
    }

    else if ( strcmp(data->base.name, "New Character01") == 0) {
        fprintf( stderr, "INDICATOR2: %d %.3lf %.3lf %.3lf %.3lf \n" ,
                 simFrame,data->base.pos.x,
                 data->base.pos.y, data->base.geo.dimX, data->base.geo.dimY );
    }
}

void GroundTruthFlow::parseEntry( RDB_IMAGE_t *data, const double & simTime, const unsigned int & simFrame, const
unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {

    if ( !data )
        return;
    fprintf( stderr, "handleRDBitem: image\n" );
    fprintf( stderr, "    simTime = %.3lf, simFrame = %d, mLastShmFrame = %d\n", simTime, simFrame, getLastShmFrame());
    fprintf( stderr, "    width / height = %d / %d\n", data->width, data->height );
    fprintf( stderr, "    dataSize = %d\n", data->imgSize );

    // ok, I have an image:
    setHaveImage(1);
    char* image_data_=NULL;
    RDB_IMAGE_t* image = reinterpret_cast<RDB_IMAGE_t*>(data); /// raw image data

    /// RDB image information of \see image_data_
    RDB_IMAGE_t image_info_;
    memcpy(&image_info_, image, sizeof(RDB_IMAGE_t));

    if (NULL == image_data_) {
        image_data_ = reinterpret_cast<char*>(malloc(image_info_.imgSize));
    } else {
        image_data_ = reinterpret_cast<char*>(realloc(image_data_, image_info_.imgSize));
    }
    // jump data header
    memcpy(image_data_, reinterpret_cast<char*>(image) + sizeof(RDB_IMAGE_t), image_info_.imgSize);

    if ( image_info_.imgSize == image_info_.width*image_info_.height*3){
        png::image<png::rgb_pixel> save_image(image_info_.width, image_info_.height);
        unsigned int count = 0;
        for (int32_t v=0; v<image_info_.height; v++) {
            for (int32_t u=0; u<image_info_.width; u++) {
                png::rgb_pixel val;
                val.red   = (unsigned char)image_data_[count++];
                val.green = (unsigned char)image_data_[count++];
                val.blue  = (unsigned char)image_data_[count++];
                //val.alpha = (unsigned char)image_data_[count++];
                save_image.set_pixel(u,v,val);
            }
        }

        char file_name_image[500];

        if ( simFrame > 0 ) {
            sprintf(file_name_image, "000%03d_10.png", ( simFrame - 7 ));
            std::string input_image_file_with_path = m_dataset.getInputPath().string() + "/image_02/" + file_name_image;
            save_image.write(input_image_file_with_path);
        }
    }
    else {
        fprintf(stderr, "ignoring file with %d channels\n", image_info_.imgSize /( image_info_
                                                                                           .width*image_info_.height));
    }
}

void GroundTruthFlow::plot(std::string resultsordner) {

    PlotFlow::plot(m_dataset, resultsordner);

}

