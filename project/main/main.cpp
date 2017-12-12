#include <boost/bind.hpp>
#include "vires/RDBHandler.hh"

#include <sys/ipc.h>
#include <sys/shm.h>

#include <boost/filesystem.hpp>

#include <iostream>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <png++/rgb_pixel.hpp>
#include <png++/image.hpp>
#include <viRDBIcd.h>

#include "main.h"
#include "kbhit.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <boost/filesystem.hpp>

#include "datasets.h"
#include <kitti/io_flow.h>
#include "GridLayout.h"

extern int          mHaveImage    ;                                 // is an image available?
extern int          mClient       ;                                // client socket
extern unsigned int mSimFrame     ;                                 // simulation frame counter
extern double       mSimTime      ;                               // simulation time
extern double       mDeltaTime    ;                              // simulation step width
extern int mLastShmFrame;

extern unsigned int mShmPtr_writer;
extern bool mVerbose;
extern unsigned int mShmKey;
extern unsigned int mShmPtr;
extern double       mFrameTime;
extern unsigned int mCheckMask;
extern int mForceBuffer;

extern int  writeTriggerToShm();
extern void openShm();
extern void openNetwork();
extern void openNetwork_GT();
extern void readNetwork();
extern int checkShm();

extern char  szServer[128];             // Server to connect to
extern int   iPort;  // Port on server to connect to

extern int          mPortTx;
extern int          mSocketTx;
extern unsigned int mAddressTx;
extern int          mPortRx;
extern int          mSocketRx;
extern unsigned int mAddressRx;


extern void sendRDBTrigger( int & sendSocket, const double & simTime, const unsigned int & simFrame );


using boost_path=boost::filesystem::path;

extern void ground_truth(const boost::filesystem::path dataset_path);
extern std::string prepare_directories(const boost::filesystem::path dataset_path, const std::string result_sha);

extern void flow(const boost::filesystem::path dataset_path, const std::string result_sha, const std::string image_input_sha, FRAME_TYPES frame_types, short noise);
//extern bool eval(std::string result_sha, Mail *mail);
extern void plotVectorField (FlowImage &F,std::string dir,char* prefix);

extern void read_kitti_calibration(boost::filesystem::path);
extern void of_algo(boost::filesystem::path dataset_path, std::string video, std::string algo);
extern void make_video_from_png(boost::filesystem::path dataset_path, std::string unterordner);
extern void disparity(boost::filesystem::path dataset_path);
extern boost_path get_file(const boost_path &dataset_path, const boost_path &subfolder, const boost_path
&file_name);


/**
* information about usage of the software
* this method will exit the program
*/
void usage()
{
    printf("usage: videoTest [-k:key] [-c:checkMask] [-v] [-f:bufferId] [-p:x] [-s:IP] [-h]\n\n");
    printf("       -k:key        SHM key that is to be addressed\n");
    printf("       -c:checkMask  mask against which to check before reading an SHM buffer\n");
    printf("       -f:bufferId   force reading of a given buffer (0 or 1) instead of checking for a valid checkMask\n");
    printf("       -p:x          Remote port to send to\n");
    printf("       -s:IP         Server's IP address or hostname\n");
    printf("       -v            run in verbose mode\n");
    exit(1);
}

/**
* validate the arguments given in the command line
*/
void ValidateArgs(int argc, char **argv)
{
    // initalize the server variable
    strcpy( szServer, "127.0.0.1" );

    for( int i = 1; i < argc; i++)
    {
        if ((argv[i][0] == '-') || (argv[i][0] == '/'))
        {
            switch (tolower(argv[i][1]))
            {
                case 'k':        // shared memory key
                    if ( strlen( argv[i] ) > 3 )
                        sscanf( &argv[i][3], "0x%x", &mShmKey );
                    break;

                case 'c':       // check mask
                    if ( strlen( argv[i] ) > 3 )
                        mCheckMask = atoi( &argv[i][3] );
                    break;

                case 'f':       // force reading a given buffer
                    if ( strlen( argv[i] ) > 3 )
                        mForceBuffer = atoi( &argv[i][3] );
                    break;

                case 'v':       // verbose mode
                    mVerbose = true;
                    break;

                case 'p':        // Remote port
                    if (strlen(argv[i]) > 3)
                        iPort = atoi(&argv[i][3]);
                    break;
                case 's':       // Server
                    if (strlen(argv[i]) > 3)
                        strcpy(szServer, &argv[i][3]);
                    break;

                case 'h':
                default:
                    usage();
                    break;
            }
        }
    }

    fprintf( stderr, "ValidateArgs: key = 0x%x, checkMask = 0x%x, mForceBuffer = %d\n",
             mShmKey, mCheckMask, mForceBuffer );
}


/**
* main program with high frequency loop for checking the shared memory;
* does nothing else
*/


int main ( int argc, char *argv[]) {
    // Thread 2: Read two kitti image files without rain. The two image files are from kitti

    boost::filesystem::path calib_path;
    calib_path+=KITTI_RAW_CALIBRATION_PATH;
    calib_path+="calib_cam_to_cam.txt";
    if ( !boost::filesystem::exists(calib_path) ) {
        throw ("FileNotFound error");
    }
    read_kitti_calibration(calib_path);

    boost::filesystem::path kitti_full_image_path1, kitti_full_image_path2;

    kitti_full_image_path1 = get_file(KITTI_RAW_DATASET_PATH, "data/2011_09_28_drive_0016_sync/image_02/data/", "0000000169.png");
    kitti_full_image_path2 = get_file(KITTI_RAW_DATASET_PATH, "data/2011_09_28_drive_0016_sync/image_02/data/", "0000000170.png");

    std::cout << kitti_full_image_path1 << std::endl << kitti_full_image_path2 << std::endl;

    cv::Mat image_manual_bare1(cv::imread(kitti_full_image_path1.string(), CV_LOAD_IMAGE_COLOR));
    cv::Mat image_manual_bare2(cv::imread(kitti_full_image_path2.string(), CV_LOAD_IMAGE_COLOR));

    assert(image_manual_bare1.empty() == 0);
    assert(image_manual_bare2.empty() == 0);

    GridLayout grid_manual(image_manual_bare1, image_manual_bare2);
    cv::Mat image_manual_bare_grid = grid_manual.render();
    ImageShow grid_manual_show;
    grid_manual_show.show(image_manual_bare_grid);

    cv::waitKey(0);
    cv::destroyAllWindows();

//    boost::filesystem::path  dataset_path1 = KITTI_FLOW_DATASET_PATH;
//    make_video_from_png(dataset_path1, "data/stereo_flow/image_02/"); // give the path of the folder with pngs.

//    boost::filesystem::path  dataset_path2 = KITTI_RAW_DATASET_PATH;
//    make_video_from_png(dataset_path2, "data/2011_09_28_drive_0016_sync/image_02/data/"); // give the path of the folder with pngs.

    std::string result_dir;
    //test_kitti_original();

//    result_dir = prepare_directories(CPP_DATASET_PATH, "GT");
//    ground_truth(CPP_DATASET_PATH);

//    result_dir = prepare_directories(CPP_DATASET_PATH, "FB");
//    flow(CPP_DATASET_PATH, result_dir);

//    result_dir = prepare_directories(CPP_DATASET_PATH, "LK");
//    flow(CPP_DATASET_PATH, result_dir);

    result_dir = prepare_directories(MATLAB_DATASET_PATH, "LK");
    flow(MATLAB_DATASET_PATH, result_dir, std::string("image_02_slow/no_noise/"), continous_frames,0);

    result_dir = prepare_directories(MATLAB_DATASET_PATH, "LK");
    flow(MATLAB_DATASET_PATH, result_dir, std::string("image_02_slow/static_BG/"), continous_frames,1);


    result_dir = prepare_directories(MATLAB_DATASET_PATH, "LK");
    flow(MATLAB_DATASET_PATH, result_dir, std::string("image_02_slow/static_FG/"), continous_frames,2);

    result_dir = prepare_directories(MATLAB_DATASET_PATH, "LK");
    flow(MATLAB_DATASET_PATH, result_dir, std::string("image_02_slow/dynamic_BG/"), continous_frames,3);
    result_dir = prepare_directories(MATLAB_DATASET_PATH, "LK");
    flow(MATLAB_DATASET_PATH, result_dir, std::string("image_02_slow/dynamic_FG/"), continous_frames,4);



//    result_dir = prepare_directories(KITTI_FLOW_DATASET_PATH, "LK");
//    flow(KITTI_FLOW_DATASET_PATH, result_dir);

    //result_dir = prepare_directories("KITTI")KITTI_FLOW_DATASET_PATH


    //of_algo(dataset_path, "2011_09_28_drive_0016_sync.avi", "FB");
    //of_algo(dataset_path, "2011_09_28_drive_0016_sync.avi", "LK");
    //dataset_path = MATLAB_DATASET_PATH;
    //of_algo(dataset_path, "Movement.avi", "FB");
    //of_algo(dataset_path, "Movement.avi", "LK");
    //disparity(dataset_path);

    std::string m_server;
    int m_port;
    int m_sensor_port;
    bool m_triggers;
    int m_frames_to_read;
    bool m_write_ts_gt;
    boost::filesystem::path m_ts_gt_out_dir;

    int initCounter = 6;


    // Parse the command line
    ValidateArgs(argc, argv);

    // open the network connection to the taskControl (so triggers may be sent)
    fprintf( stderr, "creating network connection....\n" );
    openNetwork();  // this is blocking until the network has been opened
    openNetwork_GT();



    // now: open the shared memory (try to attach without creating a new segment)
    fprintf( stderr, "attaching to shared memory 0x%x....\n", mShmKey );

    while ( !mShmPtr )
    {
        openShm();
        usleep( 1000 );     // do not overload the CPU
    }

    fprintf( stderr, "...attached! Reading now...\n" );

    // now check the SHM for the time being
    while ( 1 )
    {
        readNetwork();

        if ( initCounter <= 0 )
            checkShm();

        // has an image arrived or do the first frames need to be triggered
        //(first image will arrive with a certain frame delay only)
        if ( mHaveImage || ( initCounter-- > 0 ) )
        {
            sendRDBTrigger( mClient, mSimTime, mSimFrame );

            // increase internal counters
            mSimTime += mDeltaTime;
            mSimFrame++;
        }

        // ok, reset image indicator
        mHaveImage = 0;

        usleep( 10000 );
    }


}


void MyRDBHandler::parseMessage( RDB_MSG_t* msg ) {
    Framework::RDBHandler::parseMessage(msg);
}

void MyRDBHandler::parseStartOfFrame(const double &simTime, const unsigned int &simFrame) {
    fprintf( stderr, "headers %d\n,", RDB_PKG_ID_START_OF_FRAME );
    fprintf( stderr, "RDBHandler::parseStartOfFrame: simTime = %.3f, simFrame = %d\n", simTime, simFrame );
}

void MyRDBHandler::parseEndOfFrame( const double & simTime, const unsigned int & simFrame )
{
    fprintf( stderr, "headers %d\n,", RDB_PKG_ID_END_OF_FRAME );
    fprintf( stderr, "RDBHandler::parseEndOfFrame: simTime = %.3f, simFrame = %d\n", simTime, simFrame );
}

void MyRDBHandler::parseEntry( RDB_OBJECT_CFG_t *data, const double & simTime, const unsigned int & simFrame, const
unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {
    RDB_OBJECT_CFG_t* object = reinterpret_cast<RDB_OBJECT_CFG_t*>(data); /// raw image data
    std::cout << object->type;
}

void MyRDBHandler::parseEntry( RDB_OBJECT_STATE_t *data, const double & simTime, const unsigned int & simFrame, const unsigned
short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {
    RDB_OBJECT_STATE_t* object = reinterpret_cast<RDB_OBJECT_STATE_t*>(data); /// raw image data
    if ( strcmp(data->base.name, "New Character") == 0) {
//        fprintf( stderr, "handleRDBitem: handling object state\n" );
//        fprintf( stderr, "    simTime = %.3lf, simFrame = %d\n", simTime, simFrame );
//        fprintf( stderr, "    object = %s, id = %d\n", data->base.name, data->base.id );
//        fprintf( stderr, "    position = %.3lf / %.3lf / %.3lf\n", data->base.pos.x, data->base.pos.y, data->base
//                .pos.z );

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


void MyRDBHandler::parseEntry( RDB_IMAGE_t *data, const double & simTime, const unsigned int & simFrame, const
unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {
    if ( !data )
        return;
    fprintf( stderr, "handleRDBitem: image\n" );
    fprintf( stderr, "    simTime = %.3lf, simFrame = %d, mLastShmFrame = %d\n", simTime, simFrame, mLastShmFrame );
    fprintf( stderr, "    width / height = %d / %d\n", data->width, data->height );
    fprintf( stderr, "    dataSize = %d\n", data->imgSize );

    // ok, I have an image:
    mHaveImage = 1;
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

        char file_name[500];
        sprintf(file_name,
                "/local/git/PriorityGraphSensors/vires_dataset/data/stereo_flow/image_02_car/000"
                        "%03d_10.png", simFrame);
        save_image.write(file_name);
    }
    else {
        fprintf(stderr, "ignoring file with %d channels\n", image_info_.imgSize /( image_info_
                                                                                           .width*image_info_.height));
    }
}
