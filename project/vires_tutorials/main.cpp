//#include "ImageUtils.h"
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

int main(int argc, char* argv[])
{
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
