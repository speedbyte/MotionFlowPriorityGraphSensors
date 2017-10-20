// ShmReader.cpp : Sample implementation of a process reading
// from a shared memory segment (double buffered) with RDB layout
// (c) 2016 by VIRES Simulationstechnologie GmbH
// Provided AS IS without any warranty!
//

#include <stdlib.h>
#include <stdio.h>
#include <sys/shm.h>
#include <string.h>
#include <unistd.h>
#include <viRDBIcd.h>
#include <adsim/vtd/rdb_codec.h>
#include <iostream>
#include <assert.h>
#include <png++/rgba_pixel.hpp>
#include <png++/image.hpp>
#include "RDBHandler.hh"

// forward declarations of methods

/**
* method for checking the contents of the SHM
*/
int  checkShm();
void openShm();

/**
* routine for handling an RDB message; to be provided by user;
* here, only a printing of the message is performed
* @param msg    pointer to the message that is to be handled
*/
void handleMessage( RDB_MSG_t* msg );

/**
* some global variables, considered "members" of this example
*/
unsigned int mShmKey_reader       = RDB_SHM_ID_IMG_GENERATOR_OUT;      // key of the SHM segment
unsigned int mCheckMask    = RDB_SHM_BUFFER_FLAG_TC;
void*        mShmPtr_reader       = 0;                                 // pointer to the SHM segment
size_t       mShmTotalSize = 0;                                 // remember the total size of the SHM segment
bool         mVerbose      = false;                             // run in verbose mode?
int          mForceBuffer  = -1;                                // force reading one of the SHM buffers (0=A, 1=B)


/**
* open the shared memory segment
*/
void openShm_reader()
{
    // do not open twice!
    if ( mShmPtr_reader )
        return;

    int shmid = 0;

    if ( ( shmid = shmget( mShmKey_reader, 0, 0 ) ) < 0 )
        return;

    if ( ( mShmPtr_reader = (char *)shmat( shmid, (char *)0, 0 ) ) == (char *) -1 )
    {
        perror("openShm: shmat()");
        mShmPtr_reader = 0;
    }

    if ( mShmPtr_reader )
    {
        struct shmid_ds sInfo;

        if ( ( shmid = shmctl( shmid, IPC_STAT, &sInfo ) ) < 0 )
            perror( "openShm: shmctl()" );
        else
            mShmTotalSize = sInfo.shm_segsz;
    }
}

int checkShm()
{
    if ( !mShmPtr_reader )
        return 0;

    // get a pointer to the shm info block
    RDB_SHM_HDR_t* shmHdr = ( RDB_SHM_HDR_t* ) ( mShmPtr_reader );

    if ( !shmHdr )
        return 0;

    if ( ( shmHdr->noBuffers != 2 ) )
    {
        fprintf( stderr, "checkShm: no or wrong number of buffers in shared memory. I need two buffers!" );
        return 0;
    }

    // allocate space for the buffer infos
    RDB_SHM_BUFFER_INFO_t** pBufferInfo = ( RDB_SHM_BUFFER_INFO_t** ) ( new char [ shmHdr->noBuffers * sizeof( RDB_SHM_BUFFER_INFO_t* ) ] );
    RDB_SHM_BUFFER_INFO_t*  pCurrentBufferInfo = 0;

    char* dataPtr = ( char* ) shmHdr;
    dataPtr += shmHdr->headerSize;

    for ( int i = 0; i < shmHdr->noBuffers; i++ )
    {
        pBufferInfo[ i ] = ( RDB_SHM_BUFFER_INFO_t* ) dataPtr;
        dataPtr += pBufferInfo[ i ]->thisSize;
    }

    // get the pointers to message section in each buffer
    RDB_MSG_t* pRdbMsgA = ( RDB_MSG_t* ) ( ( ( char* ) mShmPtr_reader ) + pBufferInfo[0]->offset );
    RDB_MSG_t* pRdbMsgB = ( RDB_MSG_t* ) ( ( ( char* ) mShmPtr_reader ) + pBufferInfo[1]->offset );

    // pointer to the message that will actually be read
    RDB_MSG_t* pRdbMsg  = 0;

    // remember the flags that are set for each buffer
    unsigned int flagsA = pBufferInfo[ 0 ]->flags;
    unsigned int flagsB = pBufferInfo[ 1 ]->flags;

    // check whether any buffer is ready for reading (checkMask is set (or 0) and buffer is NOT locked)
    bool readyForReadA = ( ( flagsA & mCheckMask ) || !mCheckMask ) && !( flagsA & RDB_SHM_BUFFER_FLAG_LOCK );
    bool readyForReadB = ( ( flagsB & mCheckMask ) || !mCheckMask ) && !( flagsB & RDB_SHM_BUFFER_FLAG_LOCK );

    if ( mVerbose )
    {
        fprintf( stderr, "ImageReader::checkShm: before processing SHM\n" );
        fprintf( stderr, "ImageReader::checkShm: Buffer A: frameNo = %06d, flags = 0x%x, locked = <%s>, lock mask set = <%s>, readyForRead = <%s>\n",
                 pRdbMsgA->hdr.frameNo,
                 flagsA,
                 ( flagsA & RDB_SHM_BUFFER_FLAG_LOCK ) ? "true" : "false",
                 ( flagsA & mCheckMask ) ? "true" : "false",
                 readyForReadA ?  "true" : "false" );

        fprintf( stderr, "                       Buffer B: frameNo = %06d, flags = 0x%x, locked = <%s>, lock mask set = <%s>, readyForRead = <%s>\n",
                 pRdbMsgB->hdr.frameNo,
                 flagsB,
                 ( flagsB & RDB_SHM_BUFFER_FLAG_LOCK ) ? "true" : "false",
                 ( flagsB & mCheckMask ) ? "true" : "false",
                 readyForReadB ?  "true" : "false" );
    }

    if ( mForceBuffer < 0 )  // auto-select the buffer if none is forced to be read
    {
        // check which buffer to read
        if ( ( readyForReadA ) && ( readyForReadB ) )
        {
            if ( pRdbMsgA->hdr.frameNo > pRdbMsgB->hdr.frameNo )        // force using the latest image!!
            {
                pRdbMsg            = pRdbMsgA;
                pCurrentBufferInfo = pBufferInfo[ 0 ];
            }
            else
            {
                pRdbMsg            = pRdbMsgB;
                pCurrentBufferInfo = pBufferInfo[ 1 ];
            }
        }
        else if ( readyForReadA )
        {
            pRdbMsg            = pRdbMsgA;
            pCurrentBufferInfo = pBufferInfo[ 0 ];
        }
        else if ( readyForReadB )
        {
            pRdbMsg            = pRdbMsgB;
            pCurrentBufferInfo = pBufferInfo[ 1 ];
        }
    }
    else if ( ( mForceBuffer == 0 ) && readyForReadA )   // force reading buffer A
    {
        pRdbMsg            = pRdbMsgA;
        pCurrentBufferInfo = pBufferInfo[ 0 ];
    }
    else if ( ( mForceBuffer == 1 ) && readyForReadB ) // force reading buffer B
    {
        pRdbMsg            = pRdbMsgB;
        pCurrentBufferInfo = pBufferInfo[ 1 ];
    }

    // lock the buffer that will be processed now (by this, no other process will alter the contents)
    if ( pCurrentBufferInfo )
        pCurrentBufferInfo->flags |= RDB_SHM_BUFFER_FLAG_LOCK;

    // no data available?
    if ( !pRdbMsg || !pCurrentBufferInfo )
    {
        delete pBufferInfo;
        pBufferInfo = 0;

        // return with valid result if simulation is not yet running
        if ( ( pRdbMsgA->hdr.frameNo == 0 ) && ( pRdbMsgB->hdr.frameNo == 0 ) )
            return 1;

        // otherwise return a failure
        return 0;
    }

    // handle all messages in the buffer
    if ( !pRdbMsg->hdr.dataSize )
    {
        fprintf( stderr, "checkShm: zero message data size, error.\n" );
        return 0;
    }

    unsigned int maxReadSize = pCurrentBufferInfo->bufferSize;

    while ( 1 )
    {
        // handle the message that is contained in the buffer; this method should be provided by the user (i.e. YOU!)
        handleMessage( pRdbMsg );

        // do not read more bytes than there are in the buffer (avoid reading a following buffer accidentally)
        maxReadSize -= pRdbMsg->hdr.dataSize + pRdbMsg->hdr.headerSize;

        if ( maxReadSize < ( pRdbMsg->hdr.headerSize + pRdbMsg->entryHdr.headerSize ) )
            break;

        // go to the next message (if available); there may be more than one message in an SHM buffer!
        pRdbMsg = ( RDB_MSG_t* ) ( ( ( char* ) pRdbMsg ) + pRdbMsg->hdr.dataSize + pRdbMsg->hdr.headerSize );

        if ( !pRdbMsg )
            break;

        if ( pRdbMsg->hdr.magicNo != RDB_MAGIC_NO )
            break;
    }

    // release after reading
    pCurrentBufferInfo->flags &= ~mCheckMask;                   // remove the check mask
    pCurrentBufferInfo->flags &= ~RDB_SHM_BUFFER_FLAG_LOCK;     // remove the lock mask

    if ( mVerbose )
    {
        unsigned int flagsA = pBufferInfo[ 0 ]->flags;
        unsigned int flagsB = pBufferInfo[ 1 ]->flags;

        fprintf( stderr, "ImageReader::checkShm: after processing SHM\n" );
        fprintf( stderr, "ImageReader::checkShm: Buffer A: frameNo = %06d, flags = 0x%x, locked = <%s>, lock mask set = <%s>\n",
                 pRdbMsgA->hdr.frameNo,
                 flagsA,
                 ( flagsA & RDB_SHM_BUFFER_FLAG_LOCK ) ? "true" : "false",
                 ( flagsA & mCheckMask ) ? "true" : "false" );
        fprintf( stderr, "                       Buffer B: frameNo = %06d, flags = 0x%x, locked = <%s>, lock mask set = <%s>.\n",
                 pRdbMsgB->hdr.frameNo,
                 flagsB,
                 ( flagsB & RDB_SHM_BUFFER_FLAG_LOCK ) ? "true" : "false",
                 ( flagsB & mCheckMask ) ? "true" : "false" );
    }

    return 1;
}

void handleMessage( RDB_MSG_t* msg )
{
    // just print the message
    //Framework::RDBHandler::printMessage( msg, true );
    bool csv = false;
    bool csvHeader = false;
    bool details = false;
    bool binDump = false;
    if ( !msg )
    {
        fprintf( stderr, "RDBHandler::printMessage: no message available\n" );
        return;
    }

    if ( !csv && !csvHeader )
    {
        fprintf( stderr, "\nRDBHandler::printMessage: ---- %s ----- BEGIN\n", details ? "full info" : "short info" );
        fprintf( stderr, "  message: version = 0x%04x, simTime = %.3f, simFrame = %d, headerSize = %d, dataSize = %d\n",
                 msg->hdr.version, msg->hdr.simTime, msg->hdr.frameNo, msg->hdr.headerSize, msg->hdr.dataSize );
    }

    if ( !msg->hdr.dataSize )
        return;

    RDB_MSG_ENTRY_HDR_t* entry = NULL;
    uint32_t remainingBytes = 0;

    if (NULL == msg) {
        return;
    }

    if (0 == msg->hdr.dataSize) {
        return;
    }

    // jump primary header
    entry = reinterpret_cast<RDB_MSG_ENTRY_HDR_t*>(reinterpret_cast<char*>(msg) + msg->hdr.headerSize);
    remainingBytes    = msg->hdr.dataSize;

    while ( remainingBytes > 0 )
    {

        /* Process start */
        //rdb_codec.cpp -> process(entry);

        uint32_t number_elements = 0;
        uint32_t i = 0;
        char* data = NULL;

        if (NULL == entry) {
            return;
        }

        if (0 == entry->elementSize) {

            switch (entry->pkgId) {
                case RDB_PKG_ID_START_OF_FRAME:
                    if ( csvHeader )
                        fprintf( stderr, "%23s,%23s,", "simTime", "simFrame" );
                    else if ( csv )
                        fprintf( stderr, "%+.16e,%23d,", msg->hdr.simTime, msg->hdr.frameNo );
                    //process(reinterpret_cast<RDB_START_OF_FRAME_t*>(data));
                    break;

                case RDB_PKG_ID_END_OF_FRAME:
                    //process(reinterpret_cast<RDB_END_OF_FRAME_t*>(data));
                    break;

                default:
                    std::cerr << "RDB_CODEC: RDB_PKG_ID " << entry->pkgId << " not implemented" << std::endl;
                    break;
            }
        } else {

            number_elements = entry->dataSize / entry->elementSize;
            assert(number_elements * entry->elementSize == entry->dataSize);

            // jump secondary header
            data = reinterpret_cast<char*>(entry) + entry->headerSize;

            for (i = 0; i < number_elements; ++i) {
                switch (entry->pkgId) {
                    case RDB_PKG_ID_COORD_SYSTEM:
                        //process(reinterpret_cast<RDB_COORD_SYSTEM_t*>(data));
                        break;

                    case RDB_PKG_ID_COORD:
                        //process(reinterpret_cast<RDB_COORD_t*>(data));
                        break;

                    case RDB_PKG_ID_ROAD_POS:
                        //process(reinterpret_cast<RDB_ROAD_POS_t*>(data));
                        break;

                    case RDB_PKG_ID_LANE_INFO:
                        //process(reinterpret_cast<RDB_LANE_INFO_t*>(data));
                        break;

                    case RDB_PKG_ID_ROADMARK:
                        //process(reinterpret_cast<RDB_ROADMARK_t*>(data));
                        break;

                    case RDB_PKG_ID_OBJECT_CFG:
                        //process(reinterpret_cast<RDB_OBJECT_CFG_t*>(data));
                        break;

                    case RDB_PKG_ID_OBJECT_STATE:
                        //process(reinterpret_cast<RDB_OBJECT_STATE_t*>(data),
                        //entry->flags & RDB_PKG_FLAG_EXTENDED);
                        break;

                    case RDB_PKG_ID_VEHICLE_SYSTEMS:
                        //process(reinterpret_cast<RDB_VEHICLE_SYSTEMS_t*>(data));
                        break;

                    case RDB_PKG_ID_VEHICLE_SETUP:
                        //process(reinterpret_cast<RDB_VEHICLE_SETUP_t*>(data));
                        break;

                    case RDB_PKG_ID_ENGINE:
                        //process(reinterpret_cast<RDB_ENGINE_t*>(data), entry->flags & RDB_PKG_FLAG_EXTENDED);
                        break;

                    case RDB_PKG_ID_DRIVETRAIN:
                        //process(reinterpret_cast<RDB_DRIVETRAIN_t*>(data), entry->flags & RDB_PKG_FLAG_EXTENDED);
                        break;

                    case RDB_PKG_ID_WHEEL:
                        //process(reinterpret_cast<RDB_WHEEL_t*>(data), entry->flags & RDB_PKG_FLAG_EXTENDED);
                        break;

                    case RDB_PKG_ID_PED_ANIMATION:
                        //process(reinterpret_cast<RDB_PED_ANIMATION_t*>(data));
                        break;

                    case RDB_PKG_ID_SENSOR_STATE:
                        //process(reinterpret_cast<RDB_SENSOR_STATE_t*>(data));
                        break;

                    case RDB_PKG_ID_SENSOR_OBJECT:
                        //process(reinterpret_cast<RDB_SENSOR_OBJECT_t*>(data));
                        break;

                    case RDB_PKG_ID_CAMERA:
                        //process(reinterpret_cast<RDB_CAMERA_t*>(data));
                        break;

                    case RDB_PKG_ID_CONTACT_POINT:
                        //process(reinterpret_cast<RDB_CONTACT_POINT_t*>(data));
                        break;

                    case RDB_PKG_ID_TRAFFIC_SIGN:
                        //process(reinterpret_cast<RDB_TRAFFIC_SIGN_t*>(data));
                        break;

                    case RDB_PKG_ID_ROAD_STATE:
                        //process(reinterpret_cast<RDB_ROAD_STATE_t*>(data));
                        break;

                    case RDB_PKG_ID_IMAGE:
                    case RDB_PKG_ID_LIGHT_MAP:
                    {
                        char* image_data_=NULL;
                        RDB_IMAGE_t* image = reinterpret_cast<RDB_IMAGE_t*>(data); /// raw image data

                        /// RDB image information of \see image_data_
                        RDB_IMAGE_t image_info_;
                        memset(&image_info_, 0, sizeof(RDB_IMAGE_t));
                        memcpy(&image_info_, image, sizeof(RDB_IMAGE_t));

                        if (NULL == image_data_) {
                            image_data_ = reinterpret_cast<char*>(malloc(image_info_.imgSize));
                        } else {
                            image_data_ = reinterpret_cast<char*>(realloc(image_data_, image_info_.imgSize));
                        }
                        // jump data header
                        memcpy(image_data_, reinterpret_cast<char*>(image) + sizeof(RDB_IMAGE_t), image_info_.imgSize);

                        png::image<png::rgba_pixel> save_image(image_info_.width, image_info_.height);
                        unsigned int count = 0;
                        for (int32_t v=0; v<image_info_.height; v++) {
                            for (int32_t u=0; u<image_info_.width; u++) {
                                png::rgba_pixel val;
                                val.red   = (unsigned char)image_data_[count++];
                                val.green = (unsigned char)image_data_[count++];
                                val.blue  = (unsigned char)image_data_[count++];
                                val.alpha = (unsigned char)image_data_[count++];
                                save_image.set_pixel(u,v,val);
                            }
                        }
                        std::string file_name = "/local/tmp/abcd.png";
                        save_image.write(file_name);
                        //process(reinterpret_cast<RDB_IMAGE_t*>(data));
                        break;
                    }

                    case RDB_PKG_ID_OCCLUSION_MATRIX:
                        // TODO(robin)
                        //assert(false && "RDB_PKG_ID_OCCLUSION_MATRIX not implemented");
                        break;

                    case RDB_PKG_ID_LIGHT_SOURCE:
                        //process(reinterpret_cast<RDB_LIGHT_SOURCE_t*>(data), entry->flags & RDB_PKG_FLAG_EXTENDED);
                        break;

                    case RDB_PKG_ID_ENVIRONMENT:
                        //process(reinterpret_cast<RDB_ENVIRONMENT_t*>(data));
                        break;

                    case RDB_PKG_ID_TRIGGER:
                        //process(reinterpret_cast<RDB_TRIGGER_t*>(data));
                        break;

                    case RDB_PKG_ID_DRIVER_CTRL:
                        //process(reinterpret_cast<RDB_DRIVER_CTRL_t*>(data));
                        break;

                    case RDB_PKG_ID_TRAFFIC_LIGHT:
                        //process(reinterpret_cast<RDB_TRAFFIC_LIGHT_t*>(data), entry->flags & RDB_PKG_FLAG_EXTENDED);
                        break;

                    case RDB_PKG_ID_SYNC:
                        //process(reinterpret_cast<RDB_SYNC_t*>(data));
                        break;

                    case RDB_PKG_ID_DRIVER_PERCEPTION:
                        //process(reinterpret_cast<RDB_DRIVER_PERCEPTION_t*>(data));
                        break;

                    case RDB_PKG_ID_TONE_MAPPING:
                        //process(reinterpret_cast<RDB_FUNCTION_t*>(data));
                        break;

                    case RDB_PKG_ID_ROAD_QUERY:
                        //process(reinterpret_cast<RDB_ROAD_QUERY_t*>(data));
                        break;

                    case RDB_PKG_ID_SCP:
                        // TODO(robin): relevant?
                        assert(false && "RDB_PKG_ID_SCP not implemented");
                        break;

                    case RDB_PKG_ID_TRAJECTORY:
                        //process(reinterpret_cast<RDB_TRAJECTORY_t*>(data));
                        break;

                    case RDB_PKG_ID_DYN_2_STEER:
                        //process(reinterpret_cast<RDB_DYN_2_STEER_t*>(data));
                        break;

                    case RDB_PKG_ID_STEER_2_DYN:
                        //process(reinterpret_cast<RDB_STEER_2_DYN_t*>(data));
                        break;

                    case RDB_PKG_ID_PROXY:
                        //process(reinterpret_cast<RDB_PROXY_t*>(data));
                        break;

                    case RDB_PKG_ID_MOTION_SYSTEM:
                        //process(reinterpret_cast<RDB_MOTION_SYSTEM_t*>(data));
                        break;

                    case RDB_PKG_ID_FREESPACE:
                        //process(reinterpret_cast<RDB_FREESPACE_t*>(data));
                        break;

                    case RDB_PKG_ID_DYN_EL_SWITCH:
                        //process(reinterpret_cast<RDB_DYN_EL_SWITCH_t*>(data));
                        break;

                    case RDB_PKG_ID_DYN_EL_DOF:
                        //process(reinterpret_cast<RDB_DYN_EL_DOF_t*>(data));
                        break;

                    case RDB_PKG_ID_IG_FRAME:
                        //process(reinterpret_cast<RDB_IG_FRAME_t*>(data));
                        break;

//        case RDB_PKG_ID_RT_PERFORMANCE:
//          process(reinterpret_cast<RDB_RT_PERFORMANCE_t*>(data));
//          break;

                    case RDB_PKG_ID_CUSTOM_SCORING:
                        //process(reinterpret_cast<RDB_CUSTOM_SCORING_t*>(data));
                        break;

                    case RDB_PKG_ID_CUSTOM_OBJECT_CTRL_TRACK:
                        //process(reinterpret_cast<RDB_CUSTOM_OBJECT_CTRL_TRACK_t*>(data));
                        break;

                    default:
                        std::cerr << "RDB_CODEC: RDB_PKG_ID " << entry->pkgId << " not implemented" << std::endl;
                        break;
                }
                data = reinterpret_cast<char*>(data) + entry->elementSize;
            }
            assert(reinterpret_cast<char*>(entry) + entry->headerSize + entry->dataSize == data);
        }

        Framework::RDBHandler::printMessageEntry( entry, details, csv, csvHeader );
        /* Process end */

        remainingBytes -= ( entry->headerSize + entry->dataSize );
        if ( remainingBytes )
            entry = ( RDB_MSG_ENTRY_HDR_t* ) ( ( ( ( char* ) entry ) + entry->headerSize + entry->dataSize ) );
    }
    assert(remainingBytes == 0);

    // create a binary dump?
    if ( binDump )
    {
        fprintf( stderr, "RDBHandler::printMessage: ---- binary dump ----- \n" );

        uint32_t remainingBytes = msg->hdr.dataSize + msg->hdr.headerSize;
        unsigned char* dataPtr = ( unsigned char* ) msg;

        for ( unsigned int i = 1; i <= remainingBytes; i++ )
        {
            fprintf( stderr, "%02x ", *dataPtr );

            dataPtr++;

            if ( !( i % 16 ) )
                fprintf( stderr, "\n" );
        }
        fprintf( stderr, "\n" );
    }

    if ( !csv )
        fprintf( stderr, "RDBHandler::printMessage: ---- %s ----- END\n", details ? "full info" : "short info" );

}