// ShmWriter.cpp : Sample implementation of a connection to the IG's control
// SHM for triggering individual render frames.
//
// (c) 2015 by VIRES Simulationstechnologie GmbH
// Provided AS IS without any warranty!
//

#include <stdlib.h>
#include <stdio.h>
#include <sys/shm.h>
#include <string.h>
#include <unistd.h>
#include "RDBHandler.hh"

// forward declarations of methods

/**
* method for writing the commands to the SHM
*/
void openShm();
int  writeTriggerToShm();

/**
* some global variables, considered "members" of this example
*/
unsigned int mShmKey_writer       = RDB_SHM_ID_CONTROL_GENERATOR_IN;   // key of the SHM segment
unsigned int mFrameNo      = 0;
double       mFrameTime    = 0.030;                             // frame time is 30ms
void*        mShmPtr_writer       = 0;                                 // pointer to the SHM segment
size_t       mShmTotalSize_writer = 64 * 1024;                         // 64kB total size of SHM segment
Framework::RDBHandler mRdbHandler;                              // use the RDBHandler helper routines to handle
// the memory and message management


/**
* open the shared memory segment; create if necessary
*/
void openShm_writer()
{
    // do not open twice!
    if ( mShmPtr_writer )
        return;

    int shmid = 0;
    int flag  = IPC_CREAT | 0777;

    // does the memory already exist?
    if ( ( shmid = shmget( mShmKey_writer, 0, 0 ) ) < 0 )
    {
        // not yet there, so let's create the segment
        if ( ( shmid = shmget( mShmKey_writer, mShmTotalSize_writer, flag ) ) < 0 )
        {
            perror("openShm: shmget()");
            mShmPtr_writer = 0;
            return;
        }
    }

    // now attach to the segment
    if ( ( mShmPtr_writer = (char *)shmat( shmid, (char *)0, 0 ) ) == (char *) -1 )
    {
        perror("openShm: shmat()");
        mShmPtr_writer = 0;
    }

    if ( !mShmPtr_writer )
        return;

    struct shmid_ds sInfo;

    if ( ( shmid = shmctl( shmid, IPC_STAT, &sInfo ) ) < 0 )
        perror( "openShm: shmctl()" );
    else
        mShmTotalSize_writer = sInfo.shm_segsz;

    // allocate a single buffer within the shared memory segment
    mRdbHandler.shmConfigure( mShmPtr_writer, 1, mShmTotalSize_writer );
}

int writeTriggerToShm()
{
    if ( !mShmPtr_writer )
        return 0;

    // get access to the administration information of the first RDB buffer in SHM
    RDB_SHM_BUFFER_INFO_t* info = mRdbHandler.shmBufferGetInfo( 0 );

    if ( !info )
        return 0;

    // is the buffer ready for write?
    if ( info->flags )      // is the buffer accessible (flags == 0)?
        return 0;

    // clear the buffer before writing to it (otherwise messages will accumulate)
    if ( !mRdbHandler.shmBufferClear( 0, true ) )   // true = clearing will be forced; not recommended!
        return 0;

    fprintf( stderr, "writeTriggerToShm: sending single trigger\n" );

    // increase the frame number
    mFrameNo++;

    // create a message containing the sync information
    RDB_SYNC_t* syncData = ( RDB_SYNC_t* ) mRdbHandler.addPackage( mFrameNo * mFrameTime, mFrameNo, RDB_PKG_ID_SYNC );

    if ( !syncData )
        return 0;

    syncData->mask    = 0x0;
    syncData->cmdMask = RDB_SYNC_CMD_RENDER_SINGLE_FRAME;

    // set some information concerning the RDB buffer itself
    info->id    = 1;
    info->flags = RDB_SHM_BUFFER_FLAG_IG;

    // now copy the symc message to the first RDB buffer in SHM
    mRdbHandler.mapMsgToShm( 0 );

    return 1;
}