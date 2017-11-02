
#include <stdlib.h>
#include <stdio.h>
#include <sys/shm.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include "RDBHandler.hh"
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
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include "RDBHandler.hh"
#include "main.h"


// forward declarations of methods
/**
* method for writing the commands to the SHM
*/
int  writeTriggerToShm();

/**
* open the network interface for sending trigger data
*/
void openNetwork();

/**
* make sure network data is being read
*/
void readNetwork();

/**
* open the shared memory segment for reading image data
*/
void openShm();

/**
* check and parse the contents of the shared memory
*/
int checkShm();

/**
* check for data that has to be read (otherwise socket will be clogged)
* @param descriptor socket descriptor
*/
int getNoReadyRead( int descriptor );

/**
* parse an RDB message (received via SHM or network)
* @param msg    pointer to message that shall be parsed
*/
void parseRDBMessage( RDB_MSG_t* msg );

/**
* parse an RDB message entry
* @param simTime    simulation time of the message
* @param simFrame   simulation frame of the message
* @param entryHdr   pointer to the entry header itself
*/
void parseRDBMessageEntry( const double & simTime, const unsigned int & simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr );

/**
* handle an RDB item of type IMAGE
* @param simTime    simulation time of the message
* @param simFrame   simulation frame of the message
* @param img        pointer to the image data
*/
void handleRDBitem( const double & simTime, const unsigned int & simFrame, RDB_IMAGE_t* img );

/**
* routine for handling an RDB message; to be provided by user;
* here, only a printing of the message is performed
* @param msg    pointer to the message that is to be handled
*/
void handleMessage( RDB_MSG_t* msg );

/**
* send a trigger to the taskControl via network socket
* @param sendSocket socket descriptor
* @param simTime    internal simulation time
* @param simFrame   internal simulation frame
*/
void sendRDBTrigger( int & sendSocket, const double & simTime, const unsigned int & simFrame );


/**
* some global variables, considered "members" of this example
*/
unsigned int mShmKey       = RDB_SHM_ID_IMG_GENERATOR_OUT;      // key of the SHM segment
unsigned int mCheckMask    = RDB_SHM_BUFFER_FLAG_TC;
void*        mShmPtr       = 0;                                 // pointer to the SHM segment
size_t       mShmTotalSize = 0;                                 // remember the total size of the SHM segment
bool         mVerbose      = false;                             // run in verbose mode?
int          mForceBuffer  = -1;                                // force reading one of the SHM buffers (0=A, 1=B)
char         szServer[128];                                     // Server to connect to
int          iPort         = DEFAULT_PORT;                      // Port on server to connect to
int          mHaveImage    = 0;                                 // is an image available?
int          mClient       = -1;                                // client socket
int          mClient_GT    = -1;
unsigned int mSimFrame     = 0;                                 // simulation frame counter
double       mSimTime      = 0.0;                               // simulation time
double       mDeltaTime    = 0.01;                              // simulation step width
int          mLastShmFrame = -1;

/**
* some global variables, considered "members" of this example
*/
unsigned int mShmKey_writer       = RDB_SHM_ID_CONTROL_GENERATOR_IN;   // key of the SHM segment
unsigned int mFrameNo      = 0;
double       mFrameTime    = 0.030;                             // frame time is 30ms
void*        mShmPtr_writer       = 0;                                 // pointer to the SHM segment
size_t       mShmTotalSize_writer = 64 * 1024;                         // 64kB total size of SHM segment
// the memory and message management


int          mPortTx    = DEFAULT_TX_PORT;
int          mSocketTx  = -1;
unsigned int mAddressTx = INADDR_BROADCAST;
int          mPortRx    = DEFAULT_RX_PORT;
int          mSocketRx  = -1;
unsigned int mAddressRx = inet_addr( "127.0.0.1" );


void handleMessage( RDB_MSG_t* msg )
{
    bool csv = false;
    bool csvHeader = false;
    bool details = false;
    static uint32_t prevFrame=65535;
    if ( !msg )
    {
        fprintf( stderr, "RDBHandler::printMessage: no message available\n" );
        return;
    }

    if ( msg->hdr.frameNo == prevFrame ) {
        return;
    }

    if ( !csv && !csvHeader )
    {
        fprintf( stderr, "\nRDBHandler::printMessage: ---- %s ----- BEGIN\n", details ? "full info" : "short info" );
        fprintf( stderr, "  message: version = 0x%04x, simTime = %.3f, simFrame = %d, headerSize = %d, dataSize = %d\n",
                 msg->hdr.version, msg->hdr.simTime, msg->hdr.frameNo, msg->hdr.headerSize, msg->hdr.dataSize );
    }
    prevFrame = msg->hdr.frameNo;
    parseRDBMessage( msg );
}


void openNetwork()
{
    struct sockaddr_in server;
    struct hostent    *host = NULL;

    // Create the socket, and attempt to connect to the server
    mClient = socket( AF_INET, SOCK_STREAM, IPPROTO_TCP );

    if ( mClient == -1 )
    {
        fprintf( stderr, "socket() failed: %s\n", strerror( errno ) );
        return;
    }

    int opt = 1;
    setsockopt ( mClient, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof( opt ) );

    server.sin_family      = AF_INET;
    server.sin_port        = htons(iPort);
    server.sin_addr.s_addr = inet_addr(szServer);

    // If the supplied server address wasn't in the form
    // "aaa.bbb.ccc.ddd" it's a hostname, so try to resolve it
    if ( server.sin_addr.s_addr == INADDR_NONE )
    {
        host = gethostbyname(szServer);
        if ( host == NULL )
        {
            fprintf( stderr, "Unable to resolve server: %s\n", szServer );
            return;
        }
        memcpy( &server.sin_addr, host->h_addr_list[0], host->h_length );
    }

    // wait for connection
    bool bConnected = false;

    while ( !bConnected )
    {
        if ( connect( mClient, (struct sockaddr *)&server, sizeof( server ) ) == -1 )
        {
            fprintf( stderr, "connect() failed: %s\n", strerror( errno ) );
            sleep( 1 );
        }
        else
            bConnected = true;
    }

    fprintf( stderr, "connected!\n" );
}

void openNetwork_GT()
{
    struct sockaddr_in server;
    struct hostent    *host = NULL;

    // Create the socket, and attempt to connect to the server
    mClient_GT = socket( AF_INET, SOCK_STREAM, IPPROTO_TCP );

    if ( mClient_GT == -1 )
    {
        fprintf( stderr, "socket() failed: %s\n", strerror( errno ) );
        return;
    }

    int opt = 1;
    setsockopt ( mClient_GT, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof( opt ) );

    server.sin_family      = AF_INET;
    server.sin_port        = htons(48185);
    server.sin_addr.s_addr = inet_addr(szServer);

    // If the supplied server address wasn't in the form
    // "aaa.bbb.ccc.ddd" it's a hostname, so try to resolve it
    if ( server.sin_addr.s_addr == INADDR_NONE )
    {
        host = gethostbyname(szServer);
        if ( host == NULL )
        {
            fprintf( stderr, "Unable to resolve server: %s\n", szServer );
            return;
        }
        memcpy( &server.sin_addr, host->h_addr_list[0], host->h_length );
    }

    // wait for connection
    bool bConnected = false;

    while ( !bConnected )
    {
        if ( connect( mClient_GT, (struct sockaddr *)&server, sizeof( server ) ) == -1 )
        {
            fprintf( stderr, "connect() failed: %s\n", strerror( errno ) );
            sleep( 1 );
        }
        else
            bConnected = true;
    }

    fprintf( stderr, "connected!\n" );
}


void readNetwork()
{
    static char*         szBuffer       = 0;
    int                  ret            = 0;
    static unsigned int  sBytesInBuffer = 0;
    static size_t        sBufferSize    = sizeof( RDB_MSG_HDR_t );
    static unsigned char *spData        = ( unsigned char* ) calloc( 1, sBufferSize );
    static int           sVerbose       = 0;

    if ( !szBuffer )
        szBuffer = new char[DEFAULT_BUFFER];  // allocate on heap

    // make sure this is non-bloekcing and read everything that's available!
    do
    {
        ret = 0;        // nothing read yet

        int noReady = getNoReadyRead( mClient_GT );

        if ( noReady < 0 )
        {
            fprintf( stderr, "recv() failed: %s\n", strerror( errno ) );
            break;
        }

        if ( noReady > 0 )
        {
            // read data
            if ( ( ret = recv( mClient_GT, szBuffer, DEFAULT_BUFFER, 0 ) ) != 0 )
            {
                // do we have to grow the buffer??
                if ( ( sBytesInBuffer + ret ) > sBufferSize )
                {
                    spData      = ( unsigned char* ) realloc( spData, sBytesInBuffer + ret );
                    sBufferSize = sBytesInBuffer + ret;
                }

                memcpy( spData + sBytesInBuffer, szBuffer, ret );
                sBytesInBuffer += ret;

                // already complete messagae?
                if ( sBytesInBuffer >= sizeof( RDB_MSG_HDR_t ) )
                {
                    RDB_MSG_HDR_t* hdr = ( RDB_MSG_HDR_t* ) spData;

                    // is this message containing the valid magic number?
                    if ( hdr->magicNo != RDB_MAGIC_NO )
                    {
                        fprintf( stderr, "message receiving is out of sync; discarding data" );
                        sBytesInBuffer = 0;
                    }
                    else
                    {
                        // handle all messages in the buffer before proceeding
                        while ( sBytesInBuffer >= ( hdr->headerSize + hdr->dataSize ) )
                        {
                            unsigned int msgSize = hdr->headerSize + hdr->dataSize;

                            // print the message
                            if ( sVerbose )
                                Framework::RDBHandler::printMessage( ( RDB_MSG_t* ) spData, true );

                            // now parse the message
                            parseRDBMessage( ( RDB_MSG_t* ) spData );

                            // remove message from queue
                            memmove( spData, spData + msgSize, sBytesInBuffer - msgSize );
                            sBytesInBuffer -= msgSize;
                        }
                    }
                }
            }
        }
    }
    while ( ret > 0 );
}


/**
* open the shared memory segment
*/

void openShm()
{
    // do not open twice!
    if ( mShmPtr )
        return;

    int shmid = 0;

    if ( ( shmid = shmget( mShmKey, 0, 0 ) ) < 0 )
        return;

    if ( ( mShmPtr = (char *)shmat( shmid, (char *)0, 0 ) ) == (char *) -1 )
    {
        perror("openShm: shmat()");
        mShmPtr = 0;
    }

    if ( mShmPtr )
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
    if ( !mShmPtr )
        return 0;

    // get a pointer to the shm info block
    RDB_SHM_HDR_t* shmHdr = ( RDB_SHM_HDR_t* ) ( mShmPtr );

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
    RDB_MSG_t* pRdbMsgA = ( RDB_MSG_t* ) ( ( ( char* ) mShmPtr ) + pBufferInfo[0]->offset );
    RDB_MSG_t* pRdbMsgB = ( RDB_MSG_t* ) ( ( ( char* ) mShmPtr ) + pBufferInfo[1]->offset );

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

    // running the same frame again?
    if ( (( int ) pRdbMsg->hdr.frameNo ) > mLastShmFrame )
    {
        // remember the last frame that was read
        mLastShmFrame = pRdbMsg->hdr.frameNo;

        while ( 1 )
        {
            // handle the message that is contained in the buffer; this method should be provided by the user (i.e. YOU!)
            handleMessage( pRdbMsg );

            // go to the next message (if available); there may be more than one message in an SHM buffer!
            pRdbMsg = ( RDB_MSG_t* ) ( ( ( char* ) pRdbMsg ) + pRdbMsg->hdr.dataSize + pRdbMsg->hdr.headerSize );

            if ( !pRdbMsg )
                break;

            if ( pRdbMsg->hdr.magicNo != RDB_MAGIC_NO )
                break;
        }
    }

    //fprintf( stderr, "checkShm: mLastShmFrame = %ld\n", mLastShmFrame );

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

void parseRDBMessage( RDB_MSG_t* msg )
{
    MyRDBHandler mRdbHandler;                              // use the RDBHandler helper routines to handle
    mRdbHandler.parseMessage(msg);
}

int getNoReadyRead( int descriptor )
{
    fd_set         fs;
    struct timeval time;

    // still no valid descriptor?
    if ( descriptor < 0 )
        return 0;

    FD_ZERO( &fs );
    FD_SET( descriptor, &fs );

    memset( &time, 0, sizeof( struct timeval ) );

    int retVal = 0;

    retVal = select( descriptor + 1, &fs, (fd_set*) 0, (fd_set*) 0, &time);

    if ( retVal > 0 )
        return FD_ISSET( descriptor, &fs );

    return retVal;
}

void sendRDBTrigger( int & sendSocket, const double & simTime, const unsigned int & simFrame )
{
    // is the socket available?
    if ( mClient < 0 )
        return;

    Framework::RDBHandler myHandler;

    // start a new message
    myHandler.initMsg();

    // add extended package for the object state
    RDB_TRIGGER_t *trigger = ( RDB_TRIGGER_t* ) myHandler.addPackage( simTime, simFrame, RDB_PKG_ID_TRIGGER, 1, true );

    if ( !trigger )
    {
        fprintf( stderr, "sendRDBTrigger: could not create trigger\n" );
        return;
    }

    trigger->frameNo = simFrame;
    trigger->deltaT  = mDeltaTime;

    fprintf( stderr, "sendRDBTrigger: sending trigger, deltaT = %.4lf\n", trigger->deltaT );

    int retVal = send( mClient, ( const char* ) ( myHandler.getMsg() ), myHandler.getMsgTotalSize(), 0 );

    if ( !retVal )
        fprintf( stderr, "sendRDBTrigger: could not send trigger\n" );
}
