#include "VTDConnector.h"

#include <QtDebug>
#include <QThread>
#include <stdlib.h>
#include <stdio.h>
#include <sys/shm.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include "RDBHandler.hh"

VTDConnector::VTDConnector(size_t numSHMs, unsigned int keys[], std::string& server, int port)
{
    m_numSHMs = numSHMs;
    m_shmKey = new unsigned int[numSHMs];
    m_checkMask = RDB_SHM_BUFFER_FLAG_TC;
    m_shmPtr = new void*[numSHMs];
    m_shmTotalSize = new size_t[numSHMs];
    m_verbose = false;
    m_forceBuffer = -1;
    m_haveImage = new int[numSHMs];
    m_server = server;
    m_port = port;
    m_client = -1;
    m_numSensorClients = 0;
    m_simFrame = 0;
    m_simTime = 0.0;
    m_deltaTime = 0.01;
    m_lastShmFrame = new int[numSHMs];

    for(size_t i = 0; i < m_numSHMs; ++i)
    {
        m_shmKey[i] = keys[i];
        m_shmPtr[i] = 0;
        m_shmTotalSize[0] = 0;
        m_haveImage[i]  = 0;
        m_lastShmFrame[i] = -1;
    }
}

VTDConnector::~VTDConnector()
{
    close(m_client);

    delete[] m_shmKey;
    delete[] m_shmPtr;
    delete[] m_shmTotalSize;
    delete[] m_haveImage;
    delete[] m_lastShmFrame;
}


void VTDConnector::registerDataCallback(OnData_t callback)
{
    m_onData = callback;
}

void VTDConnector::registerInfoCallback(OnSimInfo_t callback, size_t num_clients, int* ports)
{
    //Prepare data structures
    m_numSensorClients = num_clients;
    m_sensorPorts = new int[num_clients];
    m_sensorClients = new int[num_clients];
    for(int i = 0; i < num_clients; ++i){
        m_sensorPorts[i] = ports[i];

        //Init network sockets
        openNetwork(m_sensorPorts[i], &m_sensorClients[i]);
    }

    //Register callback
    m_onSimInfo = callback;
}

bool VTDConnector::open()
{
    bool success = openNetwork(m_port, &m_client);  // this is blocking until the network has been opened

    if (success)
    {
        for(size_t i = 0; i < m_numSHMs; ++i)
        {
            success = openShm(i);
            if(!success)
                break;
        }
    }
    m_simFrame = 0;
    m_simTime = 0.0;
    m_deltaTime = 0.01;
    m_haveFirstImage = 0;
    return success;
}

void VTDConnector::getFrame()
{
    //Read default RDB connection
    readNetwork(m_client);

    //Read sensor RDB connections
    for(int i = 0; i < m_numSensorClients; ++i){
        readNetwork(m_sensorClients[i]);
    }

    for(size_t i = 0; i < m_numSHMs; ++i)
    {
        if ( !m_haveImage[i] )
        {
            checkShm( 0, false);
        }
    }

    // has a set of images occured?
    bool haveImage = true;
    for(size_t i = 0; i < m_numSHMs; ++i)
    {
        haveImage &= m_haveImage[i];
    }
    if ( haveImage)
        m_haveFirstImage--;

    // has an image arrived or do the first frames need to be triggered
    //(first image will arrive with a certain frame delay only)
    if ( haveImage || ( m_haveFirstImage > 0 ) )
    {
      // do not initialize too fast
      if ( m_haveFirstImage > 0 ){
          //usleep( 100000 );   // 10H
          //QThread::sleep(1);
      }

      sendRDBTrigger( m_client, m_simTime, m_simFrame );

      // increase internal counters
      m_simTime += m_deltaTime;
      m_simFrame++;
    }

    // ok, reset image indicator
    if ( haveImage )
    {
        for(size_t i = 0; i < m_numSHMs; ++i)
        {
            m_haveImage[i] = 0;
        }
    }
}

bool VTDConnector::openNetwork( int port, int* client )
{
    struct sockaddr_in server;
    struct hostent    *host = NULL;

    // Create the socket, and attempt to connect to the server
    *client = socket( AF_INET, SOCK_STREAM, IPPROTO_TCP );

    if ( *client == -1 )
    {
       qDebug() << "socket() failed: " << strerror( errno );
       return false;
    }

    int opt = 1;
    setsockopt ( *client, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof( opt ) );

    server.sin_family      = AF_INET;
    server.sin_port        = htons(static_cast<uint16_t>(port));
    const char* hostname = m_server.c_str();
    server.sin_addr.s_addr = inet_addr(hostname);

    // If the supplied server address wasn't in the form
    // "aaa.bbb.ccc.ddd" it's a hostname, so try to resolve it
    if ( server.sin_addr.s_addr == INADDR_NONE )
    {
       host = gethostbyname(hostname);
       if ( host == NULL )
       {
           qDebug() << "Unable to resolve server: " << hostname;
           return false;
       }
       memcpy( &server.sin_addr, host->h_addr_list[0], host->h_length );
    }

    // wait for connection
    bool bConnected = false;

    while ( !bConnected  )
    {
       if ( connect( *client, (struct sockaddr *)&server, sizeof( server ) ) == -1 )
       {
           qDebug() << "connect() to port " << port << " failed: " << strerror( errno );
           sleep( 1 );
       }
       else
           bConnected = true;
    }

    qDebug() << "connected to port " << port << "!";
    return true;
}

bool VTDConnector::openShm(size_t index)
{
    // do not open twice!
     if ( m_shmPtr[index] )
         return true;

     int shmid = 0;

     if ( ( shmid = shmget( m_shmKey[index], 0, 0 ) ) < 0 )
         return false;

     if ( ( m_shmPtr[index] = (char *)shmat( shmid, (char *)0, 0 ) ) == (char *) -1 )
     {
         qDebug() << "openShm: shmat()";
         m_shmPtr[index] = 0;
     }

     if ( m_shmPtr[index] )
     {
         struct shmid_ds sInfo;

         if ( ( shmid = shmctl( shmid, IPC_STAT, &sInfo ) ) < 0 )
             qDebug() << "openShm: shmctl()";
         else
             m_shmTotalSize[index] = sInfo.shm_segsz;
     }
     return true;
}

int VTDConnector::checkShm(size_t index, bool clearContent)
{
    if ( !m_shmPtr[index] )
         return 0;

     // get a pointer to the shm info block
     RDB_SHM_HDR_t* shmHdr = ( RDB_SHM_HDR_t* ) ( m_shmPtr[index] );

     if ( !shmHdr )
         return 0;

     if ( ( shmHdr->noBuffers != 2 ) )
     {
         qDebug() << "checkShm: no or wrong number of buffers in shared memory. I need two buffers!";
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
     RDB_MSG_t* pRdbMsgA = ( RDB_MSG_t* ) ( ( ( char* ) m_shmPtr[index] ) + pBufferInfo[0]->offset );
     RDB_MSG_t* pRdbMsgB = ( RDB_MSG_t* ) ( ( ( char* ) m_shmPtr[index] ) + pBufferInfo[1]->offset );

     if ( m_verbose )
         qDebug() << "ImageReader::checkShm: Buffer A: offset = " << pBufferInfo[0]->offset
                  <<" Buffer B: offset = " << pBufferInfo[1]->offset;

     if ( clearContent )
     {
         if ( m_verbose )
             qDebug() << "ImageReader::checkShm: clearing the content\n";

         memset( pRdbMsgA, 0, sizeof( RDB_MSG_HDR_t ) );
         memset( pRdbMsgB, 0, sizeof( RDB_MSG_HDR_t ) );
     }

     // pointer to the message that will actually be read
     RDB_MSG_t* pRdbMsg  = 0;

     if ( clearContent )
     {
         pBufferInfo[0]->flags = 0;
         pBufferInfo[1]->flags = 0;
     }

     // remember the flags that are set for each buffer
     unsigned int flagsA = pBufferInfo[ 0 ]->flags;
     unsigned int flagsB = pBufferInfo[ 1 ]->flags;

     // check whether any buffer is ready for reading (checkMask is set (or 0) and buffer is NOT locked)
     bool readyForReadA = ( ( flagsA & m_checkMask ) || !m_checkMask ) && !( flagsA & RDB_SHM_BUFFER_FLAG_LOCK );
     bool readyForReadB = ( ( flagsB & m_checkMask ) || !m_checkMask ) && !( flagsB & RDB_SHM_BUFFER_FLAG_LOCK );

     if ( m_verbose )
     {
         qDebug() << "ImageReader::checkShm: before processing SHM 0x" << std::hex << m_shmKey[index];
         qDebug() << "ImageReader::checkShm: Buffer A: frameNo = " << pRdbMsgA->hdr.frameNo
                  << " flags = " << flagsA
                  << " locked = " << (( flagsA & RDB_SHM_BUFFER_FLAG_LOCK ) ? "true" : "false")
                  << " check mask set = "<< (( flagsA & m_checkMask ) ? "true" : "false")
                  << " readyForRead = "<< (readyForReadA ?  "true" : "false" );

         qDebug() << "Buffer B: frameNo = " << pRdbMsgB->hdr.frameNo
                  << " flags = " << flagsB
                  << " locked = " << ( (flagsB & RDB_SHM_BUFFER_FLAG_LOCK ) ? "true" : "false")
                  << " check mask set = " << (( flagsB & m_checkMask ) ? "true" : "false")
                  << " readyForRead = " << (readyForReadB ?  "true" : "false");
     }

     if ( m_forceBuffer < 0 )  // auto-select the buffer if none is forced to be read
     {
         // check which buffer to read
         if ( ( readyForReadA ) && ( readyForReadB ) )
         {
             if ( pRdbMsgA->hdr.frameNo < pRdbMsgB->hdr.frameNo )        // force using the latest image!!
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
     else if ( ( m_forceBuffer == 0 ) && readyForReadA )   // force reading buffer A
     {
         pRdbMsg            = pRdbMsgA;
         pCurrentBufferInfo = pBufferInfo[ 0 ];
     }
     else if ( ( m_forceBuffer == 1 ) && readyForReadB ) // force reading buffer B
     {
         pRdbMsg            = pRdbMsgB;
         pCurrentBufferInfo = pBufferInfo[ 1 ];
     }

     if ( m_verbose )
     {
         if ( pCurrentBufferInfo == pBufferInfo[ 0 ] )
             qDebug() << "ImageReader::checkShm: reading buffer A\n";
         else if ( pCurrentBufferInfo == pBufferInfo[ 1 ] )
             qDebug() << "ImageReader::checkShm: reading buffer B\n";
         else
             qDebug() << "ImageReader::checkShm: reading no buffer\n";
     }


     // lock the buffer that will be processed now (by this, no other process will alter the contents)
     if ( pCurrentBufferInfo )
     {
         pCurrentBufferInfo->flags |= RDB_SHM_BUFFER_FLAG_LOCK;

         if ( m_verbose )
             qDebug() << "ImageReader::checkShm: locked buffer, pRdbMsg = " << pRdbMsg;
     }

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
         qDebug() << "checkShm: zero message data size, error.";
         return 0;
     }

     if ( m_verbose )
         qDebug() << "ImageReader::checkShm: pRdbMsg->hdr.dataSize = " << pRdbMsg->hdr.dataSize;

     // running the same frame again?
     if ( (( int ) pRdbMsg->hdr.frameNo ) > m_lastShmFrame[index] )
     {
         // remember the last frame that was read
         m_lastShmFrame[index] = pRdbMsg->hdr.frameNo;

         while ( 1 )
         {
             if ( m_verbose )
                 qDebug() << "ImageReader::checkShm: handling message, frame = " << pRdbMsg->hdr.frameNo;

             // handle the message that is contained in the buffer; this method should be provided by the user (i.e. YOU!)
             if(m_onData)
             {
                 m_onData(index, pRdbMsg);
                 m_haveImage[index] = true;
             }

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
     pCurrentBufferInfo->flags &= ~m_checkMask;                   // remove the check mask
     pCurrentBufferInfo->flags &= ~RDB_SHM_BUFFER_FLAG_LOCK;     // remove the lock mask

     if ( m_verbose )
     {
         unsigned int flagsA = pBufferInfo[ 0 ]->flags;
         unsigned int flagsB = pBufferInfo[ 1 ]->flags;

         fprintf( stderr, "ImageReader::checkShm: after processing SHM 0x%x\n", m_shmKey[index] );
         fprintf( stderr, "ImageReader::checkShm: Buffer A: frameNo = %06d, flags = 0x%x, locked = <%s>, lock mask set = <%s>\n",
                          pRdbMsgA->hdr.frameNo,
                          flagsA,
                          ( flagsA & RDB_SHM_BUFFER_FLAG_LOCK ) ? "true" : "false",
                          ( flagsA & m_checkMask ) ? "true" : "false" );
         fprintf( stderr, "                       Buffer B: frameNo = %06d, flags = 0x%x, locked = <%s>, lock mask set = <%s>.\n",
                          pRdbMsgB->hdr.frameNo,
                          flagsB,
                          ( flagsB & RDB_SHM_BUFFER_FLAG_LOCK ) ? "true" : "false",
                          ( flagsB & m_checkMask ) ? "true" : "false" );
     }

     return 1;
}

void VTDConnector::sendRDBTrigger( int & sendSocket, const double & simTime, const unsigned int & simFrame )
{
    // is the socket available?
    if ( m_client < 0 )
        return;

    Framework::RDBHandler myHandler;

    // start a new message
    myHandler.initMsg();

    // add extended package for the object state
    RDB_TRIGGER_t *trigger = ( RDB_TRIGGER_t* ) myHandler.addPackage( simTime, simFrame, RDB_PKG_ID_TRIGGER, 1, true );

    if ( !trigger )
    {
        qDebug() << "sendRDBTrigger: could not create trigger";
        return;
    }

    trigger->frameNo = simFrame;
    trigger->deltaT  = m_deltaTime;

    qDebug() << "sendRDBTrigger: sending trigger, deltaT = " << trigger->deltaT;

    int retVal = send( sendSocket, ( const char* ) ( myHandler.getMsg() ), myHandler.getMsgTotalSize(), 0 );

    if ( !retVal )
        qDebug() << "sendRDBTrigger: could not send trigger";
}

void VTDConnector::readNetwork( int client )
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

        int noReady = getNoReadyRead( client );

        if ( noReady < 0 )
        {
            fprintf( stderr, "recv() failed: %s\n", strerror( errno ) );
            break;
        }

        if ( noReady > 0 )
        {
            // read data
            if ( ( ret = recv( client, szBuffer, DEFAULT_BUFFER, 0 ) ) != 0 )
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
                            //parseRDBMessage( ( RDB_MSG_t* ) spData );
                            if(m_onSimInfo) m_onSimInfo(( RDB_MSG_t* ) spData);

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

int VTDConnector::getNoReadyRead(int descriptor)
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
