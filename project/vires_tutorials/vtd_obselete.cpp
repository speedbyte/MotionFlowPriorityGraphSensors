
/* External
#include "TrafficSigns.h"
#include "Lanes.h"
#include "ImageUtils.h"
#include "../Parameters/ParameterList.h"
#include "../Parameters/ParameterImpl.h"
#include "TrafficSigns.h"
#include "Lanes.h"
#include "VTDSource.h"
 */
#include <QDebug>
#include <QImage>
#include <boost/bind.hpp>
#include "RDBHandler.hh"
#include <sstream>

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
#include <iostream>
#include <thread>
#include "RDBHandler.hh"
#include "vtd_obselete.h"

class TrafficSigns;

VTDBroadcastComm::VTDBroadcastComm() : BaseSource()
{
    data_ready = false;
}


void VTDBroadcastComm::init()
{
    fetchParamValues();
}

void VTDBroadcastComm::fetchParamValues()
{
    std::cout << "initialising" << std::endl;
}

// TODO
bool VTDBroadcastComm::open(){

    std::string server = "localhost"; //m_parameters.getString(0);
    int port = 48179; //m_parameters.getInt(1);
    int sensor_port = 2000; //m_parameters.getInt(2);

    m_vtdConnector = new VTDConnector(1, 0, server, port);
    m_vtdConnector->registerDataCallback(boost::bind(&VTDBroadcastComm::handleMsg, this, _1 ,_2));
    std::cout << "VTD opened" << std::endl;
    return m_vtdConnector->open();
}


void VTDBroadcastComm::handleMsg(size_t /* shmNum */, RDB_MSG_t* pRdbMsg){

    if (pRdbMsg->entryHdr.pkgId == RDB_PKG_ID_PROXY)
    {
        size_t remainingBytes = pRdbMsg->entryHdr.dataSize;
        std::cout << "Incoming broadcast message: " << pRdbMsg->hdr.frameNo << std::endl;
        RDB_MSG_ENTRY_HDR_t* p_entry_hdr = &pRdbMsg->entryHdr;
        char* data_ptr = (char*)p_entry_hdr + p_entry_hdr->headerSize;

        // use stringstreams for conversion
        uint16_t type = &data_ptr;
        data_ptr += type;
        uint32_t sender = &data_ptr;
        data_ptr += sender;
        char* msg = data_ptr;

        // decide what to do with message
        switch (type)
        {
            case 5:
                std::cout << "Hello, this is proof of concept!" << std::endl;
                break;
            default:
                break;
        }

        data_ready = true;
    }
}


RDB_MSG_t* VTDBroadcastComm::composeMsg(uint16_t type, uint32_t sender, char* data)
{
    RDB_MSG_t* msg = new RDB_MSG_t();
    msg->entryHdr.pkgId = RDB_PKG_ID_PROXY;
    msg->entryHdr.dataSize = std::getSize(&data);
    RDB_MSG_ENTRY_HDR_t* entryPtr = &msg->entryHdr;
    char* dataPtr = ( char* )entryPtr + entryPtr->headerSize;

    return msg;
}

bool VTDBroadcastComm::sendMessage(RDB_MSG_t* msg)
{

    if(msg != NULL)
        return true;
    else
        return false;
}


void VTDBroadcastComm::printMsgInfo(RDB_MSG_t* msg)
{
    //std::cout << "Received VTD message number " << msg->entryHdr.pkId << ". Type: " << msg->entryHdr.framNo << std::endl;
    //std::cout << "Message Data: " << msg->data << std::endl;
}

void VTDBroadcastComm::close()
{
    delete m_vtdConnector;
}


VTDCameraSensor::VTDCameraSensor(adsim::vtd::RDBTransceiver* rdb_client)
        : RDBCodec(*rdb_client)
        , m_left_img(nullptr)
        , m_right_img(nullptr)
        , m_depth_img(nullptr)
{
    // init first camera model, before the correct one is received via rdb
    m_camera_info.clipNear = 0.1;
    m_camera_info.clipFar = 100;
}

void VTDCameraSensor::process(RDB_IMAGE_t* image)
{
    //	std::cout << "image" << std::endl;
    size_t width = image->width;
    size_t height = image->height;

    uchar* buf_img = reinterpret_cast<uchar*>(image + 1);

    if (image->pixelFormat == RDB_PIX_FORMAT_DEPTH32) {
        if (m_depth_img) return;
        m_depth_img = new std::unique_ptr<DepthImage>(new DepthImage(width, height, 1));
        std::unique_ptr<DepthImage>& depthImage = *m_depth_img;
        copyBuffToImageData<DepthImage>(buf_img, *depthImage);
        convertToDepth(*depthImage);
    } else {
        if (m_left_img) return;
        // DONT access m_image_info.spare1 or .color, as these are pointers!
        // See comment below in process(RDB_CAMERA_t*)
        m_image_info = *image;
        size_t channels = 3;

        // Left image
        m_left_img = new std::unique_ptr<ByteImage>(new ByteImage(width, height, channels));
        std::unique_ptr<ByteImage>& leftImage = *m_left_img;
        copyBuffToImageData<ByteImage>(buf_img, *leftImage);

        // Right image
        m_right_img = new std::unique_ptr<ByteImage>(new ByteImage(width, height, channels));
        std::unique_ptr<ByteImage>& rightImage = *m_right_img;
        copyBuffToImageData<ByteImage>(buf_img, *rightImage);
    }
}

void VTDCameraSensor::process(RDB_CAMERA_t* camera)
{
    //	std::cout << "camera" << std::endl;

    // DONT access m_camera_info.spare1, for it is a pointer and not deep
    // copied by this shallow copy operation. This only works because there are
    // no pointers in the RDB_CAMERA_t struct (and all its member strutcts)!
    m_camera_info = *camera;
}

QImage VTDCameraSensor::getCutFromLeftImage(QRect rect)
{
    return QImage(m_left_img->get()->ptr,
                  m_image_info.width,
                  m_image_info.height,
                  QImage::Format_RGB888).copy(rect);
}

void VTDCameraSensor::convertToDepth(DepthImage& image)
{
    //z = 0.5*(f+n)/(f-n) + (-f*n)/(f-n) * (1/d) + 0.5
    //
    //with:
    //z: z-buffer value (normalized in [0,1]. Non-normalized fixed point zf = z * (s^n - 1 ) where n is bit depth of the depth buffer)
    //d: distance of fragment (pixel) to xy plane of camera coordinate system
    //n: near plane (camera frustum setting)
    //f: far plane (camera frustum setting)

    unsigned int* src = reinterpret_cast<unsigned int*>(image.ptr);
    float* dst = reinterpret_cast<float*>(image.ptr);

    /*
     *float alpha = cameraInfo->focalY / 2;
     *float n = cameraInfo->height / 2 / tan(alpha);
     */

    for(size_t pos = 0; pos < image.width * image.height; ++pos)
    {
        unsigned int z = src[pos];
        float z1 = (float)z / (unsigned int)0xFFFFFFFF; // ZMAX
        float d = 2 * m_camera_info.clipNear * m_camera_info.clipFar /
                  (m_camera_info.clipFar + m_camera_info.clipNear - z1 *
                                                                    (m_camera_info.clipFar - m_camera_info.clipNear));
        dst[pos] = d;
    }
}

template<typename T>
void VTDCameraSensor::copyBuffToImageData(uchar* buf_img, T& image) {
    image.frame_nr = FrameNumber();
    size_t bufSize = image.getImageSize();
    memcpy(image.ptr, buf_img, bufSize);
}


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


void VTDSource::init()
{
    fetchParamValues();
}

bool VTDSource::open(){
    try {
        m_rdb_shm_client = new RDBTransceiverSharedMemory(SHM_IMG_BUFFER, FREE_FLAG);
        m_camera_sensor = new VTDCameraSensor(m_rdb_shm_client);

        m_rdb_tcp_info_client = new RDBTransceiverTCP(m_server, std::to_string(m_sensor_port));
        m_info_sensor = new VTDInfoSensor(m_rdb_tcp_info_client);

        m_rdb_tcp_command_client = new RDBTransceiverTCP(m_server, std::to_string(m_port));
        m_command_connector = new adsim::vtd::RDBCodec(*m_rdb_tcp_command_client);

        if (m_triggers)
            m_command_connector->addTriggerAndSend(1./60);

        return true;
    } catch (const std::exception& e) {
        // in case vtd is not running and shm/tcp connections fail
        std::cerr << e.what() << std::endl;
        return false;
    }
}

bool VTDSource::load()
{
    size_t n = 200;
    while (!m_camera_sensor->hasData()
           || m_previous_frame_nr == m_camera_sensor->FrameNumber()) {
        m_camera_sensor->process();
        m_info_sensor->process();

        // suuuuper dirty hack: after some time the command connection gets
        // terminated. this checks for long times without a new image and
        // reconnects to vtd. fixes the symptom, not the cause, but well...
        if (!--n) {
            m_rdb_tcp_command_client->reconnect(m_server, std::to_string(m_port));
            if (m_triggers)
                m_command_connector->addTriggerAndSend(1./60);
            n = 200;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // check for dropped frames
    if(m_previous_frame_nr && m_previous_frame_nr != m_camera_sensor->FrameNumber() - 1)
        std::cerr << " -- WARNING: Frames " << m_previous_frame_nr + 1 << " to "
                  << (uint)m_camera_sensor->FrameNumber() - 1 << " dropped! "
                  << m_previous_frame_nr << " -> "
                  << (int)m_camera_sensor->FrameNumber() << std::endl;

    copyDataToOutputs();

    // remember last frame
    m_previous_frame_nr = m_camera_sensor->FrameNumber();

    // reset sensors
    m_camera_sensor->clear();
    m_info_sensor->clear(m_camera_sensor->FrameNumber());

    // trigger vtd server for next frame (we read that in the next load() call)
    if (m_triggers)
        m_command_connector->addTriggerAndSend(1./60);

    // stop after m_frames_to_read frames, never stop if m_frames_to_read == -1
    return m_frames_to_read == -1 || --m_frames_to_read > 0;
}

void VTDSource::copyDataToOutputs()
{
    std::unique_ptr<TrafficSigns>& ts_out = *(new std::unique_ptr<TrafficSigns>(new TrafficSigns()));

    RDB_IMAGE_t imageInfo = m_camera_sensor->getImageInfo();
    QRect window = QRect(QPoint(0, 0), QSize(imageInfo.width, imageInfo.height));

    // flip and add all signs to output
    uint foundNo = 0;
    for (TrafficSign* ts: m_info_sensor->getTrafficSigns(m_camera_sensor->FrameNumber())) {
        QPoint pos = ts->getCenter();
        pos.setY(imageInfo.height - pos.y());
        ts->setCenter(pos);
        ts_out->push_back(ts);

        // if we want to, save all image regions with traffic signs in them
        if(m_write_ts_gt) {

            // only save images that are not cut by the screen
            QSize s = ts->getSize();
            QRect bb = ts->boundingRect();
            // grow region to circumvent vires bug (hotfix)
            QRect cut = bb.marginsAdded(QMargins(s.width()*.1, s.height()*.1,
                                                 s.width()*.1, s.height()*.1));
            if (!window.contains(cut)) continue;

            QString file = QString::fromStdString(m_ts_gt_out_dir);
            file += "/" + QString::number(ts->getType()) + "-" +
                    QString::number(ts->getSubtype()) + "/";
            QDir dir(file);
            if(!dir.exists(file))
            {
                dir.mkdir(file);
            }
            file += QString::number(ts->getFrameNumber()) + "_" +
                    QString::number(foundNo++) + "_x" +
                    QString::number(pos.x()) + "_y" +
                    QString::number(pos.y()) + ".bmp";

            m_camera_sensor->getCutFromLeftImage(cut).save(file);
        }
    }

    io_collection->setOutputData<TrafficSigns>("TSR-GT", ts_out);


    std::unique_ptr<Lanes>& lanes_out = *(new std::unique_ptr<Lanes>(new Lanes()));

    Lanes lanes = m_info_sensor->getLanes();
    lanes.flipTesselationPoints(m_camera_sensor->getCameraInfo().height);
    lanes.cropToWindow(window);
    lanes.cleanVTDLanes();
    for (Lane* lane: lanes) {
        lanes_out->push_back(lane);
    }

    io_collection->setOutputData<Lanes>("Lane-GT", lanes_out);

    io_collection->setOutputData<ByteImage>("Left Image",
                                            *m_camera_sensor->getLeftImage());
    io_collection->setOutputData<ByteImage>("Right Image",
                                            *m_camera_sensor->getRightImage());
    io_collection->setOutputData<DepthImage>("Depth Image",
                                             *m_camera_sensor->getDepthImage());
}

void VTDSource::fillDefaultParameters()
{
    m_parameters.add("localhost", "VTD host address");
    m_parameters.add(48190, "VTD port");
    m_parameters.add(48195, "Sensor port (GT)");
    m_parameters.add(true, "Trigger VTD?");
    m_parameters.add(-1, "Number of frames to read (-1 = infty)");
    m_parameters.add(false, "Write GT TrafficSigns to folder");
    FileParameter out_dir("ts-gt-output", "TS-GT output folder");
    out_dir.setRefType(RefDir);
    m_parameters.add(out_dir);
}


void VTDSource::close() {
    delete m_rdb_shm_client;
    delete m_rdb_tcp_command_client;
    delete m_rdb_tcp_info_client;
    delete m_camera_sensor;
    delete m_command_connector;
    delete m_info_sensor;
}


VTDInfoSensor::VTDInfoSensor(adsim::vtd::RDBTransceiver* rdb_client)
        : RDBCodec(*rdb_client)
{

}

void VTDInfoSensor::process(RDB_TRAFFIC_SIGN_t* ts_package)
{
    //	std::cout << "\n=============ts" << std::endl;
    //	Framework::RDBHandler::print(*ts_package);
    uint frame_nr = FrameNumber();
    uint id = ts_package->id;
    int type = ts_package->type;
    int subType = ts_package->subType;
    QMap<uint, TrafficSign*> map = m_ts_map.value(frame_nr);

    if (map.contains(id)) {
        TrafficSign* ts = map.value(id);
        ts->frame_nr = frame_nr;
        ts->setType(type);
        ts->subtype = subType;
    } else {
        map.insert(id, new TrafficSign(id, frame_nr, type, subType));
    }

    m_ts_map.insert(frame_nr, map);
}

void VTDInfoSensor::process(RDB_OBJECT_STATE_t* object_state, bool /*extended*/)
{
    //	std::cout << "\n=============object" << std::endl;
    //	Framework::RDBHandler::print(*object_state);
    RDB_OBJECT_STATE_BASE_t os_package = object_state->base;
    uint id = os_package.id;
    uint frame_nr = FrameNumber();
    QMap<uint, TrafficSign*> map = m_ts_map.value(frame_nr);

    QVector3D pos(os_package.pos.x, os_package.pos.y, os_package.pos.z);
    QSize size(os_package.geo.dimX, os_package.geo.dimY);

    if (os_package.type == RDB_OBJECT_TYPE_TRAFFIC_SIGN && map.contains(id)) {
        TrafficSign* ts = map.value(id);
        // set size before moving box, see QRect::setSize
        ts->setSize(size);
        ts->setCenter(pos.toPoint());
        ts->z = os_package.pos.z;
    } else if (os_package.type == RDB_OBJECT_TYPE_TRAFFIC_SIGN) {
        map.insert(id, new TrafficSign(id, pos, size));
    }

    m_ts_map.insert(frame_nr, map);
}

void VTDInfoSensor::process(RDB_ROADMARK_t* rm_package)
{
    //	std::cout << "\n=============lane" << std::endl;
    //	Framework::RDBHandler::print(*rm_package);
    uint frame_nr = FrameNumber();
    int8_t lane_id = rm_package->id;

    //Find roadmarks and extract info, store in QMap
    Lane* lane = new Lane(lane_id, frame_nr,
                          rm_package->prevId, rm_package->nextId);
    RDB_POINT_t* p = (RDB_POINT_t*)(rm_package + 1);

    for (uint i = 0; i < rm_package->noDataPoints; i++) {
        QVector3D tess_p(p[i].x, p[i].y, p[i].z);
        lane->addTesselationPoint(tess_p);
    }

    m_lanes.insert(lane_id, lane);
}

QVector<Lane*> VTDInfoSensor::getLanes()
{
    QVector<Lane*> result;
    QVector<int> visited;

    for (Lane* lane: m_lanes) {
        int8_t id = lane->getID();
        uint frame_nr = lane->getFrameNr();

        if (visited.contains(id)) continue;
        visited.push_back(id);

        int8_t neighbour_id = -1;
        bool stitched = false;

        if ((neighbour_id = lane->getPrevID()) != -1
            && !visited.contains(neighbour_id)) {
            Lane* prev_lane = m_lanes.value(neighbour_id);
            visited.push_back(neighbour_id);
            Lane* new_lane = new Lane(id, frame_nr);
            new_lane->addTesselationPoints(prev_lane->getTesselationPoints());
            new_lane->addTesselationPoints(lane->getTesselationPoints());
            result.push_back(new_lane);
            m_stitched_ids.insert(id);
            m_stitched_ids.insert(neighbour_id);
            stitched = true;
        }

        if ((neighbour_id = lane->getNextID()) != -1
            && !visited.contains(neighbour_id)) {
            Lane* next_lane = m_lanes.value(neighbour_id);
            visited.push_back(neighbour_id);
            Lane* new_lane = new Lane(id, frame_nr);
            new_lane->addTesselationPoints(lane->getTesselationPoints());
            new_lane->addTesselationPoints(next_lane->getTesselationPoints());
            result.push_back(new_lane);
            m_stitched_ids.insert(id);
            m_stitched_ids.insert(neighbour_id);
            stitched = true;
        }

        if (!stitched) result.push_back(lane);

        if (visited.size() == m_lanes.size()) break;
    }

    return result;
}

void VTDInfoSensor::clear(uint frame)
{
    m_ts_map.remove(frame);
    for (int8_t id: m_stitched_ids) {
        delete m_lanes[id];
    }
    m_stitched_ids.clear();
    m_lanes.clear();
}
