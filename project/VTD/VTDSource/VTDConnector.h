#ifndef VTDCONNECTOR_H
#define VTDCONNECTOR_H
#include <string>
#include <boost/function.hpp>
#include "viRDBIcd.h"

typedef boost::function<void(size_t, RDB_MSG_t*)> OnData_t;
typedef boost::function<void(RDB_MSG_t*)> OnSimInfo_t;


class VTDConnector
{
public:
    VTDConnector(size_t numSHMs, unsigned int keys[], std::string& server, int port);
    ~VTDConnector();
    bool open();
    void getFrame();

    void registerDataCallback(OnData_t callback);
    void registerInfoCallback(OnSimInfo_t callback, size_t num_clients, int* ports);

private:
    bool openNetwork(int port, int* client);
    void readNetwork(int client);
    void sendRDBTrigger( int & sendSocket, const double & simTime, const unsigned int & simFrame );

    int getNoReadyRead(int descriptor);

    bool openShm(size_t index);
    int checkShm(size_t index, bool clearContent);


    size_t        m_numSHMs;
    unsigned int* m_shmKey;                                         // key of the SHM segment
    unsigned int  m_checkMask;
    void**        m_shmPtr;                                         // pointer to the SHM segment
    size_t*       m_shmTotalSize;                                  // remember the total size of the SHM segment
    bool          m_verbose;                                        // run in verbose mode?
    int           m_forceBuffer;                                      // force reading one of the SHM buffers (0=A, 1=B)
    int*          m_haveImage;                       // is an image available?
    std::string   m_server;                                     // Server to connect to
    int           m_port;                         // Port on server to connect to
    int           m_client;                                // client socket
    size_t        m_numSensorClients;                       //Number of sensor clients
    int*          m_sensorPorts;                            // Ports of the sensor RDB messages
    int*          m_sensorClients;                          // Sockets of the sensor RDB messages
    unsigned int  m_simFrame;                              // simulation frame counter
    double        m_simTime;                                // simulation time
    double        m_deltaTime;                               // simulation step width
    int*          m_lastShmFrame;
    int           m_haveFirstImage;

    OnData_t m_onData;
    OnSimInfo_t m_onSimInfo;

    const int DEFAULT_BUFFER = 204800;
};

#endif // VTDCONNECTOR_H
