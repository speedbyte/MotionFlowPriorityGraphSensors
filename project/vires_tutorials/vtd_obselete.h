#ifndef VTDCONNECTOR_H
#define VTDCONNECTOR_H

/* External
#include "../PluginChain/BaseSource.h"
#include "IPluginDescription.h"
#include "ImageUtils.h"
#include "../Parameters/ParameterList.h"
#include "../Parameters/ParameterImpl.h"
#include "../PluginChain/BaseSource.h"

#include "IPluginDescription.h"


#include "TrafficSigns.h"
#include "vtdlane.h"

*/

#include <vector>
#include <iosfwd>
#include <sstream>
#include <iomanip>
#include <boost/iostreams/categories.hpp>

#include <QFile>
#include <QTextStream>
#include <QMultiMap>

#include <memory>
#include <QImage>

#include <adsim/vtd/rdb_codec.h>

#include <string>
#include <boost/function.hpp>
#include "viRDBIcd.h"

#include <adsim/vtd/rdb_codec.h>
#include <adsim/vtd/rdb_transceiver_tcp.h>
#include <adsim/vtd/rdb_transceiver_shared_memory.h>

#include <QVector>
#include <QMap>
#include <QSet>

#include <adsim/vtd/rdb_codec.h>
#include <QtCore/QRect>

class VTDConnector;
typedef boost::function<void(size_t, RDB_MSG_t*)> OnData_t;
typedef boost::function<void(RDB_MSG_t*)> OnSimInfo_t;

namespace io = boost::iostreams;



class TrafficSign {

};
class Lane {

};

class ByteImage {


};

class DepthImage {

};




class BaseSource {
public:
    BaseSource() {}
private:
    bool data_ready;
    virtual bool open()=0;
    virtual void close()=0;
    virtual void init()=0;
    virtual void fillDefaultParameters()=0;
    virtual void fetchParamValues()=0;
    virtual bool load()=0;

};




class VTDBroadcastComm : public BaseSource
{

    struct msg_data
    {
        uint16_t type;
        uint32_t playerId;
        //uint8_t* msgData;
    };

public:

    //! create instance for broadcast messages
    //! @param parameters	[0] ... [1] ...
    VTDBroadcastComm();

    //! read data from source
    //! @param s	stream to write on
    //! @param n 	count of requested bytes
    //! @return 	returns the count of written bytes

    virtual bool open() override;
    virtual void close() override;
    virtual bool load() override;

protected:
    virtual void fillDefaultParameters() override;
    virtual void init() override;
    virtual void fetchParamValues() override;

private:

    void handleMsg(size_t shmNum, RDB_MSG_t* msg); // (base) handling messages, calling further methods (!= 5)
    void handleMsgTS(RDB_MSG_t* msg); // message containing information about traffic signs, protocol 1337
    void handleMsgRI(RDB_MSG_t* msg); // message containing information about road, protocol 1338
    void handleMsgOV(RDB_MSG_t* msg); // message containing information about other vehicles, protocol 1339
    void handleMsgGC(RDB_MSG_t* msg); // message containing generic information eg. hello world, protocol 42

    void printMsgInfo(RDB_MSG_t* msg);

    RDB_MSG_t* composeMsg(uint16_t type, uint32_t sender, char* data);
    bool sendMessage(RDB_MSG_t* msg);

    VTDConnector* m_vtdConnector;
    bool data_ready;

    //const int FREE_FLAG = RDB_SHM_BUFFER_FLAG_TC;
    //const unsigned int SHM_IMG_BUFFER = 0x0811b;
};



/**
 * @brief Sensor to connect to shm and fetch images from VTD
 */
class VTDCameraSensor : public adsim::vtd::RDBCodec
{
public:
    explicit VTDCameraSensor(adsim::vtd::RDBTransceiver* rdb_client);

    void process() override { adsim::vtd::RDBCodec::process(); }

    std::unique_ptr<ByteImage>* getLeftImage() { return m_left_img; }
    std::unique_ptr<ByteImage>* getRightImage() { return m_right_img; }
    std::unique_ptr<DepthImage>* getDepthImage() { return m_depth_img; }

    /**
     * @brief Cut part of the left image
     * @param rect The part to cut from the image
     * @return Cut of the image
     */
    QImage getCutFromLeftImage(QRect rect);

    /**
     * @brief Check if all three images (left, right, depth) were fetched from
     *        shm.
     * @return True if all images are ready
     */
    bool hasData() { return m_left_img && m_right_img && m_depth_img; }

    /**
     * @brief Reset all pointers; the data gets freed in plugins down the chain
     */
    void clear() { m_left_img = nullptr; m_right_img = nullptr; m_depth_img = nullptr; }

    const RDB_IMAGE_t& getImageInfo() const { return m_image_info; }
    const RDB_CAMERA_t& getCameraInfo() const { return m_camera_info; }
    void printImageInfo() { Framework::RDBHandler::print(m_image_info); }
    void printCameraInfo() { Framework::RDBHandler::print(m_camera_info); }

protected:
    // overwriting and implementing the process methods to obtain data and store locally
    void process(RDB_CAMERA_t* camera) override;
    void process(RDB_IMAGE_t* image) override;

    // Camera Info
    RDB_CAMERA_t m_camera_info;

    // RDB image information
    RDB_IMAGE_t m_image_info;

    // images
    std::unique_ptr<ByteImage>* m_left_img;
    std::unique_ptr<ByteImage>* m_right_img;
    std::unique_ptr<DepthImage>* m_depth_img;

private:
    /**
     * @brief Convert the vtd depth image to real depth
     * @param image depth image to be converted in place
     */
    void convertToDepth(DepthImage& image);

    /**
     * @brief Copy char buffer image data to a ByteImage or DepthImage
     * @param buf_img char buffer containing the image from vtd
     * @param image Empty but initialized ByteImage or DepthImage
     */
    template<typename T>
    void copyBuffToImageData(uchar* buf_img, T& image);
};


class VTDInfoSensor : public adsim::vtd::RDBCodec
{
public:
    explicit VTDInfoSensor(adsim::vtd::RDBTransceiver* rdb_client);

    void process() override { adsim::vtd::RDBCodec::process(); }

    QList<TrafficSign*> getTrafficSigns(uint frame) { return m_ts_map.value(frame).values(); }

    /**
     * @brief Stitch all collected lanes together and return them
     * @return Stitched lanes
     */
    QVector<Lane*> getLanes();

    /**
     * @brief Clear all traffic signs and lanes from a given frame
     * @param frame Number of frame to delete the data from
     */
    void clear(uint frame);

protected:
    void process(RDB_TRAFFIC_SIGN_t* traffic_sign) override;
    void process(RDB_OBJECT_STATE_t* object_state, bool extended) override;
    void process(RDB_ROADMARK_t* roadmark) override;

private:

    // Maps TrafficSign id to TrafficSign, sorted by framenumber
    QMap<uint, QMap<uint, TrafficSign*>> m_ts_map;

    // mapping lane id to Lane (needed for stitching lanes together)
    QMap<int8_t, Lane*> m_lanes;
    QSet<int8_t> m_stitched_ids;
};



/**
 * @brief Source for shared memory / tcp access to Vires VTD
 */
class VTDSource : public BaseSource {
public:
    //! create instance of VTD source
    VTDSource() {}
    /**
     * @brief Open all connections to VTD and send first trigger
     * @return true if all connections were successful
     */
    bool open() override;
    void close() override;
    /**
     * @brief Wait for new images in shm and load those.
     * @return true if all images for a new frame were loaded
     */
    bool load() override;
protected:

    void init() override;
    void fillDefaultParameters() override;
    void fetchParamValues() override;

    /**
     * @brief Copy all images and data from the sensors to the plugin outputs
     *        and (on demand) write the image cut outs of the traffic signs.
     */
    void copyDataToOutputs();

private:

    //Flags for VTD
    const unsigned int SHM_IMG_BUFFER = 0x0811b;  //! key
    const int FREE_FLAG = RDB_SHM_BUFFER_FLAG_TC; //! mask
    std::string m_server;     //! ip address of vtd server
    uint m_sensor_port;     //! vtd server port for receiving information packages
    uint m_port;     //! vtd server port for triggering
    adsim::vtd::RDBTransceiverSharedMemory* m_rdb_shm_client;     //! Connector to VTD via RDB shm
    adsim::vtd::RDBTransceiverTCP* m_rdb_tcp_info_client;  //! Connector to VTD via RDB tcp (for receiving information)
    adsim::vtd::RDBTransceiverTCP* m_rdb_tcp_command_client;     //! Connector to VTD via RDB tcp (for triggering)
    VTDCameraSensor* m_camera_sensor;     //! sensor to get and process the images via rdb shm
    VTDInfoSensor* m_info_sensor;   //! sensor to get and process the ground truth data (lanes, signs, ...) via rdb tcp
    adsim::vtd::RDBCodec* m_command_connector;  //! "sensor" to send triggers via rdb tcp (different port than gt data)
    int m_previous_frame_nr;     //! Store previous frame to signal frame dropping
    bool m_triggers;     //! whether to trigger VTD or not
    int m_frames_to_read;     //! number of frames to read, -1 for infty
    bool m_write_ts_gt;     //! whether to save the traffic sign ground truth images
    std::string m_ts_gt_out_dir;     //! ... and if, where to save them
};



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
