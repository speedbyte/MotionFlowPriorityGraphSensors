/********************************************************
 * Enables communication with simple (custom) messages. *
 ********************************************************
 *
 *
 * (Proxy-)Message containing:
 * ---------------------------
 * uint16 - protocol
 * uint16 - package/message ID
 * uint32 spare[6] - spares
 * uint32 - data size (size of message data following this entry) in bytes
 *
 *
 * Message data must contain:
 * ---------------------------
 * message type
 * sender information
 * actual data to be processed (traffic signs, road information, other vehicles, etc.)
 *
 */

#ifndef VTDBROADCASTCOMM_H
#define VTDBROADCASTCOMM_H

#include <vector>
#include <iosfwd>
#include <sstream>
#include <iomanip>
#include <boost/iostreams/categories.hpp>

#include "../Parameters/ParameterList.h"
#include "../Parameters/ParameterImpl.h"
#include "../PluginChain/BaseSource.h"

#include "IPluginDescription.h"

#include <QFile>
#include <QTextStream>
#include <QMultiMap>

#include "VTDConnector.h"

#include "TrafficSigns.h"
#include "vtdlane.h"

namespace io = boost::iostreams;


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
    VTDBroadcastComm(GlobalObjectRegistry* gobjreg, const ParameterList* parameters = 0);

    //! read data from source
    //! @param s	stream to write on
    //! @param n 	count of requested bytes
    //! @return 	returns the count of written bytes

    virtual bool open();
    virtual void close();
    virtual bool load();

protected:
    virtual void fillDefaultParameters();
    virtual void init();
    virtual void fetchParamValues();

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

PLUGIN_DESCRIPTION(VTDBroadcastComm, "{b01ead36-08c5-11e7-93ae-92361f002671}", Prepairing, Source, "VTD - broadcast communications for vehicles", "VTD Broadcast")


#endif // VTDBROADCASTCOMM_H
