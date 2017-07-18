#include "VTDBroadcastComm.h"
#include <QDebug>
#include <boost/bind.hpp>
#include "RDBHandler.hh"
#include <sstream>


VTDBroadcastComm::VTDBroadcastComm(GlobalObjectRegistry* gobjreg, const ParameterList* parameters) : BaseSource(gobjreg, parameters)
{
    data_ready = false;
}


void VTDBroadcastComm::init()
{
  fetchParamValues();
}


// TODO
bool VTDBroadcastComm::open(){
#ifdef __unix__
  std::string server = m_parameters.getString(0);
  int port = m_parameters.getInt(1);
  int sensor_port = m_parameters.getInt(2);

  m_vtdConnector = new VTDConnector(1, 0, server, port);
  m_vtdConnector->registerDataCallback(boost::bind(&VTDBroadcastComm::handleMsg, this, _1 ,_2));
  std::cout << "VTD opened" << std::endl;
  return m_vtdConnector->open();
#else
  return false;
#endif
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
