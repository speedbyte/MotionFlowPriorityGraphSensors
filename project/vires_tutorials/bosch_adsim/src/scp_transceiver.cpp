/**
 * \file scp_transceiver.cpp
 * \copyright 2017 Robert Bosch GmbH
 * \author Robin Keller <robin.keller@de.bosch.com>
 * \date 22 December 2016
 */

#include <adsim/vtd/scp_transceiver.h>

#include <iostream>
#include <string>

namespace adsim {
namespace vtd {

SCPClient::SCPClient(std::string host, std::string port) {
  stream_.connect(host, port);
  if (stream_.error()) {
    throw std::runtime_error(std::string("SCPClient: error during connect: ") +
                             stream_.error().message());
  }
}

SCPClient::~SCPClient() { stream_.close(); }

std::string* SCPClient::getMessage() {
  SCP_MSG_HDR_t msg_hdr;
  std::string* msg = NULL;

  // read header
  stream_.read(reinterpret_cast<char*>(&msg_hdr), sizeof(SCP_MSG_HDR_t));
  if (boost::asio::error::eof == stream_.error()) {
    free(msg);
    throw std::runtime_error(std::string("SCPClient: error during reading: ") +
                             stream_.error().message());
  }

  // check if header is in sync
  if (SCP_MAGIC_NO != msg_hdr.magicNo) {
    std::cerr << "SCPClient Error: magic number does not match" << std::endl;
    return msg;
  }

  // check if version number matches
  if (SCP_VERSION != msg_hdr.version) {
    std::cerr << "SCPClient Error: version number does not match" << std::endl;
    return msg;
  }

  // allocate memory and append data
  msg = new std::string();
  msg->reserve(msg_hdr.dataSize);

  stream_.read(&((*msg)[0]), msg_hdr.dataSize);
  if (boost::asio::error::eof == stream_.error()) {
    delete msg;
    msg = NULL;
    throw std::runtime_error(std::string("SCPClient: error during reading") +
                             stream_.error().message());
  }

  return msg;
}

std::string* SCPClient::tryGetMessage() {
  // available in sequence and on socket
  std::streamsize available = stream_.rdbuf()->in_avail() + stream_.rdbuf()->available();

  return (available >= static_cast<int>(sizeof(SCP_MSG_HDR_t))) ? getMessage() : NULL;
}

void SCPClient::freeMessage(std::string* msg) {
  if (NULL != msg) {
    delete msg;
  }
}

void SCPClient::send(const char* message) {
  SCP_MSG_HDR_t msg_hdr;

  msg_hdr.magicNo = SCP_MAGIC_NO;
  msg_hdr.version = SCP_VERSION;
  // TODO(robin): make sender and receiver name const variable
  snprintf(msg_hdr.sender, SCP_NAME_LENGTH, "adsim_vtd");
  snprintf(msg_hdr.receiver, SCP_NAME_LENGTH, "any");
  msg_hdr.dataSize = strlen(message);

  stream_.write(reinterpret_cast<char*>(&msg_hdr), sizeof(msg_hdr));
  stream_.write(message, msg_hdr.dataSize);
  stream_.flush();
}

}  // namespace vtd
}  // namespace adsim
