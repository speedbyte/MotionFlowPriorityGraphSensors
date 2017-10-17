/**
 * \file      rdb_transceiver_tcp.cpp
 * \copyright 2017 Robert Bosch GmbH
 * \author    Robin Keller <robin.keller@de.bosch.com>
 * \date      9 January 2017
 */

#include <adsim/vtd/rdb_transceiver_tcp.h>

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace adsim {
namespace vtd {

RDBTransceiverTCP::RDBTransceiverTCP(std::string host, std::string port) : RDBTransceiver() {
  stream_.connect(host, port);
  if (stream_.error()) {
    throw std::ios_base::failure(std::string("RDBTransceiverTCP: error while connecting to ") +
                                 host + std::string(":") + port + std::string(" ") +
                                 stream_.error().message());
  }
}

RDBTransceiverTCP::~RDBTransceiverTCP() { stream_.close(); }

void RDBTransceiverTCP::reconnect(std::string host, std::string port) {
  stream_.close();
  stream_.connect(host, port);
}

RDB_MSG_t* RDBTransceiverTCP::getMessage() {
  RDB_MSG_t* msg =
      reinterpret_cast<RDB_MSG_t*>(malloc(sizeof(RDB_MSG_HDR_t)));  // allocate memory for header

  if (NULL == msg) {
    throw std::runtime_error(std::string("RDBTransceiverTCP: malloc failed."));
  }

  // read header
  stream_.read(reinterpret_cast<char*>(msg), sizeof(RDB_MSG_HDR_t));
  if (boost::asio::error::eof == stream_.error()) {
    free(msg);
    throw std::runtime_error(std::string("RDBTransceiverTCP: error during reading: ") +
                             stream_.error().message());
  }

  // check if header is in sync
  if (RDB_MAGIC_NO != msg->hdr.magicNo) {
    free(msg);
    throw std::runtime_error(std::string("RDBTransceiverTCP: magic number does not match") +
                             stream_.error().message());
  }
  assert(sizeof(msg->hdr) == msg->hdr.headerSize &&
         "RDBTransceiverTCP Assert failed: sizeof(msg->hdr) == msg->hdr.headerSize");

  // allocate memory and append data
  msg = reinterpret_cast<RDB_MSG_t*>(realloc(msg, msg->hdr.headerSize + msg->hdr.dataSize));
  stream_.read(reinterpret_cast<char*>(&(msg->entryHdr)), msg->hdr.dataSize);
  if (boost::asio::error::eof == stream_.error()) {
    free(msg);
    throw std::runtime_error(std::string("RDBTransceiverTCP: error during reading") +
                             stream_.error().message());
  }

  return msg;
}

std::vector<RDB_MSG_t*> RDBTransceiverTCP::tryGetMessages() {
  std::vector<RDB_MSG_t*> messages;

  // available in buffer and on socket
  std::streamsize available = stream_.rdbuf()->in_avail() + stream_.rdbuf()->available();

  while (available >= static_cast<int>(sizeof(RDB_MSG_HDR_t))) {
    messages.push_back(getMessage());
    available = stream_.rdbuf()->in_avail() + stream_.rdbuf()->available();
  }

  return messages;
}

void RDBTransceiverTCP::freeMessages(std::vector<RDB_MSG_t*>& messages) {
  for (std::vector<RDB_MSG_t*>::iterator it = messages.begin(); it != messages.end(); ++it) {
    if (NULL != *it) {
      free(*it);
    }
  }
}

void RDBTransceiverTCP::send(RDB_MSG_t* message, size_t size) {
  stream_.write(reinterpret_cast<char*>(message), size);
  stream_.flush();
}

}  // namespace vtd
}  // namespace adsim
