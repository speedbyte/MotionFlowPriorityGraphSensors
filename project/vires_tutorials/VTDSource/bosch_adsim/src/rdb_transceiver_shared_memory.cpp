/**
 * \file rdb_transceiver_shared_memory.cpp
 * \copyright 2017 Robert Bosch GmbH
 * \author Robin Keller <robin.keller@de.bosch.com>
 * \date 5 January 2017
 */

#include <adsim/vtd/rdb_transceiver_shared_memory.h>

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>

#include <boost/interprocess/xsi_shared_memory.hpp>
#include <QDebug>

#include <RDBHandler.hh>

namespace adsim {
namespace vtd {

RDBTransceiverSharedMemory::RDBTransceiverSharedMemory(key_t key, uint32_t release_mask)
    : RDBTransceiver(), release_mask_(release_mask) {
  int shm_id = shmget(key, 0, 0);
  int i = 0;

  if (-1 == shm_id) {
    std::cout << "SharedMemoryRDB: Failed to get shared memory ID.";
    exit(12000);
  }

  boost::interprocess::xsi_shared_memory shm(boost::interprocess::open_only, shm_id);
  region_ = boost::interprocess::mapped_region(shm, boost::interprocess::read_write);

  rdb_shm_hdr_ = reinterpret_cast<RDB_SHM_HDR_t *>(region_.get_address());

  if (2 != rdb_shm_hdr_->noBuffers) {
    std::cout << "SharedMemoryRDB: Double buffering required.";
    return;
  }

  // store pointers to RDB_SHM_BUFFER_INFO_t and pointer to RDB_MSG_t for each SHM buffer
  // respectively
  buffer_info_ = new RDB_SHM_BUFFER_INFO_t *[rdb_shm_hdr_->noBuffers];
  rdb_msg_ = new RDB_MSG_t *[rdb_shm_hdr_->noBuffers];

  if (0 == rdb_shm_hdr_->dataSize) {
    std::cout << "ShardeMemoryRDB: 0 == rdb_shm_hdr->dataSize";
    return;
  }

  // Store pointers to RDB_SHM_BUFFER_INFO_t
  buffer_info_[0] = reinterpret_cast<RDB_SHM_BUFFER_INFO_t *>(
      reinterpret_cast<char *>(rdb_shm_hdr_) + rdb_shm_hdr_->headerSize);
  for (i = 1; i < rdb_shm_hdr_->noBuffers; ++i) {
    buffer_info_[i] = reinterpret_cast<RDB_SHM_BUFFER_INFO_t *>(
        reinterpret_cast<char *>(buffer_info_[i - 1]) + buffer_info_[i - 1]->thisSize);
  }

  // Store pointers to first RDB_MSG_t in each RDB_SHM_BUFFER_INFO_t
  for (i = 0; i < rdb_shm_hdr_->noBuffers; ++i) {
    rdb_msg_[i] = reinterpret_cast<RDB_MSG_t *>(reinterpret_cast<char *>(rdb_shm_hdr_) +
                                                buffer_info_[i]->offset);
    // FIXME(ben): No commented code without explanation!
    // memset(rdb_msg_[i], 0, sizeof(RDB_MSG_HDR_t));
    buffer_info_[i]->flags = 0;
  }
}

// TODO(ben): Maybe we should use other memory management tools, like unique_ptr or shared_ptr?
RDBTransceiverSharedMemory::~RDBTransceiverSharedMemory() {
  delete[] buffer_info_;
  delete[] rdb_msg_;
}

std::vector<RDB_MSG_t *> RDBTransceiverSharedMemory::tryGetMessages() {
  int i = 0;
  int buffer_id = -1;

  std::vector<RDB_MSG_t *> messages;

  if (0 == rdb_shm_hdr_->dataSize) {
    return messages;
  }

  // Store pointers to RDB_SHM_BUFFER_INFO_t
  buffer_info_[0] = reinterpret_cast<RDB_SHM_BUFFER_INFO_t *>(
      reinterpret_cast<char *>(rdb_shm_hdr_) + rdb_shm_hdr_->headerSize);
  for (i = 1; i < rdb_shm_hdr_->noBuffers; ++i) {
    buffer_info_[i] = reinterpret_cast<RDB_SHM_BUFFER_INFO_t *>(
        reinterpret_cast<char *>(buffer_info_[i - 1]) + buffer_info_[i - 1]->thisSize);
  }

  // Store pointers to first RDB_MSG_t in each RDB_SHM_BUFFER_INFO_t
  for (i = 0; i < rdb_shm_hdr_->noBuffers; ++i) {
    rdb_msg_[i] = reinterpret_cast<RDB_MSG_t *>(reinterpret_cast<char *>(rdb_shm_hdr_) +
                                                buffer_info_[i]->offset);
  }

  for (i = 0; i < rdb_shm_hdr_->noBuffers; ++i) {
    if (RDB_MAGIC_NO != rdb_msg_[i]->hdr.magicNo) {
      std::cout << "SharedMemoryRDB: magic number does not match: " << rdb_msg_[i]->hdr.magicNo;
      // FIXME(ben): No commented code without explanation!
      // buffer_info_[i]->flags = 0;
      return messages;
    }
  }
  bool is_ready[2] = {((buffer_info_[0]->flags & release_mask_) || !release_mask_) &&
                          !(buffer_info_[0]->flags & RDB_SHM_BUFFER_FLAG_LOCK),
                      ((buffer_info_[1]->flags & release_mask_) || !release_mask_) &&
                          !(buffer_info_[1]->flags & RDB_SHM_BUFFER_FLAG_LOCK)};

  if (is_ready[0] && is_ready[1]) {
    if (rdb_msg_[0]->hdr.frameNo < rdb_msg_[1]->hdr.frameNo) {
      buffer_id = 0;
    } else {
      buffer_id = 1;
    }
  } else if (is_ready[0]) {
    buffer_id = 0;
  } else if (is_ready[1]) {
    buffer_id = 1;
  }

  if (is_ready[0] || is_ready[1]) {
    buffer_info_[buffer_id]->flags |= RDB_SHM_BUFFER_FLAG_LOCK;

    // copy messages from SHM to local memory
    while (RDB_MAGIC_NO == rdb_msg_[buffer_id]->hdr.magicNo) {
      RDB_MSG_t *message = reinterpret_cast<RDB_MSG_t *>(
          malloc(rdb_msg_[buffer_id]->hdr.headerSize + rdb_msg_[buffer_id]->hdr.dataSize));
      memcpy(message, rdb_msg_[buffer_id],
             rdb_msg_[buffer_id]->hdr.headerSize + rdb_msg_[buffer_id]->hdr.dataSize);
      messages.push_back(message);
      rdb_msg_[buffer_id] = reinterpret_cast<RDB_MSG_t *>(
          reinterpret_cast<char *>(rdb_msg_[buffer_id]) + rdb_msg_[buffer_id]->hdr.headerSize +
          rdb_msg_[buffer_id]->hdr.dataSize);
    }

    buffer_info_[buffer_id]->flags &= ~release_mask_;
    buffer_info_[buffer_id]->flags &= ~RDB_SHM_BUFFER_FLAG_LOCK;
  }

  return messages;
}

void RDBTransceiverSharedMemory::freeMessages(std::vector<RDB_MSG_t *> &messages) {
  for (std::vector<RDB_MSG_t *>::iterator it = messages.begin(); it != messages.end(); ++it) {
    if (NULL != *it) {
      free(*it);
    }
  }
}

void RDBTransceiverSharedMemory::send(RDB_MSG_t * /*message*/, size_t /*size*/) {
  std::cout << "RDBClientSharedMemory: send not implemented!";
}

}  // namespace vtd
}  // namespace adsim
