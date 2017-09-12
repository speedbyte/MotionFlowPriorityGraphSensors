/**
 * \file rdb_transceiver_shared_memory.h
 * \copyright 2017 Robert Bosch GmbH
 * \author Robin Keller <robin.keller@de.bosch.com>
 * \date 5 January 2017
 */

#ifndef ADSIM_VTD_RDB_TRANSCEIVER_SHARED_MEMORY_H_
#define ADSIM_VTD_RDB_TRANSCEIVER_SHARED_MEMORY_H_

#include <vector>

#include <boost/interprocess/mapped_region.hpp>

#include <adsim/vtd/rdb_transceiver.h>
#include <viRDBIcd.h>

namespace adsim {
namespace vtd {

/**
 * This class implements an RDB client via shared memory.
 */
class RDBTransceiverSharedMemory : public RDBTransceiver {
 public:
  /**
   * Constructor.
   *
   * \param key to obtain shared memory id
   * \param release_mask mask used by VTD to mask a shared memory region as accessible
   */
  RDBTransceiverSharedMemory(key_t key, uint32_t release_mask);

  virtual ~RDBTransceiverSharedMemory();

  virtual std::vector<RDB_MSG_t*> tryGetMessages();

  virtual void freeMessages(std::vector<RDB_MSG_t*>& messages);

  virtual void send(RDB_MSG_t* message, size_t size);

 protected:
  /// shared memory region
  boost::interprocess::mapped_region region_;

  /// pointer to the shared memory management header
  RDB_SHM_HDR_t* rdb_shm_hdr_;

  /// array of pointers to buffer information
  RDB_SHM_BUFFER_INFO_t** buffer_info_;

  /// array of pointers to rdb messages
  RDB_MSG_t** rdb_msg_;

  /// VTD uses this mask to notify client when data in buffer is ready
  uint32_t release_mask_;
};

}  // namespace vtd
}  // namespace adsim

#endif  // ADSIM_VTD_RDB_TRANSCEIVER_SHARED_MEMORY_H_
