/**
 * \file rdb_transceiver.h
 * \copyright 2017 Robert Bosch GmbH
 * \author Robin Keller <robin.keller@de.bosch.com>
 * \date 20 December 2016
 */

#ifndef ADSIM_VTD_RDB_TRANSCEIVER_H_
#define ADSIM_VTD_RDB_TRANSCEIVER_H_

#include <viRDBIcd.h>
#include <vector>
#include <cstddef> // size_t

namespace adsim {
namespace vtd {

/**
 * Interface for a RDB client connection.
 *
 * TODO(ben): Maybe we should name this RDBClientInterface, because all its
 * methods are pure virtual methods.
 */
class RDBTransceiver {
 public:
// TODO(ben): I don't know why this is commented out...
#if 0
    /**
     * blocking: waits for new messages
     * allocates memory for the message
     */
    virtual std::vector<RDB_MSG_t *> getMessage() = 0;
#endif

  /**
   * Quasi-non-blocking method to get RDB messages.
   * This method checks if the start of a message exists. As long as the start of a message exists,
   * the message is
   * read and put to the resulting vector of RDB messages. It allocates memory for each RDB message!
   *
   * \return vector of RDB messages
   */
  virtual std::vector<RDB_MSG_t*> tryGetMessages() = 0;

  /**
   * Frees the memory of all passed RDB messages.
   *
   * \param messages vector of RDB messages created by \see tryGetMessages
   */
  virtual void freeMessages(std::vector<RDB_MSG_t*>& messages) = 0;

  /**
   * Sends the RDB message with the given size.
   *
   * \param message RDB message
   * \param size number of bytes
   */
  virtual void send(RDB_MSG_t* message, size_t size) = 0;
};

}  // namespace vtd
}  // namespace adsim

#endif  // ADSIM_VTD_RDB_TRANSCEIVER_H_
