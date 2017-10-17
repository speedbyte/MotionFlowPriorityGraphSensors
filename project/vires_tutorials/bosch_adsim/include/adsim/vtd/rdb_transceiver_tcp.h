/**
 * \file rdb_transceiver_tcp.h
 * \copyright 2017 Robert Bosch GmbH
 * \author Robin Keller <robin.keller@de.bosch.com>
 * \date 9 January 2017
 */

#ifndef ADSIM_VTD_RDB_TRANSCEIVER_TCP_H_
#define ADSIM_VTD_RDB_TRANSCEIVER_TCP_H_

#include <string>
#include <vector>

#include <boost/asio.hpp>

#include <adsim/vtd/rdb_transceiver.h>

namespace adsim {
namespace vtd {

/**
 * This class implements an RDB client via TCP.
 */
class RDBTransceiverTCP : public RDBTransceiver {
 public:
  RDBTransceiverTCP(std::string host, std::string port);

  virtual ~RDBTransceiverTCP();

  void reconnect(std::string host, std::string port);

  virtual std::vector<RDB_MSG_t*> tryGetMessages();

  virtual void freeMessages(std::vector<RDB_MSG_t*>& messages);

  virtual void send(RDB_MSG_t* message, size_t size);

 private:
  /// tcp io stream to server
  boost::asio::ip::tcp::iostream stream_;

 protected:
  /**
   * Blocking method to get a RDB message.
   * This method is blocking and waits until a RDB message is received.
   * It allocates memory for the RDB message!
   *
   * \return pointer to the received RBD message
   */
  virtual RDB_MSG_t* getMessage();
};

}  // namespace vtd
}  // namespace adsim

#endif  // ADSIM_VTD_RDB_TRANSCEIVER_TCP_H_
