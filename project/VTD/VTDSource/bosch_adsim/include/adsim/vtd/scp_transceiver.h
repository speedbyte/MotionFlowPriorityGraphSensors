/**
 * \file scp_transceiver.h
 * \copyright 2017 Robert Bosch GmbH
 * \author Robin Keller <robin.keller@de.bosch.com>
 * \date 22 December 2016
 */

#ifndef ADSIM_VTD_SCP_TRANSCEIVER_H_
#define ADSIM_VTD_SCP_TRANSCEIVER_H_

#include <string>

#include <boost/asio.hpp>

#include "scpIcd.h"

namespace adsim {
namespace vtd {

/**
 * TCP connection to the SCP server.
 */
class SCPClient {
 public:
  /**
   * Constructor.
   * Create object and connects to the SCP server.
   *
   * \param host host of the SCP server
   * \param port port of the SCP server
   */
  SCPClient(std::string host, std::string port);

  virtual ~SCPClient();

  /**
   * Blocking method to get a SCP message.
   * This method is blocking and waits until a SCP message is received.
   * It allocates memory for the SCP message!
   *
   * \return pointer to the received SCP message
   */
  std::string* getMessage();

  /**
   * Quasi-non-blocking method to get a SCP message.
   * This method checks if the start of a message exists. If the start of
   * a message exists, the message is returned.  It allocates memory for the
   * SCP message!
   *
   * \return pointer to the received SCP message
   */
  std::string* tryGetMessage();

  /**
   * Frees the memory of the SCP message.
   *
   * \param message pointer to a SCP message
   */
  void freeMessage(std::string* message);

  /**
   * Sends the SCP message.
   *
   * \param message SCP message
   */
  void send(const char* message);

 private:
  /// TCP stream to the SCP server
  boost::asio::ip::tcp::iostream stream_;
};

}  // namespace vtd
}  // namespace adsim

#endif  // ADSIM_VTD_SCP_TRANSCEIVER_H_
