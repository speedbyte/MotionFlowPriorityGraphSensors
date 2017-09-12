/**
 * \file rdb_camera_sensor.h
 * \copyright 2017 Robert Bosch GmbH
 * \author Robin Keller <robin.keller@de.bosch.com>
 * \date 10 Jan 2017
 */

#ifndef ADSIM_VTD_RDB_CAMERA_SENSOR_H_
#define ADSIM_VTD_RDB_CAMERA_SENSOR_H_

#include <adsim/vtd/rdb_codec.h>

namespace adsim {
namespace vtd {

/**
 * This class implements the a RDB camera sensor.
 */
class RDBCameraSensor : public RDBCodec {
 public:
  /**
   * Constructor
   *
   * \param rdb_client connection to sensor
   */
  explicit RDBCameraSensor(RDBTransceiver& rdb_client);

  virtual void process() { RDBCodec::process(); }

  /**
   * Gets the pointer to the raw image data.
   * All additional information can be obtained by \see getImageInfo().
   *
   * \return Pointer to the image data
   */
  virtual const char* getImage() const { return image_data_; }

  /**
   * Gets the image information of the by \see getImage() provided image.
   *
   * \return the RDB image information of the current image data
   */
  virtual const RDB_IMAGE_t& getImageInfo() const { return image_info_; }

 protected:
  // overwriting and implementing the process methods to obtain data and store locally
  virtual void process(RDB_START_OF_FRAME_t* /*start_of_frame*/) {}
  virtual void process(RDB_END_OF_FRAME_t* /*end_of_frame*/) {}
  virtual void process(RDB_CAMERA_t* /*camera*/) {}
  virtual void process(RDB_IMAGE_t* image);

  /// raw image data
  char* image_data_;

  /// RDB image information of \see image_data_
  RDB_IMAGE_t image_info_;
};

}  // namespace vtd
}  // namespace adsim

#endif  // ADSIM_VTD_RDB_CAMERA_SENSOR_H_
