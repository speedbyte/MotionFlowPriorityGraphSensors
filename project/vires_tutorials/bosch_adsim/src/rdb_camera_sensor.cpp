/**
 * \file rdb_camera_sensor.h
 * \copyright 2017 Robert Bosch GmbH
 * \author Robin Keller <robin.keller@de.bosch.com>
 * \date 10 January 2017
 */

#include <adsim/vtd/rdb_camera_sensor.h>

#include <cstdlib>
#include <cstring>

namespace adsim {
namespace vtd {

RDBCameraSensor::RDBCameraSensor(RDBTransceiver& rdb_client)
    : RDBCodec(rdb_client), image_data_(NULL) {
  memset(&image_info_, 0, sizeof(RDB_IMAGE_t));
}

void RDBCameraSensor::process(RDB_IMAGE_t* image) {
  memcpy(&image_info_, image, sizeof(RDB_IMAGE_t));

  if (NULL == image_data_) {
    image_data_ = reinterpret_cast<char*>(malloc(image_info_.imgSize));
  } else {
    image_data_ = reinterpret_cast<char*>(realloc(image_data_, image_info_.imgSize));
  }
  memcpy(image_data_, reinterpret_cast<char*>(image) + sizeof(RDB_IMAGE_t), image_info_.imgSize);
}

}  // namespace vtd
}  // namespace adsim
