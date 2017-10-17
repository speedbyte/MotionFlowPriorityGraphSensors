/**
 * \file rdb_codec.h
 * \copyright 2017 Robert Bosch GmbH
 * \author Robin Keller <robin.keller@de.bosch.com>
 * \date 20 December 2016
 */

#include <adsim/vtd/rdb_codec.h>

#include <cassert>
#include <iostream>
#include <vector>

namespace adsim {
namespace vtd {

RDBCodec::RDBCodec(RDBTransceiver& rdb_client)
    : rdb_client_(rdb_client), frame_number_(-1), simulation_time_(-1.0) {
  rdb_handler_.initMsg();
  // rdb_handler_.addPackage(0.0, 0, RDB_PKG_ID_START_OF_FRAME);
}

void RDBCodec::process(RDB_MSG_ENTRY_HDR_t* entry) {
  uint32_t number_elements = 0;
  uint32_t i = 0;
  char* data = NULL;

  if (NULL == entry) {
    return;
  }

  if (0 == entry->elementSize) {
    switch (entry->pkgId) {
      case RDB_PKG_ID_START_OF_FRAME:
        process(reinterpret_cast<RDB_START_OF_FRAME_t*>(data));
        break;

      case RDB_PKG_ID_END_OF_FRAME:
        process(reinterpret_cast<RDB_END_OF_FRAME_t*>(data));
        break;

      default:
        std::cerr << "RDB_CODEC: RDB_PKG_ID " << entry->pkgId << " not implemented" << std::endl;
        break;
    }
  } else {
    number_elements = entry->dataSize / entry->elementSize;
    assert(number_elements * entry->elementSize == entry->dataSize);

    data = reinterpret_cast<char*>(entry) + entry->headerSize;

    for (i = 0; i < number_elements; ++i) {
      switch (entry->pkgId) {
        case RDB_PKG_ID_COORD_SYSTEM:
          process(reinterpret_cast<RDB_COORD_SYSTEM_t*>(data));
          break;

        case RDB_PKG_ID_COORD:
          process(reinterpret_cast<RDB_COORD_t*>(data));
          break;

        case RDB_PKG_ID_ROAD_POS:
          process(reinterpret_cast<RDB_ROAD_POS_t*>(data));
          break;

        case RDB_PKG_ID_LANE_INFO:
          process(reinterpret_cast<RDB_LANE_INFO_t*>(data));
          break;

        case RDB_PKG_ID_ROADMARK:
          process(reinterpret_cast<RDB_ROADMARK_t*>(data));
          break;

        case RDB_PKG_ID_OBJECT_CFG:
          process(reinterpret_cast<RDB_OBJECT_CFG_t*>(data));
          break;

        case RDB_PKG_ID_OBJECT_STATE:
          process(reinterpret_cast<RDB_OBJECT_STATE_t*>(data),
                  entry->flags & RDB_PKG_FLAG_EXTENDED);
          break;

        case RDB_PKG_ID_VEHICLE_SYSTEMS:
          process(reinterpret_cast<RDB_VEHICLE_SYSTEMS_t*>(data));
          break;

        case RDB_PKG_ID_VEHICLE_SETUP:
          process(reinterpret_cast<RDB_VEHICLE_SETUP_t*>(data));
          break;

        case RDB_PKG_ID_ENGINE:
          process(reinterpret_cast<RDB_ENGINE_t*>(data), entry->flags & RDB_PKG_FLAG_EXTENDED);
          break;

        case RDB_PKG_ID_DRIVETRAIN:
          process(reinterpret_cast<RDB_DRIVETRAIN_t*>(data), entry->flags & RDB_PKG_FLAG_EXTENDED);
          break;

        case RDB_PKG_ID_WHEEL:
          process(reinterpret_cast<RDB_WHEEL_t*>(data), entry->flags & RDB_PKG_FLAG_EXTENDED);
          break;

        case RDB_PKG_ID_PED_ANIMATION:
          process(reinterpret_cast<RDB_PED_ANIMATION_t*>(data));
          break;

        case RDB_PKG_ID_SENSOR_STATE:
          process(reinterpret_cast<RDB_SENSOR_STATE_t*>(data));
          break;

        case RDB_PKG_ID_SENSOR_OBJECT:
          process(reinterpret_cast<RDB_SENSOR_OBJECT_t*>(data));
          break;

        case RDB_PKG_ID_CAMERA:
          process(reinterpret_cast<RDB_CAMERA_t*>(data));
          break;

        case RDB_PKG_ID_CONTACT_POINT:
          process(reinterpret_cast<RDB_CONTACT_POINT_t*>(data));
          break;

        case RDB_PKG_ID_TRAFFIC_SIGN:
          process(reinterpret_cast<RDB_TRAFFIC_SIGN_t*>(data));
          break;

        case RDB_PKG_ID_ROAD_STATE:
          process(reinterpret_cast<RDB_ROAD_STATE_t*>(data));
          break;

        case RDB_PKG_ID_IMAGE:
        case RDB_PKG_ID_LIGHT_MAP:
          process(reinterpret_cast<RDB_IMAGE_t*>(data));
          break;

        case RDB_PKG_ID_OCCLUSION_MATRIX:
          // TODO(robin)
          assert(false && "RDB_PKG_ID_OCCLUSION_MATRIX not implemented");
          break;

        case RDB_PKG_ID_LIGHT_SOURCE:
          process(reinterpret_cast<RDB_LIGHT_SOURCE_t*>(data),
                  entry->flags & RDB_PKG_FLAG_EXTENDED);
          break;

        case RDB_PKG_ID_ENVIRONMENT:
          process(reinterpret_cast<RDB_ENVIRONMENT_t*>(data));
          break;

        case RDB_PKG_ID_TRIGGER:
          process(reinterpret_cast<RDB_TRIGGER_t*>(data));
          break;

        case RDB_PKG_ID_DRIVER_CTRL:
          process(reinterpret_cast<RDB_DRIVER_CTRL_t*>(data));
          break;

        case RDB_PKG_ID_TRAFFIC_LIGHT:
          process(reinterpret_cast<RDB_TRAFFIC_LIGHT_t*>(data),
                  entry->flags & RDB_PKG_FLAG_EXTENDED);
          break;

        case RDB_PKG_ID_SYNC:
          process(reinterpret_cast<RDB_SYNC_t*>(data));
          break;

        case RDB_PKG_ID_DRIVER_PERCEPTION:
          process(reinterpret_cast<RDB_DRIVER_PERCEPTION_t*>(data));
          break;

        case RDB_PKG_ID_TONE_MAPPING:
          process(reinterpret_cast<RDB_FUNCTION_t*>(data));
          break;

        case RDB_PKG_ID_ROAD_QUERY:
          process(reinterpret_cast<RDB_ROAD_QUERY_t*>(data));
          break;

        case RDB_PKG_ID_SCP:
          // TODO(robin): relevant?
          assert(false && "RDB_PKG_ID_SCP not implemented");
          break;

        case RDB_PKG_ID_TRAJECTORY:
          process(reinterpret_cast<RDB_TRAJECTORY_t*>(data));
          break;

        case RDB_PKG_ID_DYN_2_STEER:
          process(reinterpret_cast<RDB_DYN_2_STEER_t*>(data));
          break;

        case RDB_PKG_ID_STEER_2_DYN:
          process(reinterpret_cast<RDB_STEER_2_DYN_t*>(data));
          break;

        case RDB_PKG_ID_PROXY:
          process(reinterpret_cast<RDB_PROXY_t*>(data));
          break;

        case RDB_PKG_ID_MOTION_SYSTEM:
          process(reinterpret_cast<RDB_MOTION_SYSTEM_t*>(data));
          break;

        case RDB_PKG_ID_FREESPACE:
          process(reinterpret_cast<RDB_FREESPACE_t*>(data));
          break;

        case RDB_PKG_ID_DYN_EL_SWITCH:
          process(reinterpret_cast<RDB_DYN_EL_SWITCH_t*>(data));
          break;

        case RDB_PKG_ID_DYN_EL_DOF:
          process(reinterpret_cast<RDB_DYN_EL_DOF_t*>(data));
          break;

        case RDB_PKG_ID_IG_FRAME:
          process(reinterpret_cast<RDB_IG_FRAME_t*>(data));
          break;

//        case RDB_PKG_ID_RT_PERFORMANCE:
//          process(reinterpret_cast<RDB_RT_PERFORMANCE_t*>(data));
//          break;

        case RDB_PKG_ID_CUSTOM_SCORING:
          process(reinterpret_cast<RDB_CUSTOM_SCORING_t*>(data));
          break;

        case RDB_PKG_ID_CUSTOM_OBJECT_CTRL_TRACK:
          process(reinterpret_cast<RDB_CUSTOM_OBJECT_CTRL_TRACK_t*>(data));
          break;

        default:
          std::cerr << "RDB_CODEC: RDB_PKG_ID " << entry->pkgId << " not implemented" << std::endl;
          break;
      }
      data = reinterpret_cast<char*>(data) + entry->elementSize;
    }
    assert(reinterpret_cast<char*>(entry) + entry->headerSize + entry->dataSize == data);
  }
}

void RDBCodec::process(RDB_MSG_t* msg) {
  RDB_MSG_ENTRY_HDR_t* entry = NULL;
  uint32_t remaining_bytes = 0;

  if (NULL == msg) {
    return;
  }

  if (0 == msg->hdr.dataSize) {
    return;
  }

  frame_number_ = msg->hdr.frameNo;
  simulation_time_ = msg->hdr.simTime;

  entry =
      reinterpret_cast<RDB_MSG_ENTRY_HDR_t*>(reinterpret_cast<char*>(msg) + msg->hdr.headerSize);
  remaining_bytes = msg->hdr.dataSize;

  while (remaining_bytes > 0) {
    process(entry);
    remaining_bytes -= (entry->headerSize + entry->dataSize);
    entry = reinterpret_cast<RDB_MSG_ENTRY_HDR_t*>(reinterpret_cast<char*>(entry) +
                                                   entry->headerSize + entry->dataSize);
  }
  assert(remaining_bytes == 0);
}

void RDBCodec::process() {
  std::vector<RDB_MSG_t*> messages = rdb_client_.tryGetMessages();

  for (std::vector<RDB_MSG_t*>::iterator it = messages.begin(); it != messages.end(); ++it) {
    process(*it);
  }

  rdb_client_.freeMessages(messages);
}

void RDBCodec::addDriverControl(const uint32_t player_id, const float target_acceleration,
                                const float target_steering, const uint32_t flags,
                                const uint32_t validity_flags) {
  RDB_DRIVER_CTRL_t* driverCtrl =
      reinterpret_cast<RDB_DRIVER_CTRL_t*>(rdb_handler_.addPackage(0.0, 0, RDB_PKG_ID_DRIVER_CTRL));
  if (NULL != driverCtrl) {
    driverCtrl->playerId = player_id;
    driverCtrl->accelTgt = target_acceleration;
    driverCtrl->steeringTgt = target_steering;
    driverCtrl->flags = flags;
    driverCtrl->validityFlags = validity_flags;
  }
}

void RDBCodec::addTriggerAndSend(float delta_t) {
  RDB_TRIGGER_t* trigger =
      reinterpret_cast<RDB_TRIGGER_t*>(rdb_handler_.addPackage(0.0, 0, RDB_PKG_ID_TRIGGER));
  if (NULL != trigger) {
    trigger->deltaT = delta_t;
    trigger->frameNo = 0;
    trigger->features = 0;
  }

  // rdb_handler_.addPackage(0.0, 0, RDB_PKG_ID_END_OF_FRAME);
  rdb_client_.send(rdb_handler_.getMsg(), rdb_handler_.getMsgTotalSize());

  rdb_handler_.initMsg();
  // rdb_handler_.addPackage(0.0, 0, RDB_PKG_ID_START_OF_FRAME);
}

}  // namespace vtd
}  // namespace adsim
