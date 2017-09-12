/**
 * \file      rdb_codec.h
 * \copyright 2017 Robert Bosch GmbH
 * \author    Robin Keller <robin.keller@de.bosch.com>
 * \date      20 December 2016
 */

#ifndef ADSIM_VTD_RDB_CODEC_H_
#define ADSIM_VTD_RDB_CODEC_H_

// This comes from VTD:
#include <RDBHandler.hh>

#include <adsim/vtd/rdb_transceiver.h>

namespace adsim {
namespace vtd {

/**
 * Base class for a VTD sensor which is connected via RDB.
 */
class RDBCodec {
 public:
  /**
   * Constructor.
   *
   * \param rdb_client connection to sensor
   */
  explicit RDBCodec(RDBTransceiver& rdb_client);

  virtual ~RDBCodec() {}

  /// Processes the incoming messages.
  virtual void process();

  /**
   * Returns the last processed frame number.
   *
   * \return the frame number of the last processed frame
   */
  virtual int FrameNumber() const { return frame_number_; }

  /**
   * Return the simulation time of the last processed frame.
   *
   * \return the simulation time of the last processed frame
   */
  virtual double SimulationTime() const { return simulation_time_; }

  /**
   * Adds driver control to the RDB message for the current frame.
   * \see addTriggerAndSend
   *
   * \param player_id VTD player id
   * \param target_acceleration target acceleration in [m/s^2]
   * \param target_steering target steering angle at wheels in [rad]
   * \param flags RDB_DRIVER_FLAG_INDICATOR_L, RDB_DRIVER_FLAG_INDICATOR_R,
   * RDB_DRIVER_FLAG_PARKING_BRAKE
   * \param validity_flags RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING,
   * RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL, RDB_DRIVER_INPUT_VALIDITY_ADD_ON,
   * RDB_DRIVER_INPUT_VALIDITY_FLAGS
   */
  void addDriverControl(const uint32_t player_id, const float target_acceleration,
                        const float target_steering, const uint32_t flags,
                        const uint32_t validity_flags);

  /**
   * Adds the trigger package and sends the packed RDB message to the task control server.
   *
   * \param delta_t time step in [s]
   */
  void addTriggerAndSend(float delta_t);

 protected:
  /**
   * Processes a RDB message.
   * This method is called in process() for each RDB message.
   *
   * \param msg message to be processed
   */
  virtual void process(RDB_MSG_t* msg);

  /**
   * Processes each RDB message entry.
   * This method is called in process(RDB_MSG_t) for each entry in a message.
   *
   * \param entry RDB message entry to be processed
   */
  virtual void process(RDB_MSG_ENTRY_HDR_t* entry);

  /**
   * Processes a start of frame.
   *
   * \param start_of_frame only used for dynamic binding; do not access.
   */
  virtual void process(RDB_START_OF_FRAME_t* /*start_of_frame*/) {}

  virtual void process(RDB_END_OF_FRAME_t* /*end_of_frame*/) {}
  virtual void process(RDB_COORD_SYSTEM_t* /*coord_system*/) {}
  virtual void process(RDB_COORD_t* /*coord*/) {}
  virtual void process(RDB_ROAD_POS_t* /*road_pos*/) {}
  virtual void process(RDB_LANE_INFO_t* /*lane_info*/) {}
  virtual void process(RDB_ROADMARK_t* /*roadmark*/) {}
  virtual void process(RDB_OBJECT_CFG_t* /*object_cfg*/) {}
  virtual void process(RDB_OBJECT_STATE_t* /*object_state*/, bool /*extended*/) {}
  virtual void process(RDB_VEHICLE_SYSTEMS_t* /*vehicle_systems*/) {}
  virtual void process(RDB_VEHICLE_SETUP_t* /*vehicle_setup*/) {}
  virtual void process(RDB_ENGINE_t* /*engine*/, bool /*extended*/) {}
  virtual void process(RDB_DRIVETRAIN_t* /*drivetrain*/, bool /*extended*/) {}
  virtual void process(RDB_WHEEL_t* /*wheel*/, bool /*extended*/) {}
  virtual void process(RDB_PED_ANIMATION_t* /*ped_animation*/) {}
  virtual void process(RDB_SENSOR_STATE_t* /*sensor_state*/) {}
  virtual void process(RDB_SENSOR_OBJECT_t* /*sensor_object*/) {}
  virtual void process(RDB_CAMERA_t* /*camera*/) {}
  virtual void process(RDB_CONTACT_POINT_t* /*contact_point*/) {}
  virtual void process(RDB_TRAFFIC_SIGN_t* /*traffic_sign*/) {}
  virtual void process(RDB_ROAD_STATE_t* /*road_state*/) {}
  virtual void process(RDB_IMAGE_t* /*image*/) {}
  virtual void process(RDB_LIGHT_SOURCE_t* /*light_source*/, bool /*extended*/) {}
  virtual void process(RDB_ENVIRONMENT_t* /*environment*/) {}
  virtual void process(RDB_TRIGGER_t* /*trigger*/) {}
  virtual void process(RDB_DRIVER_CTRL_t* /*driver_ctrl*/) {}
  virtual void process(RDB_TRAFFIC_LIGHT_t* /*traffic_light*/, bool /*extended*/) {}
  virtual void process(RDB_SYNC_t* /*sync*/) {}
  virtual void process(RDB_DRIVER_PERCEPTION_t* /*driver_perception*/) {}
  virtual void process(RDB_FUNCTION_t* /*function*/) {}
  virtual void process(RDB_ROAD_QUERY_t* /*road_query*/) {}
  virtual void process(RDB_TRAJECTORY_t* /*trajectory*/) {}
  virtual void process(RDB_DYN_2_STEER_t* /*dyn_to_steer*/) {}
  virtual void process(RDB_STEER_2_DYN_t* /*steer_to_dyn*/) {}
  virtual void process(RDB_PROXY_t* /*proxy*/) {}
  virtual void process(RDB_MOTION_SYSTEM_t* /*motion_system*/) {}
  virtual void process(RDB_FREESPACE_t* /*freepsace*/) {}
  virtual void process(RDB_DYN_EL_SWITCH_t* /*dyn_el_switch*/) {}
  virtual void process(RDB_DYN_EL_DOF_t* /*dyn_el_dof*/) {}
  virtual void process(RDB_IG_FRAME_t* /*ig_frame*/) {}
  // comment by Alex (type not found, bosch special?)
//  virtual void process(RDB_RT_PERFORMANCE_t* /*rt_performance*/) {}
  virtual void process(RDB_CUSTOM_SCORING_t* /*custom_scoring*/) {}
  virtual void process(RDB_CUSTOM_OBJECT_CTRL_TRACK_t* /*custom_object_ctrl_track*/) {}

 private:
  /// Connection via RDB bus, e.g., TCP, SHM, to VTD
  RDBTransceiver& rdb_client_;

  /// Frame number from last processed RDB message
  int frame_number_;

  /// Simulation time from last processed RDB message
  double simulation_time_;

  /// RDBHandler to construct RDB messages convenient
  Framework::RDBHandler rdb_handler_;
};
}  // namespace vtd
}  // namespace adsim

#endif  // ADSIM_VTD_RDB_CODEC_H_
