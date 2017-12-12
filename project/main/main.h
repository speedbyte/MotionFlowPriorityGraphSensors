//
// Created by veikas on 01.11.17.
//

#ifndef VIRES_TUTORIAL_MAIN_H
#define VIRES_TUTORIAL_MAIN_H




#define DEFAULT_PORT        48190   /* for image port it should be 48192 */
#define DEFAULT_BUFFER      204800

#define DEFAULT_RX_PORT     48185   /* for image port it should be 48192 */
#define DEFAULT_TX_PORT     48191

class MyRDBHandler : Framework::RDBHandler {

public:
    void parseStartOfFrame(const double &simTime, const unsigned int &simFrame);

    void parseEndOfFrame( const double & simTime, const unsigned int & simFrame );

    void parseEntry( RDB_GEOMETRY_t *                 data, const double & simTime, const unsigned int & simFrame,
                     const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const
                     unsigned int & totalElem ) {}
    void parseEntry( RDB_COORD_SYSTEM_t *             data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_COORD_t *                    data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_ROAD_POS_t *                 data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_LANE_INFO_t *                data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_ROADMARK_t *                 data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}

    void parseEntry( RDB_OBJECT_CFG_t *data, const double & simTime, const unsigned int &
    simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem );

    void parseEntry( RDB_OBJECT_STATE_t *data, const double & simTime, const unsigned int & simFrame, const unsigned
    short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem );

    void parseEntry( RDB_VEHICLE_SYSTEMS_t *          data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_VEHICLE_SETUP_t *            data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_ENGINE_t *                   data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_DRIVETRAIN_t *               data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_WHEEL_t *                    data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_PED_ANIMATION_t *            data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_SENSOR_STATE_t *             data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_SENSOR_OBJECT_t *            data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_CAMERA_t *                   data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_CONTACT_POINT_t *            data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_TRAFFIC_SIGN_t *             data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_ROAD_STATE_t *               data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}

    void parseEntry( RDB_IMAGE_t *data, const double & simTime, const unsigned int & simFrame, const unsigned
    short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem );

    void parseEntry( RDB_LIGHT_SOURCE_t *             data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_ENVIRONMENT_t *              data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_TRIGGER_t *                  data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_DRIVER_CTRL_t *              data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_TRAFFIC_LIGHT_t *            data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_SYNC_t *                     data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_DRIVER_PERCEPTION_t *        data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_FUNCTION_t *                 data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_ROAD_QUERY_t *               data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_POINT_t *                    data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_TRAJECTORY_t *               data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_CUSTOM_SCORING_t *           data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_DYN_2_STEER_t *              data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_SCP_t *                      data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_STEER_2_DYN_t *              data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_PROXY_t *                    data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_MOTION_SYSTEM_t *            data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_FREESPACE_t *                data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_DYN_EL_SWITCH_t *            data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_DYN_EL_DOF_t *               data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_IG_FRAME_t *                 data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_RT_PERFORMANCE_t *           data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}
    void parseEntry( RDB_CUSTOM_OBJECT_CTRL_TRACK_t * data, const double & simTime, const unsigned int & simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem ) {}

    void parseMessage( RDB_MSG_t* msg );

};

#endif //VIRES_TUTORIAL_MAIN_H
