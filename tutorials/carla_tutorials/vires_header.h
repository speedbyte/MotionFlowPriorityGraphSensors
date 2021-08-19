//
// Created by veikas on 13.10.18.
//

#ifndef CARLA_TUTORIALS_VIRES_HEADER_H
#define CARLA_TUTORIALS_VIRES_HEADER_H


ushort frame_number = (ushort) ((simFrame - MAX_DUMPS - 2) / IMAGE_SKIP_FACTOR_DYNAMIC);
const char marker[] = "$$";
//std::cout << sizeof(marker) << " " << sizeof(frame_number) << " " << sizeof(RDB_SENSOR_STATE_t) << std::endl;
fstream_output_sensor_state.write(marker, sizeof(marker-1));
fstream_output_sensor_state.write((char *)&frame_number, sizeof(frame_number));
fstream_output_sensor_state.write((char *)data, sizeof(RDB_SENSOR_STATE_t));

typedef struct
{
    uint32_t    id;                          /**< id of the sensor                                      @unit _                                     @version 0x0100 */
    uint8_t     type;                        /**< type of the sensor                                    @unit @link RDB_SENSOR_TYPE     @endlink    @version 0x0100 */
    uint8_t     hostCategory;                /**< category of the object hosting the sensor             @unit @link RDB_OBJECT_CATEGORY @endlink    @version 0x0100 */
    uint16_t    spare0;                      /**< for future use                                        @unit _                                     @version 0x0100 */
    uint32_t    hostId;                      /**< unique id of the sensor's host                        @unit _                                     @version 0x0100 */
    char        name[RDB_SIZE_OBJECT_NAME];  /**< name of the sensor                                    @unit _                                     @version 0x0100 */
    float       fovHV[2];                    /**< field-of-view (horizontal/vertical)                   @unit rad,rad                               @version 0x0100 */
    float       clipNF[2];                   /**< clipping ranges (near/far)                            @unit m,m                                   @version 0x0100 */
    RDB_COORD_t pos;                         /**< position and orientation of sensor's reference point  @unit m,m,m,rad,rad,rad                     @version 0x0100 */
    RDB_COORD_t originCoordSys;              /**< position and orientation of sensor's coord origin     @unit m,m,m,rad,rad,rad                     @version 0x0100 */
    float       fovOffHV[2];                 /**< field-of-view offset (horizontal/vertical)            @unit rad, rad                              @version 0x011B */
    int32_t     spare[2];                    /**< for future use                                        @unit _                                     @version 0x0100 */
} RDB_SENSOR_STATE_t;

// ---------------------------------------------------------------------------

ushort frame_number = (ushort) ((simFrame - (MAX_DUMPS+2)) / IMAGE_SKIP_FACTOR_DYNAMIC);
const char marker[] = "$$";
//std::cout << sizeof(marker) << " " << sizeof(frame_number) << " " << sizeof(RDB_OBJECT_STATE_t) << std::endl;
fstream_output_object_state.write(marker, sizeof(marker-1));
fstream_output_object_state.write((char *)&frame_number, sizeof(frame_number));
fstream_output_object_state.write((char *)data, sizeof(RDB_OBJECT_STATE_t));

/** ------ complete object data (basic and extended info) ------- */
typedef struct
{
    RDB_OBJECT_STATE_BASE_t base;           /**< state of an object     @unit RDB_OBJECT_STATE_BASE_t   @version 0x0100 */
    RDB_OBJECT_STATE_EXT_t  ext;            /**< extended object data   @unit RDB_OBJECT_STATE_EXT_t    @version 0x0100 */
} RDB_OBJECT_STATE_t;

/** ------ state of an object (may be extended by the next structure) ------- */
typedef struct
{
    uint32_t            id;                         /**< unique object ID                                              @unit _                                  @version 0x0100 */
    uint8_t             category;                   /**< object category                                               @unit @link RDB_OBJECT_CATEGORY @endlink @version 0x0100 */
    uint8_t             type;                       /**< object type                                                   @unit @link RDB_OBJECT_TYPE     @endlink @version 0x0100 */
    uint16_t            visMask;                    /**< visibility mask                                               @unit @link RDB_OBJECT_VIS_FLAG @endlink @version 0x0100 */
    char                name[RDB_SIZE_OBJECT_NAME]; /**< symbolic name                                                 @unit _                                  @version 0x0100 */
    RDB_GEOMETRY_t      geo;                        /**< info about object's geometry                                  @unit m,m,m,m,m,m                        @version 0x0100 */
    RDB_COORD_t         pos;                        /**< position and orientation of object's reference point          @unit m,m,m,rad,rad,rad                  @version 0x0100 */
    uint32_t            parent;                     /**< unique ID of parent object                                    @unit _                                  @version 0x0100 */
    uint16_t            cfgFlags;                   /**< configuration flags                                           @unit @link RDB_OBJECT_CFG_FLAG @endlink @version 0x0100 */
    int16_t             cfgModelId;                 /**< visual model ID (configuration parameter)                     @unit _                                  @version 0x0100 */
} RDB_OBJECT_STATE_BASE_t;

/** ------ extended object data (e.g. for dynamic objects) ------- */
typedef struct
{
    RDB_COORD_t         speed;                      /**< speed and rates                                               @unit m/s,m/s,m/s,rad/s,rad/s,rad/s          @version 0x0100 */
    RDB_COORD_t         accel;                      /**< acceleration                                                  @unit m/s2,m/s2,m/s2,rad/s2,rad/s2/rad/s2    @version 0x0100 */
    float               traveledDist;               /**< traveled distance                                             @unit m                                      @version 0x011a */
    uint32_t            spare[3];                   /**< reserved for future use                                       @unit _                                      @version 0x0100 */
} RDB_OBJECT_STATE_EXT_t;


// ---------------------------------------------------------------------------


ushort frame_number = (ushort) ((simFrame - (MAX_DUMPS+2)) / IMAGE_SKIP_FACTOR_DYNAMIC);
const char marker[] = "$$";
//std::cout << sizeof(marker) << " " << sizeof(frame_number) << " " << sizeof(RDB_SENSOR_OBJECT_t) << std::endl;
fstream_output_sensor_object.write(marker, sizeof(marker-1));
fstream_output_sensor_object.write((char *)&frame_number, sizeof(frame_number));
fstream_output_sensor_object.write((char *)data, sizeof(RDB_SENSOR_OBJECT_t));


/** ------ information about an object registered within a sensor ------ */
typedef struct
{
    uint8_t     category;    /**< object category                                                                @unit @link RDB_OBJECT_CATEGORY    @endlink  @version 0x0100 */
    uint8_t     type;        /**< object type                                                                    @unit @link RDB_OBJECT_TYPE        @endlink  @version 0x0100 */
    uint16_t    flags;       /**< sensor object flags                                                            @unit @link RDB_SENSOR_OBJECT_FLAG @endlink  @version 0x0100 */
    uint32_t    id;          /**< id of the object                                                               @unit _                                      @version 0x0100 */
    uint32_t    sensorId;    /**< id of the detecting sensor                                                     @unit _                                      @version 0x0100 */
    double      dist;        /**< distance between object and referring device                                   @unit m                                      @version 0x0100 */
    RDB_COORD_t sensorPos;   /**< position and orientation of object in sensor coord                             @unit m,m,m,rad,rad,rad                      @version 0x0100 */
    int8_t      occlusion;   /**< degree of occlusion for viewer (-1 = not valid, 0..127 = 0..100% occlusion)    @unit [-1, 0..127]                           @version 0x0100 */
    uint8_t     spare0[3];   /**< for future use                                                                 @unit _                                      @version 0x0100 */
    uint32_t    spare[3];    /**< for future use                                                                 @unit _                                      @version 0x0100 */
} RDB_SENSOR_OBJECT_t;


#endif //CARLA_TUTORIALS_VIRES_HEADER_H
