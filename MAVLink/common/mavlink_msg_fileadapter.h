#pragma once
// MESSAGE FileAdapter PACKING

#define MAVLINK_MSG_ID_FileAdapter 220


typedef struct __mavlink_fileadapter_t {
 uint16_t total_size; /*<  total size*/
 uint16_t pack_size; /*<  pack size*/
 uint16_t pack_id; /*<  pack id*/
 uint8_t Type; /*<  type*/
 uint8_t Version; /*<  version id*/
 uint8_t segment_size; /*<  segment size*/
 uint8_t pack_segment_id; /*<  pack segment id*/
 uint8_t Payload[245]; /*<  payload*/
} mavlink_fileadapter_t;

#define MAVLINK_MSG_ID_FileAdapter_LEN 255
#define MAVLINK_MSG_ID_FileAdapter_MIN_LEN 255
#define MAVLINK_MSG_ID_220_LEN 255
#define MAVLINK_MSG_ID_220_MIN_LEN 255

#define MAVLINK_MSG_ID_FileAdapter_CRC 116
#define MAVLINK_MSG_ID_220_CRC 116

#define MAVLINK_MSG_FileAdapter_FIELD_PAYLOAD_LEN 245

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FileAdapter { \
    220, \
    "FileAdapter", \
    8, \
    {  { "Type", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_fileadapter_t, Type) }, \
         { "Version", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_fileadapter_t, Version) }, \
         { "total_size", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_fileadapter_t, total_size) }, \
         { "pack_size", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_fileadapter_t, pack_size) }, \
         { "segment_size", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_fileadapter_t, segment_size) }, \
         { "pack_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_fileadapter_t, pack_id) }, \
         { "pack_segment_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_fileadapter_t, pack_segment_id) }, \
         { "Payload", NULL, MAVLINK_TYPE_UINT8_T, 245, 10, offsetof(mavlink_fileadapter_t, Payload) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FileAdapter { \
    "FileAdapter", \
    8, \
    {  { "Type", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_fileadapter_t, Type) }, \
         { "Version", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_fileadapter_t, Version) }, \
         { "total_size", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_fileadapter_t, total_size) }, \
         { "pack_size", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_fileadapter_t, pack_size) }, \
         { "segment_size", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_fileadapter_t, segment_size) }, \
         { "pack_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_fileadapter_t, pack_id) }, \
         { "pack_segment_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_fileadapter_t, pack_segment_id) }, \
         { "Payload", NULL, MAVLINK_TYPE_UINT8_T, 245, 10, offsetof(mavlink_fileadapter_t, Payload) }, \
         } \
}
#endif

/**
 * @brief Pack a fileadapter message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param Type  type
 * @param Version  version id
 * @param total_size  total size
 * @param pack_size  pack size
 * @param segment_size  segment size
 * @param pack_id  pack id
 * @param pack_segment_id  pack segment id
 * @param Payload  payload
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fileadapter_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t Type, uint8_t Version, uint16_t total_size, uint16_t pack_size, uint8_t segment_size, uint16_t pack_id, uint8_t pack_segment_id, const uint8_t *Payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FileAdapter_LEN];
    _mav_put_uint16_t(buf, 0, total_size);
    _mav_put_uint16_t(buf, 2, pack_size);
    _mav_put_uint16_t(buf, 4, pack_id);
    _mav_put_uint8_t(buf, 6, Type);
    _mav_put_uint8_t(buf, 7, Version);
    _mav_put_uint8_t(buf, 8, segment_size);
    _mav_put_uint8_t(buf, 9, pack_segment_id);
    _mav_put_uint8_t_array(buf, 10, Payload, 245);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FileAdapter_LEN);
#else
    mavlink_fileadapter_t packet;
    packet.total_size = total_size;
    packet.pack_size = pack_size;
    packet.pack_id = pack_id;
    packet.Type = Type;
    packet.Version = Version;
    packet.segment_size = segment_size;
    packet.pack_segment_id = pack_segment_id;
    mav_array_memcpy(packet.Payload, Payload, sizeof(uint8_t)*245);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FileAdapter_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FileAdapter;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FileAdapter_MIN_LEN, MAVLINK_MSG_ID_FileAdapter_LEN, MAVLINK_MSG_ID_FileAdapter_CRC);
}

/**
 * @brief Pack a fileadapter message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param Type  type
 * @param Version  version id
 * @param total_size  total size
 * @param pack_size  pack size
 * @param segment_size  segment size
 * @param pack_id  pack id
 * @param pack_segment_id  pack segment id
 * @param Payload  payload
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fileadapter_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t Type,uint8_t Version,uint16_t total_size,uint16_t pack_size,uint8_t segment_size,uint16_t pack_id,uint8_t pack_segment_id,const uint8_t *Payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FileAdapter_LEN];
    _mav_put_uint16_t(buf, 0, total_size);
    _mav_put_uint16_t(buf, 2, pack_size);
    _mav_put_uint16_t(buf, 4, pack_id);
    _mav_put_uint8_t(buf, 6, Type);
    _mav_put_uint8_t(buf, 7, Version);
    _mav_put_uint8_t(buf, 8, segment_size);
    _mav_put_uint8_t(buf, 9, pack_segment_id);
    _mav_put_uint8_t_array(buf, 10, Payload, 245);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FileAdapter_LEN);
#else
    mavlink_fileadapter_t packet;
    packet.total_size = total_size;
    packet.pack_size = pack_size;
    packet.pack_id = pack_id;
    packet.Type = Type;
    packet.Version = Version;
    packet.segment_size = segment_size;
    packet.pack_segment_id = pack_segment_id;
    mav_array_memcpy(packet.Payload, Payload, sizeof(uint8_t)*245);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FileAdapter_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FileAdapter;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FileAdapter_MIN_LEN, MAVLINK_MSG_ID_FileAdapter_LEN, MAVLINK_MSG_ID_FileAdapter_CRC);
}

/**
 * @brief Encode a fileadapter struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param fileadapter C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fileadapter_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_fileadapter_t* fileadapter)
{
    return mavlink_msg_fileadapter_pack(system_id, component_id, msg, fileadapter->Type, fileadapter->Version, fileadapter->total_size, fileadapter->pack_size, fileadapter->segment_size, fileadapter->pack_id, fileadapter->pack_segment_id, fileadapter->Payload);
}

/**
 * @brief Encode a fileadapter struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fileadapter C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fileadapter_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_fileadapter_t* fileadapter)
{
    return mavlink_msg_fileadapter_pack_chan(system_id, component_id, chan, msg, fileadapter->Type, fileadapter->Version, fileadapter->total_size, fileadapter->pack_size, fileadapter->segment_size, fileadapter->pack_id, fileadapter->pack_segment_id, fileadapter->Payload);
}

/**
 * @brief Send a fileadapter message
 * @param chan MAVLink channel to send the message
 *
 * @param Type  type
 * @param Version  version id
 * @param total_size  total size
 * @param pack_size  pack size
 * @param segment_size  segment size
 * @param pack_id  pack id
 * @param pack_segment_id  pack segment id
 * @param Payload  payload
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fileadapter_send(mavlink_channel_t chan, uint8_t Type, uint8_t Version, uint16_t total_size, uint16_t pack_size, uint8_t segment_size, uint16_t pack_id, uint8_t pack_segment_id, const uint8_t *Payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FileAdapter_LEN];
    _mav_put_uint16_t(buf, 0, total_size);
    _mav_put_uint16_t(buf, 2, pack_size);
    _mav_put_uint16_t(buf, 4, pack_id);
    _mav_put_uint8_t(buf, 6, Type);
    _mav_put_uint8_t(buf, 7, Version);
    _mav_put_uint8_t(buf, 8, segment_size);
    _mav_put_uint8_t(buf, 9, pack_segment_id);
    _mav_put_uint8_t_array(buf, 10, Payload, 245);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FileAdapter, buf, MAVLINK_MSG_ID_FileAdapter_MIN_LEN, MAVLINK_MSG_ID_FileAdapter_LEN, MAVLINK_MSG_ID_FileAdapter_CRC);
#else
    mavlink_fileadapter_t packet;
    packet.total_size = total_size;
    packet.pack_size = pack_size;
    packet.pack_id = pack_id;
    packet.Type = Type;
    packet.Version = Version;
    packet.segment_size = segment_size;
    packet.pack_segment_id = pack_segment_id;
    mav_array_memcpy(packet.Payload, Payload, sizeof(uint8_t)*245);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FileAdapter, (const char *)&packet, MAVLINK_MSG_ID_FileAdapter_MIN_LEN, MAVLINK_MSG_ID_FileAdapter_LEN, MAVLINK_MSG_ID_FileAdapter_CRC);
#endif
}

/**
 * @brief Send a fileadapter message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_fileadapter_send_struct(mavlink_channel_t chan, const mavlink_fileadapter_t* fileadapter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_fileadapter_send(chan, fileadapter->Type, fileadapter->Version, fileadapter->total_size, fileadapter->pack_size, fileadapter->segment_size, fileadapter->pack_id, fileadapter->pack_segment_id, fileadapter->Payload);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FileAdapter, (const char *)fileadapter, MAVLINK_MSG_ID_FileAdapter_MIN_LEN, MAVLINK_MSG_ID_FileAdapter_LEN, MAVLINK_MSG_ID_FileAdapter_CRC);
#endif
}

#if MAVLINK_MSG_ID_FileAdapter_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_fileadapter_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t Type, uint8_t Version, uint16_t total_size, uint16_t pack_size, uint8_t segment_size, uint16_t pack_id, uint8_t pack_segment_id, const uint8_t *Payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, total_size);
    _mav_put_uint16_t(buf, 2, pack_size);
    _mav_put_uint16_t(buf, 4, pack_id);
    _mav_put_uint8_t(buf, 6, Type);
    _mav_put_uint8_t(buf, 7, Version);
    _mav_put_uint8_t(buf, 8, segment_size);
    _mav_put_uint8_t(buf, 9, pack_segment_id);
    _mav_put_uint8_t_array(buf, 10, Payload, 245);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FileAdapter, buf, MAVLINK_MSG_ID_FileAdapter_MIN_LEN, MAVLINK_MSG_ID_FileAdapter_LEN, MAVLINK_MSG_ID_FileAdapter_CRC);
#else
    mavlink_fileadapter_t *packet = (mavlink_fileadapter_t *)msgbuf;
    packet->total_size = total_size;
    packet->pack_size = pack_size;
    packet->pack_id = pack_id;
    packet->Type = Type;
    packet->Version = Version;
    packet->segment_size = segment_size;
    packet->pack_segment_id = pack_segment_id;
    mav_array_memcpy(packet->Payload, Payload, sizeof(uint8_t)*245);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FileAdapter, (const char *)packet, MAVLINK_MSG_ID_FileAdapter_MIN_LEN, MAVLINK_MSG_ID_FileAdapter_LEN, MAVLINK_MSG_ID_FileAdapter_CRC);
#endif
}
#endif

#endif

// MESSAGE FileAdapter UNPACKING


/**
 * @brief Get field Type from fileadapter message
 *
 * @return  type
 */
static inline uint8_t mavlink_msg_fileadapter_get_Type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field Version from fileadapter message
 *
 * @return  version id
 */
static inline uint8_t mavlink_msg_fileadapter_get_Version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field total_size from fileadapter message
 *
 * @return  total size
 */
static inline uint16_t mavlink_msg_fileadapter_get_total_size(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field pack_size from fileadapter message
 *
 * @return  pack size
 */
static inline uint16_t mavlink_msg_fileadapter_get_pack_size(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field segment_size from fileadapter message
 *
 * @return  segment size
 */
static inline uint8_t mavlink_msg_fileadapter_get_segment_size(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field pack_id from fileadapter message
 *
 * @return  pack id
 */
static inline uint16_t mavlink_msg_fileadapter_get_pack_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field pack_segment_id from fileadapter message
 *
 * @return  pack segment id
 */
static inline uint8_t mavlink_msg_fileadapter_get_pack_segment_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field Payload from fileadapter message
 *
 * @return  payload
 */
static inline uint16_t mavlink_msg_fileadapter_get_Payload(const mavlink_message_t* msg, uint8_t *Payload)
{
    return _MAV_RETURN_uint8_t_array(msg, Payload, 245,  10);
}

/**
 * @brief Decode a fileadapter message into a struct
 *
 * @param msg The message to decode
 * @param fileadapter C-struct to decode the message contents into
 */
static inline void mavlink_msg_fileadapter_decode(const mavlink_message_t* msg, mavlink_fileadapter_t* fileadapter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    fileadapter->total_size = mavlink_msg_fileadapter_get_total_size(msg);
    fileadapter->pack_size = mavlink_msg_fileadapter_get_pack_size(msg);
    fileadapter->pack_id = mavlink_msg_fileadapter_get_pack_id(msg);
    fileadapter->Type = mavlink_msg_fileadapter_get_Type(msg);
    fileadapter->Version = mavlink_msg_fileadapter_get_Version(msg);
    fileadapter->segment_size = mavlink_msg_fileadapter_get_segment_size(msg);
    fileadapter->pack_segment_id = mavlink_msg_fileadapter_get_pack_segment_id(msg);
    mavlink_msg_fileadapter_get_Payload(msg, fileadapter->Payload);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FileAdapter_LEN? msg->len : MAVLINK_MSG_ID_FileAdapter_LEN;
        memset(fileadapter, 0, MAVLINK_MSG_ID_FileAdapter_LEN);
    memcpy(fileadapter, _MAV_PAYLOAD(msg), len);
#endif
}
