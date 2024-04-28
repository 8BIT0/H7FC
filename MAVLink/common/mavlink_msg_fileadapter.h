#pragma once
// MESSAGE FileAdapter PACKING

#define MAVLINK_MSG_ID_FileAdapter 220


typedef struct __mavlink_fileadapter_t {
 uint16_t total_size; /*<  total size*/
 uint16_t pack_num; /*<  pack size*/
 uint8_t FrameType; /*<  frame type*/
 uint8_t Version; /*<  version id*/
 uint8_t DataType; /*<  data type*/
} mavlink_fileadapter_t;

#define MAVLINK_MSG_ID_FileAdapter_LEN 7
#define MAVLINK_MSG_ID_FileAdapter_MIN_LEN 7
#define MAVLINK_MSG_ID_220_LEN 7
#define MAVLINK_MSG_ID_220_MIN_LEN 7

#define MAVLINK_MSG_ID_FileAdapter_CRC 10
#define MAVLINK_MSG_ID_220_CRC 10



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FileAdapter { \
    220, \
    "FileAdapter", \
    5, \
    {  { "FrameType", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_fileadapter_t, FrameType) }, \
         { "Version", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_fileadapter_t, Version) }, \
         { "DataType", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_fileadapter_t, DataType) }, \
         { "total_size", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_fileadapter_t, total_size) }, \
         { "pack_num", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_fileadapter_t, pack_num) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FileAdapter { \
    "FileAdapter", \
    5, \
    {  { "FrameType", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_fileadapter_t, FrameType) }, \
         { "Version", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_fileadapter_t, Version) }, \
         { "DataType", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_fileadapter_t, DataType) }, \
         { "total_size", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_fileadapter_t, total_size) }, \
         { "pack_num", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_fileadapter_t, pack_num) }, \
         } \
}
#endif

/**
 * @brief Pack a fileadapter message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param FrameType  frame type
 * @param Version  version id
 * @param DataType  data type
 * @param total_size  total size
 * @param pack_num  pack size
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fileadapter_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t FrameType, uint8_t Version, uint8_t DataType, uint16_t total_size, uint16_t pack_num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FileAdapter_LEN];
    _mav_put_uint16_t(buf, 0, total_size);
    _mav_put_uint16_t(buf, 2, pack_num);
    _mav_put_uint8_t(buf, 4, FrameType);
    _mav_put_uint8_t(buf, 5, Version);
    _mav_put_uint8_t(buf, 6, DataType);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FileAdapter_LEN);
#else
    mavlink_fileadapter_t packet;
    packet.total_size = total_size;
    packet.pack_num = pack_num;
    packet.FrameType = FrameType;
    packet.Version = Version;
    packet.DataType = DataType;

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
 * @param FrameType  frame type
 * @param Version  version id
 * @param DataType  data type
 * @param total_size  total size
 * @param pack_num  pack size
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fileadapter_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t FrameType,uint8_t Version,uint8_t DataType,uint16_t total_size,uint16_t pack_num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FileAdapter_LEN];
    _mav_put_uint16_t(buf, 0, total_size);
    _mav_put_uint16_t(buf, 2, pack_num);
    _mav_put_uint8_t(buf, 4, FrameType);
    _mav_put_uint8_t(buf, 5, Version);
    _mav_put_uint8_t(buf, 6, DataType);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FileAdapter_LEN);
#else
    mavlink_fileadapter_t packet;
    packet.total_size = total_size;
    packet.pack_num = pack_num;
    packet.FrameType = FrameType;
    packet.Version = Version;
    packet.DataType = DataType;

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
    return mavlink_msg_fileadapter_pack(system_id, component_id, msg, fileadapter->FrameType, fileadapter->Version, fileadapter->DataType, fileadapter->total_size, fileadapter->pack_num);
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
    return mavlink_msg_fileadapter_pack_chan(system_id, component_id, chan, msg, fileadapter->FrameType, fileadapter->Version, fileadapter->DataType, fileadapter->total_size, fileadapter->pack_num);
}

/**
 * @brief Send a fileadapter message
 * @param chan MAVLink channel to send the message
 *
 * @param FrameType  frame type
 * @param Version  version id
 * @param DataType  data type
 * @param total_size  total size
 * @param pack_num  pack size
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fileadapter_send(mavlink_channel_t chan, uint8_t FrameType, uint8_t Version, uint8_t DataType, uint16_t total_size, uint16_t pack_num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FileAdapter_LEN];
    _mav_put_uint16_t(buf, 0, total_size);
    _mav_put_uint16_t(buf, 2, pack_num);
    _mav_put_uint8_t(buf, 4, FrameType);
    _mav_put_uint8_t(buf, 5, Version);
    _mav_put_uint8_t(buf, 6, DataType);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FileAdapter, buf, MAVLINK_MSG_ID_FileAdapter_MIN_LEN, MAVLINK_MSG_ID_FileAdapter_LEN, MAVLINK_MSG_ID_FileAdapter_CRC);
#else
    mavlink_fileadapter_t packet;
    packet.total_size = total_size;
    packet.pack_num = pack_num;
    packet.FrameType = FrameType;
    packet.Version = Version;
    packet.DataType = DataType;

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
    mavlink_msg_fileadapter_send(chan, fileadapter->FrameType, fileadapter->Version, fileadapter->DataType, fileadapter->total_size, fileadapter->pack_num);
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
static inline void mavlink_msg_fileadapter_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t FrameType, uint8_t Version, uint8_t DataType, uint16_t total_size, uint16_t pack_num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, total_size);
    _mav_put_uint16_t(buf, 2, pack_num);
    _mav_put_uint8_t(buf, 4, FrameType);
    _mav_put_uint8_t(buf, 5, Version);
    _mav_put_uint8_t(buf, 6, DataType);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FileAdapter, buf, MAVLINK_MSG_ID_FileAdapter_MIN_LEN, MAVLINK_MSG_ID_FileAdapter_LEN, MAVLINK_MSG_ID_FileAdapter_CRC);
#else
    mavlink_fileadapter_t *packet = (mavlink_fileadapter_t *)msgbuf;
    packet->total_size = total_size;
    packet->pack_num = pack_num;
    packet->FrameType = FrameType;
    packet->Version = Version;
    packet->DataType = DataType;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FileAdapter, (const char *)packet, MAVLINK_MSG_ID_FileAdapter_MIN_LEN, MAVLINK_MSG_ID_FileAdapter_LEN, MAVLINK_MSG_ID_FileAdapter_CRC);
#endif
}
#endif

#endif

// MESSAGE FileAdapter UNPACKING


/**
 * @brief Get field FrameType from fileadapter message
 *
 * @return  frame type
 */
static inline uint8_t mavlink_msg_fileadapter_get_FrameType(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field Version from fileadapter message
 *
 * @return  version id
 */
static inline uint8_t mavlink_msg_fileadapter_get_Version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field DataType from fileadapter message
 *
 * @return  data type
 */
static inline uint8_t mavlink_msg_fileadapter_get_DataType(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
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
 * @brief Get field pack_num from fileadapter message
 *
 * @return  pack size
 */
static inline uint16_t mavlink_msg_fileadapter_get_pack_num(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
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
    fileadapter->pack_num = mavlink_msg_fileadapter_get_pack_num(msg);
    fileadapter->FrameType = mavlink_msg_fileadapter_get_FrameType(msg);
    fileadapter->Version = mavlink_msg_fileadapter_get_Version(msg);
    fileadapter->DataType = mavlink_msg_fileadapter_get_DataType(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FileAdapter_LEN? msg->len : MAVLINK_MSG_ID_FileAdapter_LEN;
        memset(fileadapter, 0, MAVLINK_MSG_ID_FileAdapter_LEN);
    memcpy(fileadapter, _MAV_PAYLOAD(msg), len);
#endif
}
