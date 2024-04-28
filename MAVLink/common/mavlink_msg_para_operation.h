#pragma once
// MESSAGE Para_Operation PACKING

#define MAVLINK_MSG_ID_Para_Operation 214


typedef struct __mavlink_para_operation_t {
 uint16_t Sys_ID; /*<  System ID*/
 uint16_t Comp_ID; /*<  Component ID*/
 uint16_t type_index; /*<  Parameter Type*/
 uint8_t opr_type; /*<  Parameter Operation Type*/
} mavlink_para_operation_t;

#define MAVLINK_MSG_ID_Para_Operation_LEN 7
#define MAVLINK_MSG_ID_Para_Operation_MIN_LEN 7
#define MAVLINK_MSG_ID_214_LEN 7
#define MAVLINK_MSG_ID_214_MIN_LEN 7

#define MAVLINK_MSG_ID_Para_Operation_CRC 186
#define MAVLINK_MSG_ID_214_CRC 186



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_Para_Operation { \
    214, \
    "Para_Operation", \
    4, \
    {  { "Sys_ID", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_para_operation_t, Sys_ID) }, \
         { "Comp_ID", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_para_operation_t, Comp_ID) }, \
         { "type_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_para_operation_t, type_index) }, \
         { "opr_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_para_operation_t, opr_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_Para_Operation { \
    "Para_Operation", \
    4, \
    {  { "Sys_ID", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_para_operation_t, Sys_ID) }, \
         { "Comp_ID", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_para_operation_t, Comp_ID) }, \
         { "type_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_para_operation_t, type_index) }, \
         { "opr_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_para_operation_t, opr_type) }, \
         } \
}
#endif

/**
 * @brief Pack a para_operation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param Sys_ID  System ID
 * @param Comp_ID  Component ID
 * @param type_index  Parameter Type
 * @param opr_type  Parameter Operation Type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_para_operation_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t Sys_ID, uint16_t Comp_ID, uint16_t type_index, uint8_t opr_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_Para_Operation_LEN];
    _mav_put_uint16_t(buf, 0, Sys_ID);
    _mav_put_uint16_t(buf, 2, Comp_ID);
    _mav_put_uint16_t(buf, 4, type_index);
    _mav_put_uint8_t(buf, 6, opr_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_Para_Operation_LEN);
#else
    mavlink_para_operation_t packet;
    packet.Sys_ID = Sys_ID;
    packet.Comp_ID = Comp_ID;
    packet.type_index = type_index;
    packet.opr_type = opr_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_Para_Operation_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_Para_Operation;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_Para_Operation_MIN_LEN, MAVLINK_MSG_ID_Para_Operation_LEN, MAVLINK_MSG_ID_Para_Operation_CRC);
}

/**
 * @brief Pack a para_operation message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param Sys_ID  System ID
 * @param Comp_ID  Component ID
 * @param type_index  Parameter Type
 * @param opr_type  Parameter Operation Type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_para_operation_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t Sys_ID,uint16_t Comp_ID,uint16_t type_index,uint8_t opr_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_Para_Operation_LEN];
    _mav_put_uint16_t(buf, 0, Sys_ID);
    _mav_put_uint16_t(buf, 2, Comp_ID);
    _mav_put_uint16_t(buf, 4, type_index);
    _mav_put_uint8_t(buf, 6, opr_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_Para_Operation_LEN);
#else
    mavlink_para_operation_t packet;
    packet.Sys_ID = Sys_ID;
    packet.Comp_ID = Comp_ID;
    packet.type_index = type_index;
    packet.opr_type = opr_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_Para_Operation_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_Para_Operation;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_Para_Operation_MIN_LEN, MAVLINK_MSG_ID_Para_Operation_LEN, MAVLINK_MSG_ID_Para_Operation_CRC);
}

/**
 * @brief Encode a para_operation struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param para_operation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_para_operation_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_para_operation_t* para_operation)
{
    return mavlink_msg_para_operation_pack(system_id, component_id, msg, para_operation->Sys_ID, para_operation->Comp_ID, para_operation->type_index, para_operation->opr_type);
}

/**
 * @brief Encode a para_operation struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param para_operation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_para_operation_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_para_operation_t* para_operation)
{
    return mavlink_msg_para_operation_pack_chan(system_id, component_id, chan, msg, para_operation->Sys_ID, para_operation->Comp_ID, para_operation->type_index, para_operation->opr_type);
}

/**
 * @brief Send a para_operation message
 * @param chan MAVLink channel to send the message
 *
 * @param Sys_ID  System ID
 * @param Comp_ID  Component ID
 * @param type_index  Parameter Type
 * @param opr_type  Parameter Operation Type
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_para_operation_send(mavlink_channel_t chan, uint16_t Sys_ID, uint16_t Comp_ID, uint16_t type_index, uint8_t opr_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_Para_Operation_LEN];
    _mav_put_uint16_t(buf, 0, Sys_ID);
    _mav_put_uint16_t(buf, 2, Comp_ID);
    _mav_put_uint16_t(buf, 4, type_index);
    _mav_put_uint8_t(buf, 6, opr_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_Para_Operation, buf, MAVLINK_MSG_ID_Para_Operation_MIN_LEN, MAVLINK_MSG_ID_Para_Operation_LEN, MAVLINK_MSG_ID_Para_Operation_CRC);
#else
    mavlink_para_operation_t packet;
    packet.Sys_ID = Sys_ID;
    packet.Comp_ID = Comp_ID;
    packet.type_index = type_index;
    packet.opr_type = opr_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_Para_Operation, (const char *)&packet, MAVLINK_MSG_ID_Para_Operation_MIN_LEN, MAVLINK_MSG_ID_Para_Operation_LEN, MAVLINK_MSG_ID_Para_Operation_CRC);
#endif
}

/**
 * @brief Send a para_operation message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_para_operation_send_struct(mavlink_channel_t chan, const mavlink_para_operation_t* para_operation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_para_operation_send(chan, para_operation->Sys_ID, para_operation->Comp_ID, para_operation->type_index, para_operation->opr_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_Para_Operation, (const char *)para_operation, MAVLINK_MSG_ID_Para_Operation_MIN_LEN, MAVLINK_MSG_ID_Para_Operation_LEN, MAVLINK_MSG_ID_Para_Operation_CRC);
#endif
}

#if MAVLINK_MSG_ID_Para_Operation_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_para_operation_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t Sys_ID, uint16_t Comp_ID, uint16_t type_index, uint8_t opr_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, Sys_ID);
    _mav_put_uint16_t(buf, 2, Comp_ID);
    _mav_put_uint16_t(buf, 4, type_index);
    _mav_put_uint8_t(buf, 6, opr_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_Para_Operation, buf, MAVLINK_MSG_ID_Para_Operation_MIN_LEN, MAVLINK_MSG_ID_Para_Operation_LEN, MAVLINK_MSG_ID_Para_Operation_CRC);
#else
    mavlink_para_operation_t *packet = (mavlink_para_operation_t *)msgbuf;
    packet->Sys_ID = Sys_ID;
    packet->Comp_ID = Comp_ID;
    packet->type_index = type_index;
    packet->opr_type = opr_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_Para_Operation, (const char *)packet, MAVLINK_MSG_ID_Para_Operation_MIN_LEN, MAVLINK_MSG_ID_Para_Operation_LEN, MAVLINK_MSG_ID_Para_Operation_CRC);
#endif
}
#endif

#endif

// MESSAGE Para_Operation UNPACKING


/**
 * @brief Get field Sys_ID from para_operation message
 *
 * @return  System ID
 */
static inline uint16_t mavlink_msg_para_operation_get_Sys_ID(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field Comp_ID from para_operation message
 *
 * @return  Component ID
 */
static inline uint16_t mavlink_msg_para_operation_get_Comp_ID(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field type_index from para_operation message
 *
 * @return  Parameter Type
 */
static inline uint16_t mavlink_msg_para_operation_get_type_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field opr_type from para_operation message
 *
 * @return  Parameter Operation Type
 */
static inline uint8_t mavlink_msg_para_operation_get_opr_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Decode a para_operation message into a struct
 *
 * @param msg The message to decode
 * @param para_operation C-struct to decode the message contents into
 */
static inline void mavlink_msg_para_operation_decode(const mavlink_message_t* msg, mavlink_para_operation_t* para_operation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    para_operation->Sys_ID = mavlink_msg_para_operation_get_Sys_ID(msg);
    para_operation->Comp_ID = mavlink_msg_para_operation_get_Comp_ID(msg);
    para_operation->type_index = mavlink_msg_para_operation_get_type_index(msg);
    para_operation->opr_type = mavlink_msg_para_operation_get_opr_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_Para_Operation_LEN? msg->len : MAVLINK_MSG_ID_Para_Operation_LEN;
        memset(para_operation, 0, MAVLINK_MSG_ID_Para_Operation_LEN);
    memcpy(para_operation, _MAV_PAYLOAD(msg), len);
#endif
}
