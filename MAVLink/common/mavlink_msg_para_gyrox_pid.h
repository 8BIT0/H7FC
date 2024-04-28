#pragma once
// MESSAGE Para_GyroX_PID PACKING

#define MAVLINK_MSG_ID_Para_GyroX_PID 215


typedef struct __mavlink_para_gyrox_pid_t {
 float gain_p; /*<   Gain P */
 float p_diff_max; /*<   P Diff Max */
 float p_diff_min; /*<   P Diff Min */
 float gain_I; /*<   Gain I */
 float I_Max; /*<   I Max */
 float I_Min; /*<   I Min */
 float gain_D; /*<   Gain D */
} mavlink_para_gyrox_pid_t;

#define MAVLINK_MSG_ID_Para_GyroX_PID_LEN 28
#define MAVLINK_MSG_ID_Para_GyroX_PID_MIN_LEN 28
#define MAVLINK_MSG_ID_215_LEN 28
#define MAVLINK_MSG_ID_215_MIN_LEN 28

#define MAVLINK_MSG_ID_Para_GyroX_PID_CRC 130
#define MAVLINK_MSG_ID_215_CRC 130



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_Para_GyroX_PID { \
    215, \
    "Para_GyroX_PID", \
    7, \
    {  { "gain_p", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_para_gyrox_pid_t, gain_p) }, \
         { "p_diff_max", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_para_gyrox_pid_t, p_diff_max) }, \
         { "p_diff_min", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_para_gyrox_pid_t, p_diff_min) }, \
         { "gain_I", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_para_gyrox_pid_t, gain_I) }, \
         { "I_Max", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_para_gyrox_pid_t, I_Max) }, \
         { "I_Min", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_para_gyrox_pid_t, I_Min) }, \
         { "gain_D", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_para_gyrox_pid_t, gain_D) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_Para_GyroX_PID { \
    "Para_GyroX_PID", \
    7, \
    {  { "gain_p", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_para_gyrox_pid_t, gain_p) }, \
         { "p_diff_max", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_para_gyrox_pid_t, p_diff_max) }, \
         { "p_diff_min", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_para_gyrox_pid_t, p_diff_min) }, \
         { "gain_I", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_para_gyrox_pid_t, gain_I) }, \
         { "I_Max", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_para_gyrox_pid_t, I_Max) }, \
         { "I_Min", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_para_gyrox_pid_t, I_Min) }, \
         { "gain_D", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_para_gyrox_pid_t, gain_D) }, \
         } \
}
#endif

/**
 * @brief Pack a para_gyrox_pid message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param gain_p   Gain P 
 * @param p_diff_max   P Diff Max 
 * @param p_diff_min   P Diff Min 
 * @param gain_I   Gain I 
 * @param I_Max   I Max 
 * @param I_Min   I Min 
 * @param gain_D   Gain D 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_para_gyrox_pid_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float gain_p, float p_diff_max, float p_diff_min, float gain_I, float I_Max, float I_Min, float gain_D)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_Para_GyroX_PID_LEN];
    _mav_put_float(buf, 0, gain_p);
    _mav_put_float(buf, 4, p_diff_max);
    _mav_put_float(buf, 8, p_diff_min);
    _mav_put_float(buf, 12, gain_I);
    _mav_put_float(buf, 16, I_Max);
    _mav_put_float(buf, 20, I_Min);
    _mav_put_float(buf, 24, gain_D);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_Para_GyroX_PID_LEN);
#else
    mavlink_para_gyrox_pid_t packet;
    packet.gain_p = gain_p;
    packet.p_diff_max = p_diff_max;
    packet.p_diff_min = p_diff_min;
    packet.gain_I = gain_I;
    packet.I_Max = I_Max;
    packet.I_Min = I_Min;
    packet.gain_D = gain_D;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_Para_GyroX_PID_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_Para_GyroX_PID;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_Para_GyroX_PID_MIN_LEN, MAVLINK_MSG_ID_Para_GyroX_PID_LEN, MAVLINK_MSG_ID_Para_GyroX_PID_CRC);
}

/**
 * @brief Pack a para_gyrox_pid message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gain_p   Gain P 
 * @param p_diff_max   P Diff Max 
 * @param p_diff_min   P Diff Min 
 * @param gain_I   Gain I 
 * @param I_Max   I Max 
 * @param I_Min   I Min 
 * @param gain_D   Gain D 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_para_gyrox_pid_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float gain_p,float p_diff_max,float p_diff_min,float gain_I,float I_Max,float I_Min,float gain_D)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_Para_GyroX_PID_LEN];
    _mav_put_float(buf, 0, gain_p);
    _mav_put_float(buf, 4, p_diff_max);
    _mav_put_float(buf, 8, p_diff_min);
    _mav_put_float(buf, 12, gain_I);
    _mav_put_float(buf, 16, I_Max);
    _mav_put_float(buf, 20, I_Min);
    _mav_put_float(buf, 24, gain_D);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_Para_GyroX_PID_LEN);
#else
    mavlink_para_gyrox_pid_t packet;
    packet.gain_p = gain_p;
    packet.p_diff_max = p_diff_max;
    packet.p_diff_min = p_diff_min;
    packet.gain_I = gain_I;
    packet.I_Max = I_Max;
    packet.I_Min = I_Min;
    packet.gain_D = gain_D;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_Para_GyroX_PID_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_Para_GyroX_PID;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_Para_GyroX_PID_MIN_LEN, MAVLINK_MSG_ID_Para_GyroX_PID_LEN, MAVLINK_MSG_ID_Para_GyroX_PID_CRC);
}

/**
 * @brief Encode a para_gyrox_pid struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param para_gyrox_pid C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_para_gyrox_pid_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_para_gyrox_pid_t* para_gyrox_pid)
{
    return mavlink_msg_para_gyrox_pid_pack(system_id, component_id, msg, para_gyrox_pid->gain_p, para_gyrox_pid->p_diff_max, para_gyrox_pid->p_diff_min, para_gyrox_pid->gain_I, para_gyrox_pid->I_Max, para_gyrox_pid->I_Min, para_gyrox_pid->gain_D);
}

/**
 * @brief Encode a para_gyrox_pid struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param para_gyrox_pid C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_para_gyrox_pid_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_para_gyrox_pid_t* para_gyrox_pid)
{
    return mavlink_msg_para_gyrox_pid_pack_chan(system_id, component_id, chan, msg, para_gyrox_pid->gain_p, para_gyrox_pid->p_diff_max, para_gyrox_pid->p_diff_min, para_gyrox_pid->gain_I, para_gyrox_pid->I_Max, para_gyrox_pid->I_Min, para_gyrox_pid->gain_D);
}

/**
 * @brief Send a para_gyrox_pid message
 * @param chan MAVLink channel to send the message
 *
 * @param gain_p   Gain P 
 * @param p_diff_max   P Diff Max 
 * @param p_diff_min   P Diff Min 
 * @param gain_I   Gain I 
 * @param I_Max   I Max 
 * @param I_Min   I Min 
 * @param gain_D   Gain D 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_para_gyrox_pid_send(mavlink_channel_t chan, float gain_p, float p_diff_max, float p_diff_min, float gain_I, float I_Max, float I_Min, float gain_D)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_Para_GyroX_PID_LEN];
    _mav_put_float(buf, 0, gain_p);
    _mav_put_float(buf, 4, p_diff_max);
    _mav_put_float(buf, 8, p_diff_min);
    _mav_put_float(buf, 12, gain_I);
    _mav_put_float(buf, 16, I_Max);
    _mav_put_float(buf, 20, I_Min);
    _mav_put_float(buf, 24, gain_D);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_Para_GyroX_PID, buf, MAVLINK_MSG_ID_Para_GyroX_PID_MIN_LEN, MAVLINK_MSG_ID_Para_GyroX_PID_LEN, MAVLINK_MSG_ID_Para_GyroX_PID_CRC);
#else
    mavlink_para_gyrox_pid_t packet;
    packet.gain_p = gain_p;
    packet.p_diff_max = p_diff_max;
    packet.p_diff_min = p_diff_min;
    packet.gain_I = gain_I;
    packet.I_Max = I_Max;
    packet.I_Min = I_Min;
    packet.gain_D = gain_D;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_Para_GyroX_PID, (const char *)&packet, MAVLINK_MSG_ID_Para_GyroX_PID_MIN_LEN, MAVLINK_MSG_ID_Para_GyroX_PID_LEN, MAVLINK_MSG_ID_Para_GyroX_PID_CRC);
#endif
}

/**
 * @brief Send a para_gyrox_pid message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_para_gyrox_pid_send_struct(mavlink_channel_t chan, const mavlink_para_gyrox_pid_t* para_gyrox_pid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_para_gyrox_pid_send(chan, para_gyrox_pid->gain_p, para_gyrox_pid->p_diff_max, para_gyrox_pid->p_diff_min, para_gyrox_pid->gain_I, para_gyrox_pid->I_Max, para_gyrox_pid->I_Min, para_gyrox_pid->gain_D);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_Para_GyroX_PID, (const char *)para_gyrox_pid, MAVLINK_MSG_ID_Para_GyroX_PID_MIN_LEN, MAVLINK_MSG_ID_Para_GyroX_PID_LEN, MAVLINK_MSG_ID_Para_GyroX_PID_CRC);
#endif
}

#if MAVLINK_MSG_ID_Para_GyroX_PID_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_para_gyrox_pid_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float gain_p, float p_diff_max, float p_diff_min, float gain_I, float I_Max, float I_Min, float gain_D)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, gain_p);
    _mav_put_float(buf, 4, p_diff_max);
    _mav_put_float(buf, 8, p_diff_min);
    _mav_put_float(buf, 12, gain_I);
    _mav_put_float(buf, 16, I_Max);
    _mav_put_float(buf, 20, I_Min);
    _mav_put_float(buf, 24, gain_D);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_Para_GyroX_PID, buf, MAVLINK_MSG_ID_Para_GyroX_PID_MIN_LEN, MAVLINK_MSG_ID_Para_GyroX_PID_LEN, MAVLINK_MSG_ID_Para_GyroX_PID_CRC);
#else
    mavlink_para_gyrox_pid_t *packet = (mavlink_para_gyrox_pid_t *)msgbuf;
    packet->gain_p = gain_p;
    packet->p_diff_max = p_diff_max;
    packet->p_diff_min = p_diff_min;
    packet->gain_I = gain_I;
    packet->I_Max = I_Max;
    packet->I_Min = I_Min;
    packet->gain_D = gain_D;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_Para_GyroX_PID, (const char *)packet, MAVLINK_MSG_ID_Para_GyroX_PID_MIN_LEN, MAVLINK_MSG_ID_Para_GyroX_PID_LEN, MAVLINK_MSG_ID_Para_GyroX_PID_CRC);
#endif
}
#endif

#endif

// MESSAGE Para_GyroX_PID UNPACKING


/**
 * @brief Get field gain_p from para_gyrox_pid message
 *
 * @return   Gain P 
 */
static inline float mavlink_msg_para_gyrox_pid_get_gain_p(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field p_diff_max from para_gyrox_pid message
 *
 * @return   P Diff Max 
 */
static inline float mavlink_msg_para_gyrox_pid_get_p_diff_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field p_diff_min from para_gyrox_pid message
 *
 * @return   P Diff Min 
 */
static inline float mavlink_msg_para_gyrox_pid_get_p_diff_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field gain_I from para_gyrox_pid message
 *
 * @return   Gain I 
 */
static inline float mavlink_msg_para_gyrox_pid_get_gain_I(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field I_Max from para_gyrox_pid message
 *
 * @return   I Max 
 */
static inline float mavlink_msg_para_gyrox_pid_get_I_Max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field I_Min from para_gyrox_pid message
 *
 * @return   I Min 
 */
static inline float mavlink_msg_para_gyrox_pid_get_I_Min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field gain_D from para_gyrox_pid message
 *
 * @return   Gain D 
 */
static inline float mavlink_msg_para_gyrox_pid_get_gain_D(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a para_gyrox_pid message into a struct
 *
 * @param msg The message to decode
 * @param para_gyrox_pid C-struct to decode the message contents into
 */
static inline void mavlink_msg_para_gyrox_pid_decode(const mavlink_message_t* msg, mavlink_para_gyrox_pid_t* para_gyrox_pid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    para_gyrox_pid->gain_p = mavlink_msg_para_gyrox_pid_get_gain_p(msg);
    para_gyrox_pid->p_diff_max = mavlink_msg_para_gyrox_pid_get_p_diff_max(msg);
    para_gyrox_pid->p_diff_min = mavlink_msg_para_gyrox_pid_get_p_diff_min(msg);
    para_gyrox_pid->gain_I = mavlink_msg_para_gyrox_pid_get_gain_I(msg);
    para_gyrox_pid->I_Max = mavlink_msg_para_gyrox_pid_get_I_Max(msg);
    para_gyrox_pid->I_Min = mavlink_msg_para_gyrox_pid_get_I_Min(msg);
    para_gyrox_pid->gain_D = mavlink_msg_para_gyrox_pid_get_gain_D(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_Para_GyroX_PID_LEN? msg->len : MAVLINK_MSG_ID_Para_GyroX_PID_LEN;
        memset(para_gyrox_pid, 0, MAVLINK_MSG_ID_Para_GyroX_PID_LEN);
    memcpy(para_gyrox_pid, _MAV_PAYLOAD(msg), len);
#endif
}
