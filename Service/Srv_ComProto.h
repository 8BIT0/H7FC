#ifndef __SRV_COMPROTO_H
#define __SRV_COMPROTO_H

#include "../System/runtime/runtime.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "../MAVLink/common/mavlink.h"

typedef bool (*ComProto_Callback)(uint8_t *p_data, uint32_t len);
typedef uint16_t (*DataPack_Callback)(uint8_t *pck);

#define IS_PROTO_OUT(x) (x == SrvComProto_FrameOut) ? true : false

typedef enum
{
    SrvComProto_Type_Cus = 0,
    SrvComProto_Type_MAV,
} SrvComProto_Type_List;

typedef enum
{
    SrvComProto_FrameOut = 1,
    SrvComProto_FrameIn,
} SrvComProto_IOType_List;

typedef enum
{
    MAV_Component_Attitude = 1,
    MAV_Component_Rc_Channel,
    MAV_Component_Raw_IMU,
    MAV_Component_Raw_IMU2,
} SrvComProto_MAVComponent_List;

typedef struct
{
    uint8_t *p_buf;
    uint16_t size;
    uint16_t max_size;
} SrvComProto_Stream_TypeDef;

typedef struct
{
    uint8_t system_id;
    uint8_t component_id;
    uint8_t chan;
} SrvComProto_MavPackInfo_TypeDef;

typedef struct
{
    SrvComProto_IOType_List io_type;
    uint16_t period;

    SrvComProto_MavPackInfo_TypeDef pck_info;
    mavlink_message_t *msg_obj;
    DataPack_Callback pack_callback;
    SYSTEM_RunTime proto_time;

    bool in_proto;
    bool lock_proto;
} SrvComProto_MsgInfo_TypeDef;

typedef struct
{
    uint32_t imu_update_time;
    float gyr_scale;
    float gyr_x;
    float gyr_y;
    float gyr_z;

    float acc_scale;
    float acc_x;
    float acc_y;
    float acc_z;

    uint32_t mag_update_time;
    float mag_scale;
    float mag_x;
    float mag_y;
    float mag_z;

    uint32_t baro_update_time;
    float baro_scale;
    float baro;

    uint32_t sonar_update_time;
    float sonar_scale;
    float sonar_dis;

    uint32_t tof_update_time;
    float tof_scale;
    float tof_dis;

    uint32_t att_update_time;
    float att_roll;
    float att_pitch;
    float att_yaw;

    uint32_t gnss_update_time;
    double lon;
    double lat;
    double alt;

    double vel_n;
    double vel_e;
    double vel_d;

    double forward_vel;
    double lateral_vel;
    double vertical_vel;

    uint16_t utc_year;
    uint16_t utc_month;
    uint16_t utc_day;
    uint16_t utc_hour;
    uint16_t utc_min;
    uint16_t utc_s;
    uint16_t utc_ms;

    uint32_t actuator_update_time;
    uint8_t moto_cnt;
    uint8_t moto_dir[8];
    uint16_t moto[8];

    uint8_t servo_cnt;
    uint8_t servo_dir[8];
    uint16_t servo[8];
} SrvComProto_Data_TypeDef;

typedef struct
{
    bool init_state;
    SrvComProto_Type_List Proto_Type;
    SrvComProto_Data_TypeDef proto_data;
} SrvComProto_Monitor_TypeDef;

typedef struct
{
    void (*init)(SrvComProto_Type_List type, uint8_t *arg);
    // void (*set_decode_callback)();

    void (*fill_imu)(uint32_t update_time, float acc_scale, float gyr_scale, float accx, float accy, float accz, float gyrx, float gyry, float gyrz);
    void (*fill_mag)(uint32_t update_time, float mag_scale, float magx, float magy, float magz);
    void (*fill_baro)(uint32_t update_time, float baro_scale, float bar);
    void (*fill_tof)(uint32_t update_time, float tof_scale, float tof_dis);
    void (*fill_sonar)(uint32_t update_time, float sonar_scale, float sonar_dis);
    void (*fill_attitude)(uint32_t update_time, float roll, float pitch, float yaw);

    bool (*mav_msg_decode)(uint8_t *p_buf, uint32_t len);
    bool (*mav_msg_obj_init)(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_MavPackInfo_TypeDef pck_info,
                             uint32_t period, SrvComProto_IOType_List io_dir,
                             SrvComProto_Stream_TypeDef tar_stream);
    bool (*mav_msg_proto)(SrvComProto_MsgInfo_TypeDef msg,
                          SrvComProto_Stream_TypeDef *com_stream);
} SrvComProto_TypeDef;

extern SrvComProto_TypeDef SrvComProto;

#endif
