#ifndef __DEV_FLOW_3901U_H
#define __DEV_FLOW_3901U_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define FLOW_3901U_BAUDRATE 19200
#define FLOW_3901U_BIT      8
#define FLOW_3901U_PARITY   0

#define To_DevFlow3901U_Api(x)  ((DevFlow3901_TypeDef *)x)
#define To_DevFlow3901U_Obj(x)  ((DevFlow3901Obj_TypeDef *)x)
#define DevFlow3901U_Size       sizeof(DevFlow3901Obj_TypeDef)

typedef struct
{
    uint32_t time_stamp;
    int16_t flow_x;
    int16_t flow_y;
    uint8_t quality;
} DevFlow3901UData_TypeDef;

typedef struct
{
    bool init;
    bool yaw_mode_en;
    bool yaw_mode_state;

    uint32_t time_stamp;
    int16_t dis_x;
    int16_t dis_y;
    uint8_t quality;

    uint32_t recv_cnt;
    uint32_t recv_byte;

    uint32_t decode_s;
    uint32_t decode_f;

    bool accessing;

    uint32_t (*get_sys_time)(void);
    void (*yaw_mode_ctl)(bool enable);
} DevFlow3901Obj_TypeDef;

typedef struct
{
    uint8_t (*init)(DevFlow3901Obj_TypeDef *obj);
    void (*recv)(DevFlow3901Obj_TypeDef *obj, uint8_t *p_buf, uint16_t len);
    DevFlow3901UData_TypeDef (*get)(DevFlow3901Obj_TypeDef *obj);
    bool (*yaw_mode)(DevFlow3901Obj_TypeDef *obj);
} DevFlow3901_TypeDef;

extern DevFlow3901_TypeDef DevFlow3901;

#ifdef __cplusplus
}
#endif

#endif
