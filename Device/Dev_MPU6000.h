#ifndef __DEV_MPU6000_H
#define __DEV_MPU6000_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stddef.f>
#include "imu_data.h"

typedef enum
{
    MPU6000_No_Error = 0,
    MPU6000_Obj_Error,
    MPU6000_DevID_Error,
} DevMPU6000_Error_List;

typedef struct
{
    void (*cs_init)(void);
    void (*bus_init)(void);
    void (*cs_ctl)(bool state);
    bool (*bus_trans)(uint8_t *tx, uint8_t *rx, uint16_t len);

    bool drdy;
    IMUData_TypeDef OriData;

    DevMPU6000_Error_List error;
} DevMPU6000Obj_TypeDef;

typedef struct
{
    bool (*init)(DevMPU6000Obj_TypeDef *sensor_obj);
    bool (*reset)(DevMPU6000Obj_TypeDef *snesor_obj);
    void (*set_drdy)(DevMPU6000Obj_TypeDef *sensor_obj);
    bool (*get_drdy)(DevMPU6000Obj_TypeDef *sensor_obj);
    bool (*sample)(DevMPU6000Obj_TypeDef *sensor_obj);
} DevMPU6000_TypeDef;

#endif
