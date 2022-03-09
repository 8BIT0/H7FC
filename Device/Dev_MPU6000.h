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
    MPU6000_DevID_Error,
} DevMPU6000_Error_List;

typedef struct
{
    void (*cs_init)(void);
    void (*bus_init)(void);
    void (*cs_ctl)(bool state);
    void (*bus_trans)(uint8_t *tx, uint8_t *rx, uint16_t len);
    void (*bus_send)(uint8_t tx);
    void (*bus_receive)(uint8_t rx);

    uint16_t gyro_org[3];
    uint16_t acc_org[3];

    double gyro[3];
    double acc[3];

    uint64_t time_stamp;

    bool drdy;

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
