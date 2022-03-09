#include "Dev_MPU6000.h"

static bool DevMPU6000_Reg_Read(DevMPU6000Obj_TypeDef *sensor_obj, uint8_t addr, uint8_t *rx)
{
    uint8_t addr_tmp = addr;

    if (sensor_obj == NULL || sensor_obj->cs_ctl == NULL || sensor_obj->bus_trans == NULL)
        return false;

    sensor_obj->cs_ctl(true);
    sensor_obj->bus_trans(&addr_tmp, rx, 1);
    sensor_obj->cs_ctl(false);

    return true;
}

static bool DevMPU6000_Reg_Write(DevMPU6000Obj_TypeDef *sensor_obj, uint8_t addr, uint8_t tx)
{
    if (sensor_obj == NULL || sensor_obj->cs_ctl == NULL || sensor_obj->bus_trans == NULL)
        return false;

    return true;
}

static bool DevMPU6000_Regs_Read(DevMPU6000Obj_TypeDef *sensor_obj, uint8_t addr, uint8_t *tx, uint8_t *rx)
{
    if (sensor_obj == NULL || sensor_obj->cs_ctl == NULL || sensor_obj->bus_trans == NULL)
        return false;

    return true;
}

static bool DevMPU6000_Init(DevMPU6000Obj_TypeDef *sensor_obj)
{
    if (sensor_obj == NULL)
        return false;

    if ((sensor_obj->bus_init == NULL) ||
        (sensor_obj->bus_trans == NULL) ||
        (sensor_obj->cs_ctl == NULL) ||
        (sensor_obj->cs_init == NULL))
    {
        sensor_obj->error = MPU6000_Obj_Error;
        return false;
    }

    sensor_obj->cs_init();
    sensor_obj->bus_init();

    return true;
}
