#include "Dev_MPU6000.h"

static bool DevMPU6000_Reg_Read(DevMPU6000Obj_TypeDef *sensor_obj, uint8_t addr, uint8_t *rx)
{
    uint8_t write_buff[2] = {0};
    uint8_t read_buff[2] = {0};
    bool state = false;

    if (sensor_obj == NULL || sensor_obj->cs_ctl == NULL || sensor_obj->bus_trans == NULL)
        return false;

    write_buff[0] = addr;

    sensor_obj->cs_ctl(true);
    state = sensor_obj->bus_trans(write_buff, read_buff, 2);
    sensor_obj->cs_ctl(false);

    *rx = read_buff[1];

    return state;
}

static bool DevMPU6000_Reg_Write(DevMPU6000Obj_TypeDef *sensor_obj, uint8_t addr, uint8_t tx)
{
    uint8_t write_buff[2] = {0};
    uint8_t read_buff[2] = {0};
    bool state = false;

    if (sensor_obj == NULL || sensor_obj->cs_ctl == NULL || sensor_obj->bus_trans == NULL)
        return false;

    write_buff[0] = addr;
    write_buff[1] = tx;

    sensor_obj->cs_ctl(true);
    state = sensor_obj->bus_trans(write_buff, NULL, 2);
    sensor_obj->cs_ctl(false);

    return state;
}

static bool DevMPU6000_Regs_Read(DevMPU6000Obj_TypeDef *sensor_obj, uint8_t addr, uint8_t *tx, uint8_t *rx, uint16_t size)
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

static bool DevMPU6000_Reset(DevMPU6000Obj_TypeDef *sensor_obj)
{
    bool state = false;

    if (sensor_obj == NULL)
        return false;

    return state;
}

static void DevMPU6000_SetDRDY(DevMPU6000Obj_TypeDef *sensor_obj)
{
    sensor_obj->drdy = true;
}
