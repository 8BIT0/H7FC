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
    uint8_t read_out = 0;

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

    /* reset power manage register first */
    DevMPU6000_Reg_Write(sensor_obj, MPU6000_PWR_MGMT_1, BIT_H_RESET);
    sensor_obj->delay(100);

    DevMPU6000_Reg_Read(sensor_obj, MPU6000_WHOAMI, &read_out);
    sensor_obj->delay(100);

    switch (read_out)
    {
    case MPU6000ES_REV_C4:
    case MPU6000ES_REV_C5:
    case MPU6000ES_REV_D6:
    case MPU6000ES_REV_D7:
    case MPU6000ES_REV_D8:
    case MPU6000_REV_C4:
    case MPU6000_REV_C5:
    case MPU6000_REV_D6:
    case MPU6000_REV_D7:
    case MPU6000_REV_D8:
    case MPU6000_REV_D9:
    case MPU6000_REV_D10:
        break;

    default:
        sensor_obj->error = MPU6000_DevID_Error;
        return false;
    }

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

static void DevMPU6000_Sample(DevMPU6000Obj_TypeDef *sensor_obj)
{
    if (sensor_obj->drdy)
    {

        sensor_obj->update = true;
        sensor_obj->drdy = false;
    }
}

IMUData_TypeDef DevMPU6000_Get_Data(DevMPU6000Obj_TypeDef *sensor_obj)
{
    if (sensor_obj->update)
    {
        sensor_obj->update = false;
        return sensor_obj->OriData;
    }
    else
        return sensor_obj->OriData_Lst;
}
