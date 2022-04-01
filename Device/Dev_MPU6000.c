#include "Dev_MPU6000.h"

/* internal function */
static bool DevMPU6000_Reg_Read(DevMPU6000Obj_TypeDef *sensor_obj, uint8_t addr, uint8_t *rx);
static bool DevMPU6000_Reg_Write(DevMPU6000Obj_TypeDef *sensor_obj, uint8_t addr, uint8_t tx);

/* external function */
static void DevMPU6000_PreInit(DevMPU6000Obj_TypeDef *sensor_obj,
                               cs_ctl_callback cs_ctl,
                               bus_trans_callback bus_trans,
                               delay_callback delay);
static bool DevMPU6000_Init(DevMPU6000Obj_TypeDef *sensor_obj);
static void DevMPU6000_SetDRDY(DevMPU6000Obj_TypeDef *sensor_obj);
static bool DevMPU6000_GetReady(DevMPU6000Obj_TypeDef *sensor_obj);
static bool DevMPU6000_SwReset(DevMPU6000Obj_TypeDef *sensor_obj);
static bool DevMPU6000_Sample(DevMPU6000Obj_TypeDef *sensor_obj);
IMUData_TypeDef DevMPU6000_Get_Data(DevMPU6000Obj_TypeDef *sensor_obj);
static DevMPU6000_Error_List DevMPU6000_Get_InitError(DevMPU6000Obj_TypeDef *sensor_obj);

/* external MPU6000 Object */
DevMPU6000_TypeDef DevMPU6000 = {
    .pre_init = DevMPU6000_PreInit,
    .init = DevMPU6000_Init,
    .reset = DevMPU6000_SwReset,
    .set_drdy = DevMPU6000_SetDRDY,
    .get_drdy = DevMPU6000_GetReady,
    .sample = DevMPU6000_Sample,
    .get_data = DevMPU6000_Get_Data,
    .get_error = DevMPU6000_Get_InitError,
};

static bool DevMPU6000_Reg_Read(DevMPU6000Obj_TypeDef *sensor_obj, uint8_t addr, uint8_t *rx)
{
    uint8_t write_buff[2] = {0};
    uint8_t read_buff[2] = {0};
    bool state = false;

    if (sensor_obj == NULL || sensor_obj->cs_ctl == NULL || sensor_obj->bus_trans == NULL)
        return false;

    write_buff[0] = addr | MPU6000_WRITE_MASK;

    /* CS Low */
    sensor_obj->cs_ctl(false);

    state = sensor_obj->bus_trans(write_buff, read_buff, 2);

    /* CS High */
    sensor_obj->cs_ctl(true);

    *rx = read_buff[1];

    return state;
}

/* MAX Byte I&O Speed 20k */
static bool DevMPU6000_Reg_Write(DevMPU6000Obj_TypeDef *sensor_obj, uint8_t addr, uint8_t tx)
{
    uint8_t write_buff[2] = {0};
    uint8_t read_buff[2] = {0};
    bool state = false;

    if (sensor_obj == NULL || sensor_obj->cs_ctl == NULL || sensor_obj->bus_trans == NULL)
        return false;

    write_buff[0] = addr;
    write_buff[1] = tx;

    /* CS Low */
    sensor_obj->cs_ctl(false);

    state = sensor_obj->bus_trans(write_buff, read_buff, 2);

    /* CS High */
    sensor_obj->cs_ctl(true);

    return state;
}

static void DevMPU6000_PreInit(DevMPU6000Obj_TypeDef *sensor_obj,
                               cs_ctl_callback cs_ctl,
                               bus_trans_callback bus_trans,
                               delay_callback delay)
{
    sensor_obj->cs_ctl = cs_ctl;
    sensor_obj->bus_trans = bus_trans;
    sensor_obj->delay = delay;
}

static bool DevMPU6000_Init(DevMPU6000Obj_TypeDef *sensor_obj)
{
    uint8_t read_out = 0;

    if (sensor_obj == NULL)
        return false;

    if ((sensor_obj->bus_trans == NULL) ||
        (sensor_obj->cs_ctl == NULL))
    {
        sensor_obj->error = MPU6000_Obj_Error;
        return false;
    }

    /* reset power manage register first */
    if (!DevMPU6000_Reg_Write(sensor_obj, MPU6000_PWR_MGMT_1, BIT_H_RESET))
    {
        sensor_obj->error = MPU6000_BusCommunicate_Error;
        return false;
    }
    sensor_obj->delay(15);

    if (!DevMPU6000_Reg_Read(sensor_obj, MPU6000_WHOAMI, &read_out))
    {
        sensor_obj->error = MPU6000_BusCommunicate_Error;
        return false;
    }

    switch (read_out != MPU6000_DEV_ID)
    {
        sensor_obj->error = MPU6000_DevID_Error;
        sensor_obj->delay(15);
        return false;
    }

    if (!DevMPU6000_Reg_Write(sensor_obj, MPU6000_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP))
    {
        sensor_obj->error = MPU6000_SignalPathReset_Error;
        return false;
    }
    sensor_obj->delay(15);

    /* Clock Source PPL with Z axis gyro reference */
    if (!DevMPU6000_Reg_Write(sensor_obj, MPU6000_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ))
    {
        sensor_obj->error = MPU6000_SignalPathReset_Error;
        return false;
    }
    sensor_obj->delay(15);

    /* Disable Primary I2C Interface */
    if (!DevMPU6000_Reg_Write(sensor_obj, MPU6000_USER_CTRL, BIT_I2C_IF_DIS))
    {
        sensor_obj->error = MPU6000_DisableI2C_Error;
        return false;
    }
    sensor_obj->delay(15);

    if (!DevMPU6000_Reg_Write(sensor_obj, MPU6000_PWR_MGMT_2, 0x00))
    {
        sensor_obj->error = MPU6000_PWRMNG2_Set_Error;
        return false;
    }
    sensor_obj->delay(15);

    if (!DevMPU6000_Reg_Write(sensor_obj, MPU6000_SMPLRT_DIV, 0x00))
    {
        sensor_obj->error = MPU6000_DIV_Set_Error;
        return false;
    }
    sensor_obj->delay(15);

    /* set gyro range 2000dps */
    if (!DevMPU6000_Reg_Write(sensor_obj, MPU6000_GYRO_CONFIG, BITS_FS_2000DPS))
    {
        sensor_obj->error = MPU6000_Gyr_Cfg_Error;
        return false;
    }
    sensor_obj->delay(15);

    /* set acc range 16G */
    if (!DevMPU6000_Reg_Write(sensor_obj, MPU6000_ACCEL_CONFIG, BITS_FS_16G))
    {
        sensor_obj->error = MPU6000_Acc_Cfg_Error;
        return false;
    }
    sensor_obj->delay(15);

    /* interrupt pin config */
    if (!DevMPU6000_Reg_Write(sensor_obj, MPU6000_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0))
    {
        sensor_obj->error = MPU6000_IntPin_Set_Error;
        return false;
    }
    sensor_obj->delay(15);

    /* enable interrupt */
    if (!DevMPU6000_Reg_Write(sensor_obj, MPU6000_INT_ENABLE, MPU_RF_DATA_RDY_EN))
    {
        sensor_obj->error = MPU6000_EnableInt_Error;
        return false;
    }
    sensor_obj->delay(15);

    sensor_obj->error = MPU6000_No_Error;
    return true;
}

static DevMPU6000_Error_List DevMPU6000_Get_InitError(DevMPU6000Obj_TypeDef *sensor_obj)
{
    return sensor_obj->error;
}

static void DevMPU6000_SetDRDY(DevMPU6000Obj_TypeDef *sensor_obj)
{
    sensor_obj->drdy = true;
}

static bool DevMPU6000_GetReady(DevMPU6000Obj_TypeDef *sensor_obj)
{
    return sensor_obj->drdy;
}

/* half develop */
static bool DevMPU6000_SwReset(DevMPU6000Obj_TypeDef *sensor_obj)
{
    bool state = false;

    if (sensor_obj == NULL)
        return false;

    return state;
}

static bool DevMPU6000_Sample(DevMPU6000Obj_TypeDef *sensor_obj)
{
    if ((sensor_obj->error == MPU6000_No_Error) && (sensor_obj->drdy))
    {

        sensor_obj->update = true;
        sensor_obj->drdy = false;

        return true;
    }

    return false;
}

IMUData_TypeDef DevMPU6000_Get_Data(DevMPU6000Obj_TypeDef *sensor_obj)
{
    if ((sensor_obj->error == MPU6000_No_Error) && sensor_obj->update)
    {
        sensor_obj->update = false;
        sensor_obj->OriData_Lst = sensor_obj->OriData;
        return sensor_obj->OriData;
    }
    else
        return sensor_obj->OriData_Lst;
}
