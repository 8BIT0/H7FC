#include "Dev_MPU6000.h"

/* internal function */
static bool DevMPU6000_Reg_Read(DevMPU6000Obj_TypeDef *sensor_obj, uint8_t addr, uint8_t *rx);
static bool DevMPU6000_Reg_Write(DevMPU6000Obj_TypeDef *sensor_obj, uint8_t addr, uint8_t tx);

/* external function */
static bool DevMPU6000_Detect(bus_trans_callback trans, cs_ctl_callback cs_ctl);
static void DevMPU6000_PreInit(DevMPU6000Obj_TypeDef *sensor_obj,
                               cs_ctl_callback cs_ctl,
                               bus_trans_callback bus_trans,
                               delay_callback delay,
                               get_time_stamp_callback ge_time_stamp);

static bool DevMPU6000_Config(DevMPU6000Obj_TypeDef *sensor_obj,
                              DevMPU6000_SampleRate_List rate,
                              DevMPU6000_AccTrip_List AccTrip,
                              DevMPU6000_GyrTrip_List GyrTrip);

static bool DevMPU6000_Init(DevMPU6000Obj_TypeDef *sensor_obj);
static void DevMPU6000_SetDRDY(DevMPU6000Obj_TypeDef *sensor_obj);
static bool DevMPU6000_GetReady(DevMPU6000Obj_TypeDef *sensor_obj);
static bool DevMPU6000_SwReset(DevMPU6000Obj_TypeDef *sensor_obj);
static bool DevMPU6000_Sample(DevMPU6000Obj_TypeDef *sensor_obj);
static IMUData_TypeDef DevMPU6000_Get_Data(DevMPU6000Obj_TypeDef *sensor_obj);
static DevMPU6000_Error_List DevMPU6000_Get_InitError(DevMPU6000Obj_TypeDef *sensor_obj);
static IMUModuleScale_TypeDef DevMPU6000_Get_Scale(const DevMPU6000Obj_TypeDef *sensor_obj);
static float DevMPU6000_Get_Specified_AngularSpeed_Diff(const DevMPU6000Obj_TypeDef *sensor_obj);

/* external MPU6000 Object */
DevMPU6000_TypeDef DevMPU6000 = {
    .detect = DevMPU6000_Detect,
    .pre_init = DevMPU6000_PreInit,
    .init = DevMPU6000_Init,
    .config = DevMPU6000_Config,
    .reset = DevMPU6000_SwReset,
    .set_ready = DevMPU6000_SetDRDY,
    .get_ready = DevMPU6000_GetReady,
    .sample = DevMPU6000_Sample,
    .get_data = DevMPU6000_Get_Data,
    .get_error = DevMPU6000_Get_InitError,
    .get_scale = DevMPU6000_Get_Scale,
    .get_gyr_angular_speed_diff = DevMPU6000_Get_Specified_AngularSpeed_Diff,
};

static bool DevMPU6000_Detect(bus_trans_callback trans, cs_ctl_callback cs_ctl)
{
    uint8_t read_tmp = 0;
    uint8_t write_buff[2] = {0};
    uint8_t read_buff[2] = {0};
    bool state = false;

    if (cs_ctl == NULL || trans == NULL)
        return false;

    write_buff[0] = MPU6000_WHOAMI | MPU6000_WRITE_MASK;

    /* CS Low */
    cs_ctl(false);

    state = trans(write_buff, read_buff, 2);

    /* CS High */
    cs_ctl(true);

    if(!state)
        return false;

    if(read_buff[1] == MPU6000_DEV_ID)
        return true;

    return false;
}

static bool Dev_MPU6000_Regs_Read(DevMPU6000Obj_TypeDef *sensor_obj, uint8_t addr, uint8_t *tx, uint8_t *rx, uint8_t size)
{
    bool state = false;
    uint8_t addr_tmp = addr | MPU6000_WRITE_MASK;
    uint8_t read_tmp = 0;

    if (sensor_obj == NULL || sensor_obj->cs_ctl == NULL || sensor_obj->bus_trans == NULL)
        return false;

    /* CS Low */
    sensor_obj->cs_ctl(false);

    state = sensor_obj->bus_trans(&addr_tmp, &read_tmp, 1);
    state = sensor_obj->bus_trans(tx, rx, size);

    /* CS High */
    sensor_obj->cs_ctl(true);

    return state;
}

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

static bool DevMPU6000_Config(DevMPU6000Obj_TypeDef *sensor_obj, DevMPU6000_SampleRate_List rate, DevMPU6000_AccTrip_List AccTrip, DevMPU6000_GyrTrip_List GyrTrip)
{
    switch ((uint8_t)sensor_obj->rate)
    {
    case MPU6000_SampleRate_8K:
    case MPU6000_SampleRate_4K:
    case MPU6000_SampleRate_2K:
    case MPU6000_SampleRate_1K:
        break;

    default:
        return false;
    }

    sensor_obj->rate = rate;

    switch ((uint8_t)AccTrip)
    {
    case MPU6000_Acc_2G:
        sensor_obj->acc_scale = MPU6000_ACC_2G_SCALE;
        sensor_obj->PHY_AccTrip_Val = 2;
        break;

    case MPU6000_Acc_4G:
        sensor_obj->acc_scale = MPU6000_ACC_4G_SCALE;
        sensor_obj->PHY_AccTrip_Val = 4;
        break;

    case MPU6000_Acc_8G:
        sensor_obj->acc_scale = MPU6000_ACC_8G_SCALE;
        sensor_obj->PHY_AccTrip_Val = 8;
        break;

    case MPU6000_Acc_16G:
        sensor_obj->acc_scale = MPU6000_ACC_16G_SCALE;
        sensor_obj->PHY_AccTrip_Val = 16;
        break;

    default:
        return false;
    }

    switch ((uint8_t)GyrTrip)
    {
    case MPU6000_Gyr_250DPS:
        sensor_obj->gyr_scale = MPU6000_GYR_250DPS_SCALE;
        sensor_obj->PHY_GyrTrip_Val = 250;
        break;

    case MPU6000_Gyr_500DPS:
        sensor_obj->gyr_scale = MPU6000_GYR_500DPS_SCALE;
        sensor_obj->PHY_GyrTrip_Val = 500;
        break;

    case MPU6000_Gyr_1000DPS:
        sensor_obj->gyr_scale = MPU6000_GYR_1000DPS_SCALE;
        sensor_obj->PHY_GyrTrip_Val = 1000;
        break;

    case MPU6000_Gyr_2000DPS:
        sensor_obj->gyr_scale = MPU6000_GYR_2000DPS_SCALE;
        sensor_obj->PHY_GyrTrip_Val = 2000;
        break;

    default:
        return false;
    }

    sensor_obj->AccTrip = ConvertToTrip_Reg(AccTrip);
    sensor_obj->GyrTrip = ConvertToTrip_Reg(GyrTrip);

    return true;
}

static void DevMPU6000_PreInit(DevMPU6000Obj_TypeDef *sensor_obj,
                               cs_ctl_callback cs_ctl,
                               bus_trans_callback bus_trans,
                               delay_callback delay,
                               get_time_stamp_callback get_time_stamp)
{
    sensor_obj->cs_ctl = cs_ctl;
    sensor_obj->bus_trans = bus_trans;
    sensor_obj->delay = delay;
    sensor_obj->get_timestamp = get_time_stamp;
}

static bool DevMPU6000_Init(DevMPU6000Obj_TypeDef *sensor_obj)
{
    uint8_t read_out = 0;

    if ((sensor_obj == NULL) ||
        (sensor_obj->bus_trans == NULL) ||
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

    if (read_out != MPU6000_DEV_ID)
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

    if (!DevMPU6000_Reg_Write(sensor_obj, MPU6000_SMPLRT_DIV, sensor_obj->rate))
    {
        sensor_obj->error = MPU6000_DIV_Set_Error;
        return false;
    }
    sensor_obj->delay(15);

    /* set gyro range 2000dps */
    if (!DevMPU6000_Reg_Write(sensor_obj, MPU6000_GYRO_CONFIG, sensor_obj->GyrTrip))
    {
        sensor_obj->error = MPU6000_Gyr_Cfg_Error;
        return false;
    }
    sensor_obj->delay(15);

    /* set acc range 16G */
    if (!DevMPU6000_Reg_Write(sensor_obj, MPU6000_ACCEL_CONFIG, sensor_obj->AccTrip))
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
    uint8_t Tx[14] = {0};
    uint8_t Rx[14] = {0};

    if ((sensor_obj->error == MPU6000_No_Error) && (sensor_obj->drdy))
    {
        sensor_obj->OriData.time_stamp = sensor_obj->get_timestamp();

        Dev_MPU6000_Regs_Read(sensor_obj, MPU6000_ACCEL_XOUT_H, Tx, Rx, 14);

        for (uint8_t axis = Axis_X; axis < Axis_Sum; axis++)
        {
            sensor_obj->OriData.acc_int[axis] = (int16_t)((Rx[axis * 2] << 8) | Rx[axis * 2 + 1]);
            sensor_obj->OriData.gyr_int[axis] = (int16_t)((Rx[axis * 2 + 8] << 8) | Rx[axis * 2 + 9]);

            /* convert int data to float */
            sensor_obj->OriData.acc_flt[axis] = ((float)sensor_obj->OriData.acc_int[axis] / sensor_obj->acc_scale);
            sensor_obj->OriData.gyr_flt[axis] = ((float)sensor_obj->OriData.gyr_int[axis] / sensor_obj->gyr_scale);
        }

        sensor_obj->OriData.time_stamp = sensor_obj->get_timestamp();
        sensor_obj->drdy = false;
        return true;
    }

    return false;
}

static IMUData_TypeDef DevMPU6000_Get_Data(DevMPU6000Obj_TypeDef *sensor_obj)
{
    IMUData_TypeDef tmp;

    memset(&tmp, NULL, sizeof(tmp));
    if (sensor_obj->error == MPU6000_No_Error)
        return sensor_obj->OriData;

    return tmp;
}

static IMUModuleScale_TypeDef DevMPU6000_Get_Scale(const DevMPU6000Obj_TypeDef *sensor_obj)
{
    IMUModuleScale_TypeDef scale_tmp;

    scale_tmp.acc_scale = sensor_obj->acc_scale;
    scale_tmp.gyr_scale = sensor_obj->gyr_scale;

    return scale_tmp;
}

/* specified anguler speed per millsecond */
static float DevMPU6000_Get_Specified_AngularSpeed_Diff(const DevMPU6000Obj_TypeDef *sensor_obj)
{
    return sensor_obj->PHY_GyrTrip_Val / 1000.0f;
}