#include "Dev_ICM20602.h"

/* internal function */

/* external function */

static bool DevICM20602_Regs_Read(DevICM20602Obj_TypeDef *Obj, uint32_t addr, uint8_t *tx, uint8_t *rx, uint16_t size)
{
    bool state = false;

    if (Obj == NULL || Obj->cs_ctl == NULL || Obj->bus_trans == NULL)
        return false;

    tx[0] = addr | ICM20602_WRITE_MASK;

    /* CS Low */
    Obj->cs_ctl(false);

    state = Obj->bus_trans(tx, rx, size);

    /* CS High */
    Obj->cs_ctl(true);

    return state;
}

static bool DevICM20602_Reg_Read(DevICM20602Obj_TypeDef *Obj, uint8_t addr, uint8_t *rx)
{
    uint8_t Rx_Tmp[2] = {0};
    uint8_t Tx_Tmp[2] = {0};
    bool state = false;

    if (Obj == NULL || Obj->cs_ctl == NULL || Obj->bus_trans == NULL)
        return false;

    Tx_Tmp[0] = addr | ICM20602_WRITE_MASK;

    /* cs low */
    Obj->cs_ctl(false);

    state = Obj->bus_trans(Tx_Tmp, Rx_Tmp, 2);

    *rx = Rx_Tmp[1];

    /* cs high */
    Obj->cs_ctl(true);

    return state;
}

static bool DevICM20602_Reg_Write(DevICM20602Obj_TypeDef *Obj, uint8_t addr, uint8_t tx)
{
    uint8_t Rx_Tmp[2] = {0};
    uint8_t Tx_Tmp[2] = {0};
    bool state = false;

    if (Obj == NULL || Obj->cs_ctl == NULL || Obj->bus_trans == NULL)
        return false;

    Tx_Tmp[0] = addr;
    Tx_Tmp[1] = tx;

    /* cs low */
    Obj->cs_ctl(false);

    state = Obj->bus_trans(Tx_Tmp, Rx_Tmp, 2);

    /* cs high */
    Obj->cs_ctl(true);

    return state;
}

static void DevICM20602_Config(DevICM20602Obj_TypeDef *Obj, ICM20602_SampleRate_List rate, ICM20602_GyrTrip_List GyrTrip, ICM20602_AccTrip_List AccTrip)
{
    switch ((uint8_t)Obj->rate)
    {
    case ICM20602_SampleRate_8K:
    case ICM20602_SampleRate_4K:
    case ICM20602_SampleRate_2K:
    case ICM20602_SampleRate_1K:
        break;

    default:
        return false;
    }

    Obj->rate = rate;

    switch ((uint8_t)AccTrip)
    {
    case ICM20602_Acc_2G:
        Obj->acc_scale = ICM20602_ACC_2G_SCALE;
        break;

    case ICM20602_Acc_4G:
        Obj->acc_scale = ICM20602_ACC_4G_SCALE;
        break;

    case ICM20602_Acc_8G:
        Obj->acc_scale = ICM20602_ACC_8G_SCALE;
        break;

    case ICM20602_Acc_16G:
        Obj->acc_scale = ICM20602_ACC_16G_SCALE;
        break;

    default:
        return false;
    }

    switch ((uint8_t)GyrTrip)
    {
    case ICM20602_Gyr_250DPS:
        Obj->gyr_scale = ICM20602_GYR_250DPS_SCALE;
        break;

    case ICM20602_Gyr_500DPS:
        Obj->gyr_scale = ICM20602_GYR_500DPS_SCALE;
        break;

    case ICM20602_Gyr_1000DPS:
        Obj->gyr_scale = ICM20602_GYR_1000DPS_SCALE;
        break;

    case ICM20602_Gyr_2000DPS:
        Obj->gyr_scale = ICM20602_GYR_2000DPS_SCALE;
        break;

    default:
        return false;
    }

    return true;
}



static void DevICM20602_PreInit(DevICM20602Obj_TypeDef *Obj,
                                cs_ctl_callback cs_ctl,
                                bus_trans_callback bus_trans,
                                delay_callback delay,
                                get_time_stamp_callback get_time_stamp)
{
    Obj->cs_ctl = cs_ctl;
    Obj->bus_trans = bus_trans;
    Obj->delay = delay;
    Obj->get_timestamp = get_time_stamp;
}

static ICM20602_Error_List DevICM20602_Init(DevICM20602Obj_TypeDef *Obj)
{
    uint8_t read_out = 0;

    DevICM20602_Reg_Read(Obj, ICM20602_WHO_AM_I, &read_out);
    Obj->delay(10);

    switch(read_out)
    {
        case ICM20602_DEV_V1_ID:
        case ICM20602_DEV_V2_ID:
        break;

        default:
            Obj->error = ICM20602_DevID_Error;
            return false;
    }

    // icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1, 0x01);     //时钟设置
    // icm_spi_w_reg_byte(ICM20602_PWR_MGMT_2, 0x00);     //开启陀螺仪和加速度计
    // icm_spi_w_reg_byte(ICM20602_CONFIG, 0x01);         // 176HZ 1KHZ
    // icm_spi_w_reg_byte(ICM20602_SMPLRT_DIV, 0x07);     //采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    // icm_spi_w_reg_byte(ICM20602_GYRO_CONFIG, 0x18);    //±2000 dps
    // icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG, 0x10);   //±8g
    // icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG_2, 0x03); // Average 4 samples   44.8HZ   //0x23 Average 16 samples

    return ICM20602_No_Error;
}

static void DevICM20602_SetDRDY(DevICM20602Obj_TypeDef *Obj)
{
    Obj->drdy = true;
}

static bool DevICM20602_SwReset(DevICM20602Obj_TypeDef *Obj)
{
}

static bool DevICM20602_GetReady(DevICM20602Obj_TypeDef *Obj)
{
    return Obj->drdy;
}

static void DevICM20602_Sample(DevICM20602Obj_TypeDef *Obj)
{
}

static IMUData_TypeDef DevICM20602_Get_Data(DevICM20602Obj_TypeDef *Obj)
{
}

static ICM20602_Error_List DevICM20602_Get_InitError(DevICM20602Obj_TypeDef *Obj)
{
    return Obj->error;
}
