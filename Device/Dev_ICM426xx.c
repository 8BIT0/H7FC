#include "Dev_ICM426xx.h"

#define ConvertToICM426xxTrip_Reg(trip, odr) ((3 - trip) | (odr & 0x0F))
#define UserBankToReg(x) (x & 7) 

/* internal function */
static bool DevICM426xx_Regs_Read(DevICM426xxObj_TypeDef *sensor_obj, uint32_t addr, uint8_t *tx, uint8_t *rx, uint16_t size);
static bool DevICM426xx_Reg_Write(DevICM426xxObj_TypeDef *sensor_obj, uint8_t addr, uint8_t tx);
static bool DevICM426xx_Reg_Read(DevICM426xxObj_TypeDef *sensor_obj, uint8_t addr, uint8_t *rx);

static bool DevICM426xx_SetUserBank(DevICM426xxObj_TypeDef *sensor_obj, const uint8_t bank);
static bool DevICM426xx_TurnOff_AccGyro(DevICM426xxObj_TypeDef *sensor_obj);
static bool DevICM426xx_TurnOn_AccGyro(DevICM426xxObj_TypeDef *sensor_obj);

/* external function */
static ICM426xx_Sensor_TypeList DevICM426xx_Detect(bus_trans_callback trans, cs_ctl_callback cs_ctl);
static bool DevICM426xx_Config(DevICM426xxObj_TypeDef *Obj, 
                               ICM426xx_SampleRate_List rate, 
                               ICM426xx_GyrTrip_List GyrTrip, 
                               ICM426xx_AccTrip_List AccTrip);
static void DevICM426xx_PreInit(DevICM426xxObj_TypeDef *sensor_obj,
                               cs_ctl_callback cs_ctl,
                               bus_trans_callback bus_trans,
                               delay_callback delay,
                               get_time_stamp_callback get_time_stamp);

/* external variable */
DevICM426xx_TypeDef DevICM426xx = {
    .detect = DevICM426xx_Detect,
    .config = DevICM426xx_Config,
    .pre_init = DevICM426xx_PreInit,
};

static ICM426xx_Sensor_TypeList DevICM426xx_Detect(bus_trans_callback trans, cs_ctl_callback cs_ctl)
{
    uint8_t read_tmp = 0;
    uint8_t write_buff[2] = {0};
    uint8_t read_buff[2] = {0};
    bool state = false;

    if (cs_ctl == NULL || trans == NULL)
        return false;

    write_buff[0] = ICM426XX_WHO_AM_I | ICM426XX_READ_MASK;

    /* CS Low */
    cs_ctl(false);

    state = trans(write_buff, read_buff, 2);

    /* CS High */
    cs_ctl(true);

    if(!state)
        return ICM_NONE;

    switch(read_buff[1])
    {
        case ICM42605_DEV_ID: return ICM42605;
        case ICM42688P_DEV_ID: return ICM42688P;
        default: return ICM_NONE;
    }

    return ICM_NONE;
}

static bool DevICM426xx_Regs_Read(DevICM426xxObj_TypeDef *sensor_obj, uint32_t addr, uint8_t *tx, uint8_t *rx, uint16_t size)
{
    bool state = false;
    uint8_t addr_tmp = addr | ICM426XX_READ_MASK;
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

static bool DevICM426xx_Reg_Read(DevICM426xxObj_TypeDef *sensor_obj, uint8_t addr, uint8_t *rx)
{
    uint8_t Rx_Tmp[2] = {0};
    uint8_t Tx_Tmp[2] = {0};
    bool state = false;

    if (sensor_obj == NULL || sensor_obj->cs_ctl == NULL || sensor_obj->bus_trans == NULL)
        return false;

    Tx_Tmp[0] = addr | ICM426XX_READ_MASK;

    /* cs low */
    sensor_obj->cs_ctl(false);

    state = sensor_obj->bus_trans(Tx_Tmp, Rx_Tmp, 2);

    /* cs high */
    sensor_obj->cs_ctl(true);

    *rx = Rx_Tmp[1];

    return state;
}

static bool DevICM426xx_Reg_Write(DevICM426xxObj_TypeDef *sensor_obj, uint8_t addr, uint8_t tx)
{
    uint8_t Rx_Tmp[2] = {0};
    uint8_t Tx_Tmp[2] = {0};
    bool state = false;

    if (sensor_obj == NULL || sensor_obj->cs_ctl == NULL || sensor_obj->bus_trans == NULL)
        return false;

    Tx_Tmp[0] = addr;
    Tx_Tmp[1] = tx;

    /* cs low */
    sensor_obj->cs_ctl(false);

    state = sensor_obj->bus_trans(Tx_Tmp, Rx_Tmp, 2);

    /* cs high */
    sensor_obj->cs_ctl(true);

    return state;
}

static bool DevICM426xx_Config(DevICM426xxObj_TypeDef *sensor_obj, 
                               ICM426xx_SampleRate_List rate, 
                               ICM426xx_GyrTrip_List GyrTrip, 
                               ICM426xx_AccTrip_List AccTrip)
{
    uint8_t odr_reg_val = 0;

    if(sensor_obj == NULL)
        return false;

    switch ((uint8_t)sensor_obj->rate)
    {
    case ICM426xx_SampleRate_8K:
        odr_reg_val = ICM426xx_ODR_8K;
        break;

    case ICM426xx_SampleRate_4K:
        odr_reg_val = ICM426xx_ODR_4K;
        break;

    case ICM426xx_SampleRate_2K:
        odr_reg_val = ICM426xx_ODR_2K;
        break;

    case ICM426xx_SampleRate_1K:
        odr_reg_val = ICM426xx_ODR_1K;
        break;

    default:
        return false;
    }

    sensor_obj->rate = rate;

    switch ((uint8_t)AccTrip)
    {
    case ICM426xx_Acc_2G:
        sensor_obj->acc_scale = ICM426XX_ACC_2G_SCALE;
        sensor_obj->PHY_AccTrip_Val = 2;
        break;

    case ICM426xx_Acc_4G:
        sensor_obj->acc_scale = ICM426XX_ACC_4G_SCALE;
        sensor_obj->PHY_AccTrip_Val = 4;
        break;

    case ICM426xx_Acc_8G:
        sensor_obj->acc_scale = ICM426XX_ACC_8G_SCALE;
        sensor_obj->PHY_AccTrip_Val = 8;
        break;

    case ICM426xx_Acc_16G:
        sensor_obj->acc_scale = ICM426XX_ACC_16G_SCALE;
        sensor_obj->PHY_AccTrip_Val = 16;
        break;

    default:
        return false;
    }

    switch ((uint8_t)GyrTrip)
    {
    case ICM426xx_Gyr_250DPS:
        sensor_obj->gyr_scale = ICM426XX_GYR_250DPS_SCALE;
        sensor_obj->PHY_GyrTrip_Val = 250;
        break;

    case ICM426xx_Gyr_500DPS:
        sensor_obj->gyr_scale = ICM426XX_GYR_500DPS_SCALE;
        sensor_obj->PHY_GyrTrip_Val = 500;
        break;

    case ICM426xx_Gyr_1000DPS:
        sensor_obj->gyr_scale = ICM426XX_GYR_1000DPS_SCALE;
        sensor_obj->PHY_GyrTrip_Val = 1000;
        break;

    case ICM426xx_Gyr_2000DPS:
        sensor_obj->gyr_scale = ICM426XX_GYR_2000DPS_SCALE;
        sensor_obj->PHY_GyrTrip_Val = 2000;
        break;

    default:
        return false;
    }

    /* set AccTrip and GyrTrip reg val */
    sensor_obj->AccTrip = ConvertToICM426xxTrip_Reg(AccTrip, odr_reg_val);
    sensor_obj->GyrTrip = ConvertToICM426xxTrip_Reg(GyrTrip, odr_reg_val);

    return true;
}

static void DevICM426xx_PreInit(DevICM426xxObj_TypeDef *sensor_obj,
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

static bool DevICM426xx_SetUserBank(DevICM426xxObj_TypeDef *sensor_obj, const uint8_t bank)
{
    bool state = true;
    uint8_t read_out = 0;

    if(sensor_obj == NULL)
        return false;

    state = DevICM426xx_Reg_Write(sensor_obj, ICM426XX_RA_REG_BANK_SEL, UserBankToReg(bank));

    if(!state)
        return false;

    state = DevICM426xx_Reg_Read(sensor_obj, ICM426XX_RA_REG_BANK_SEL, &read_out);
    if((!state) || (read_out != UserBankToReg(bank)))
        return false;

    return true;
}

static bool DevICM426xx_TurnOff_AccGyro(DevICM426xxObj_TypeDef *sensor_obj)
{
    bool state = true;
    uint8_t read_out = 0;

    if(sensor_obj == NULL)
        return false;

    state = DevICM426xx_Reg_Write(sensor_obj, ICM426XX_RA_PWR_MGMT0, ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF);

    if(!state)
        return false;

    state = DevICM426xx_Reg_Read(sensor_obj, ICM426XX_RA_PWR_MGMT0, &read_out);
    if((!state) || (read_out != ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF))
        return false;

    return true;
}

static bool DevICM426xx_TurnOn_AccGyro(DevICM426xxObj_TypeDef *sensor_obj)
{
    bool state = true;
    uint8_t read_out = 0;

    if((sensor_obj == NULL) || (sensor_obj->delay == NULL))
        return false;

    state = DevICM426xx_Reg_Write(sensor_obj, ICM426XX_RA_PWR_MGMT0, ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF | ICM426XX_PWR_MGMT0_ACCEL_MODE_LN | ICM426XX_PWR_MGMT0_GYRO_MODE_LN);
    sensor_obj->delay(1);

    if(!state)
        return false;

    state = DevICM426xx_Reg_Read(sensor_obj, ICM426XX_RA_PWR_MGMT0, &read_out);
    if((!state) || (read_out != ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF | ICM426XX_PWR_MGMT0_ACCEL_MODE_LN | ICM426XX_PWR_MGMT0_GYRO_MODE_LN))
        return false;

    return true;
}

static bool DevICM426xx_Init(DevICM426xxObj_TypeDef *sensor_obj)
{
    uint8_t read_out = 0;

    if((sensor_obj == NULL) || 
       (sensor_obj->cs_ctl == NULL) || 
       (sensor_obj->bus_trans == NULL))
    {
        sensor_obj->error = ICM426xx_Obj_Error;
        return false;
    }

    /* get sensor type first */
    DevICM426xx_Reg_Read(sensor_obj, ICM426XX_WHO_AM_I, &read_out);

    switch (read_out)
    {
        case ICM42605:
            sensor_obj->type = ICM42605;
            break;

        case ICM42688P:
            sensor_obj->type = ICM42688P;
            break; 

        default:
            sensor_obj->type = ICM_NONE;
            sensor_obj->error = ICM426xx_DevID_Error;
            return false;
    }

    DevICM426xx_SetUserBank(sensor_obj, ICM426XX_BANK_SELECT0);

    /* turn acc gyro off for setting */
    if(!DevICM426xx_TurnOff_AccGyro(sensor_obj))
    {
        sensor_obj->error = ICM426xx_AccGyr_TurnOff_Error;
        return false;
    }

    /* config gyro */

    /* config acc */
}




