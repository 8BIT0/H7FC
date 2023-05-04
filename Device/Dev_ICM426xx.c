#include "Dev_ICM426xx.h"

#define ConvertToICM426xxTrip_Reg(trip, odr) ((3 - trip) << 5 | (odr & 0x0F))
#define UserBankToReg(x) (x & 7) 

// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42688P
// see table in section 5.3
static ICM426xx_AAF_Config_TypeDef aafLUT42688[ICM426xx_AAF_Sum] = {
    [ICM426xx_AAF_258Hz]  = {  6,   36, 10 },
    [ICM426xx_AAF_536Hz]  = { 12,  144,  8 },
    [ICM426xx_AAF_997Hz]  = { 21,  440,  6 },
    [ICM426xx_AAF_1962Hz] = { 37, 1376,  4 },
};

// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42605
// actual cutoff differs slightly from those of the 42605
  // see table in section 5.3
static ICM426xx_AAF_Config_TypeDef aafLUT42605[ICM426xx_AAF_Sum] = {
    [ICM426xx_AAF_258Hz]  = { 21,  440,  6 }, // actually 249 Hz
    [ICM426xx_AAF_536Hz]  = { 39, 1536,  4 }, // actually 524 Hz
    [ICM426xx_AAF_997Hz]  = { 63, 3968,  3 }, // actually 995 Hz
    [ICM426xx_AAF_1962Hz] = { 63, 3968,  3 }, // 995 Hz is the max cutoff on the 42605
};

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
static bool DevICM426xx_Init(DevICM426xxObj_TypeDef *sensor_obj);
static void DevICM426xx_SetDRDY(DevICM426xxObj_TypeDef *sensor_obj);
static bool DevICM426xx_GetDRDY(DevICM426xxObj_TypeDef *sensor_obj);
static IMU_Error_TypeDef DevICM426xx_GetError(DevICM426xxObj_TypeDef *sensor_obj);
static bool DevICM426xx_Sample(DevICM426xxObj_TypeDef *sensor_obj);
static IMUData_TypeDef DevICM426xx_Get_Data(DevICM426xxObj_TypeDef *sensor_obj);
static IMUModuleScale_TypeDef DevICM426xx_Get_Scale(const DevICM426xxObj_TypeDef *sensor_obj);
static float DevICM426xx_Get_Specified_AngularSpeed_Diff(const DevICM426xxObj_TypeDef *sensor_obj);

/* external variable */
DevICM426xx_TypeDef DevICM426xx = {
    .detect = DevICM426xx_Detect,
    .config = DevICM426xx_Config,
    .init = DevICM426xx_Init,
    .pre_init = DevICM426xx_PreInit,
    .set_ready = DevICM426xx_SetDRDY,
    .get_ready = DevICM426xx_GetDRDY,
    .get_error = DevICM426xx_GetError,
    .get_data = DevICM426xx_Get_Data,
    .get_gyr_angular_speed_diff = DevICM426xx_Get_Specified_AngularSpeed_Diff,
    .get_scale = DevICM426xx_Get_Scale,
    .sample = DevICM426xx_Sample,
};

static ICM426xx_Sensor_TypeList DevICM426xx_Detect(bus_trans_callback trans, cs_ctl_callback cs_ctl)
{
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
    if((!state) || (read_out != (ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF | ICM426XX_PWR_MGMT0_ACCEL_MODE_LN | ICM426XX_PWR_MGMT0_GYRO_MODE_LN)))
        return false;

    return true;
}

static bool DevICM426xx_Init(DevICM426xxObj_TypeDef *sensor_obj)
{
    uint8_t read_out = 0;
    uint8_t intConfig1Value = 0;
    ICM426xx_AAF_Config_TypeDef *aaf_tab = NULL;

    if((sensor_obj == NULL) || 
       (sensor_obj->cs_ctl == NULL) || 
       (sensor_obj->bus_trans == NULL))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Obj_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    /* get sensor type first */
    if(!DevICM426xx_Reg_Read(sensor_obj, ICM426XX_WHO_AM_I, &read_out))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    switch (read_out)
    {
        case ICM42605_DEV_ID:
            sensor_obj->type = ICM42605;
            aaf_tab = aafLUT42605;
            break;

        case ICM42688P_DEV_ID:
            sensor_obj->type = ICM42688P;
            aaf_tab = aafLUT42688;
            break; 

        default:
            sensor_obj->type = ICM_NONE;
            sensor_obj->error.code = ICM426xx_DevID_Error;
            return false;
    }

    if(!DevICM426xx_SetUserBank(sensor_obj, ICM426XX_BANK_SELECT0))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    /* turn acc gyro off for setting */
    if(!DevICM426xx_TurnOff_AccGyro(sensor_obj))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_AccGyr_TurnOff_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    /* config gyro */
    if(!DevICM426xx_SetUserBank(sensor_obj, ICM426XX_BANK_SELECT1))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    if(!DevICM426xx_Reg_Write(sensor_obj, ICM426XX_RA_GYRO_CONFIG_STATIC3, aaf_tab[ICM426xx_AAF_258Hz].delt) ||
       !DevICM426xx_Reg_Read(sensor_obj, ICM426XX_RA_GYRO_CONFIG_STATIC3, &read_out))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    if(read_out != aaf_tab[ICM426xx_AAF_258Hz].delt)
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_GyrAAF_Error, __FUNCTION__, __LINE__, ICM426XX_RA_GYRO_CONFIG_STATIC3, read_out, aaf_tab[ICM426xx_AAF_258Hz].delt);
        return false;
    }

    if(!DevICM426xx_Reg_Write(sensor_obj, ICM426XX_RA_GYRO_CONFIG_STATIC4, aaf_tab[ICM426xx_AAF_258Hz].deltSqr & 0xFF) ||
       !DevICM426xx_Reg_Read(sensor_obj, ICM426XX_RA_GYRO_CONFIG_STATIC4, &read_out))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    if(read_out != (aaf_tab[ICM426xx_AAF_258Hz].deltSqr & 0xFF))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_GyrAAF_Error, __FUNCTION__, __LINE__, ICM426XX_RA_GYRO_CONFIG_STATIC4, read_out, 
                         aaf_tab[ICM426xx_AAF_258Hz].deltSqr & 0xFF);
        return false;
    }

    if(!DevICM426xx_Reg_Write(sensor_obj, ICM426XX_RA_GYRO_CONFIG_STATIC5, (aaf_tab[ICM426xx_AAF_258Hz].deltSqr >> 8) | (aaf_tab[ICM426xx_AAF_258Hz].bitshift << 4)) ||
       !DevICM426xx_Reg_Read(sensor_obj, ICM426XX_RA_GYRO_CONFIG_STATIC5, &read_out))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    if(read_out != ((aaf_tab[ICM426xx_AAF_258Hz].deltSqr >> 8) | (aaf_tab[ICM426xx_AAF_258Hz].bitshift << 4)))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_GyrAAF_Error, __FUNCTION__, __LINE__, ICM426XX_RA_GYRO_CONFIG_STATIC5, read_out, 
                         (aaf_tab[ICM426xx_AAF_258Hz].deltSqr >> 8) | (aaf_tab[ICM426xx_AAF_258Hz].bitshift << 4));
        return false;
    }

    /* config acc */
    if(!DevICM426xx_SetUserBank(sensor_obj, ICM426XX_BANK_SELECT2))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    if(!DevICM426xx_Reg_Write(sensor_obj, ICM426XX_RA_ACCEL_CONFIG_STATIC2, aaf_tab[ICM426xx_AAF_258Hz].delt << 1) ||
       !DevICM426xx_Reg_Read(sensor_obj, ICM426XX_RA_ACCEL_CONFIG_STATIC2, &read_out))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    if(read_out != (aaf_tab[ICM426xx_AAF_258Hz].delt << 1))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_AccAAF_Error, __FUNCTION__, __LINE__, ICM426XX_RA_ACCEL_CONFIG_STATIC2, read_out, aaf_tab[ICM426xx_AAF_258Hz].delt << 1);
        return false;
    }

    if(!DevICM426xx_Reg_Write(sensor_obj, ICM426XX_RA_ACCEL_CONFIG_STATIC3, aaf_tab[ICM426xx_AAF_258Hz].deltSqr & 0xFF) ||
       !DevICM426xx_Reg_Read(sensor_obj, ICM426XX_RA_ACCEL_CONFIG_STATIC3, &read_out))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    if(read_out != (aaf_tab[ICM426xx_AAF_258Hz].deltSqr & 0xFF))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_AccAAF_Error, __FUNCTION__, __LINE__, ICM426XX_RA_ACCEL_CONFIG_STATIC3, read_out, 
                         aaf_tab[ICM426xx_AAF_258Hz].deltSqr & 0xFF);
        return false;
    }

    if(!DevICM426xx_Reg_Write(sensor_obj, ICM426XX_RA_ACCEL_CONFIG_STATIC4, (aaf_tab[ICM426xx_AAF_258Hz].deltSqr >> 8) | (aaf_tab[ICM426xx_AAF_258Hz].bitshift << 4)) ||
       !DevICM426xx_Reg_Read(sensor_obj, ICM426XX_RA_ACCEL_CONFIG_STATIC4, &read_out))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    if(read_out != ((aaf_tab[ICM426xx_AAF_258Hz].deltSqr >> 8) | (aaf_tab[ICM426xx_AAF_258Hz].bitshift << 4)))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_AccAAF_Error, __FUNCTION__, __LINE__, ICM426XX_RA_ACCEL_CONFIG_STATIC4, read_out, 
                         (aaf_tab[ICM426xx_AAF_258Hz].deltSqr >> 8) | (aaf_tab[ICM426xx_AAF_258Hz].bitshift << 4));
        return false;
    }

    /* config UI filter */
    if(!DevICM426xx_SetUserBank(sensor_obj, ICM426XX_BANK_SELECT0))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    if(!DevICM426xx_Reg_Write(sensor_obj, ICM426XX_RA_GYRO_ACCEL_CONFIG0, ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY) ||
       !DevICM426xx_Reg_Read(sensor_obj, ICM426XX_RA_GYRO_ACCEL_CONFIG0, &read_out))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    if(read_out != (ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_UI_Filter_Error, __FUNCTION__, __LINE__, ICM426XX_RA_GYRO_ACCEL_CONFIG0, read_out, 
                         ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY);
        return false;
    }

    /* config interrupt */
    if(!DevICM426xx_Reg_Write(sensor_obj, ICM426XX_RA_INT_CONFIG, ICM426XX_INT1_MODE_PULSED | ICM426XX_INT1_DRIVE_CIRCUIT_PP | ICM426XX_INT1_POLARITY_ACTIVE_HIGH) ||
       !DevICM426xx_Reg_Read(sensor_obj, ICM426XX_RA_INT_CONFIG, &read_out))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    if(read_out != (ICM426XX_INT1_MODE_PULSED | ICM426XX_INT1_DRIVE_CIRCUIT_PP | ICM426XX_INT1_POLARITY_ACTIVE_HIGH))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_INT_Set_Error, __FUNCTION__, __LINE__, ICM426XX_RA_INT_CONFIG, read_out, 
                         ICM426XX_INT1_MODE_PULSED | ICM426XX_INT1_DRIVE_CIRCUIT_PP | ICM426XX_INT1_POLARITY_ACTIVE_HIGH);
        return false;
    }

    if(!DevICM426xx_Reg_Write(sensor_obj, ICM426XX_RA_INT_CONFIG0, ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR) ||
       !DevICM426xx_Reg_Read(sensor_obj, ICM426XX_RA_INT_CONFIG0, &read_out))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    if(read_out != ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR)
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_INT_Set_Error, __FUNCTION__, __LINE__, ICM426XX_RA_INT_CONFIG0, read_out, ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR);
        return false;
    }

    if(!DevICM426xx_Reg_Write(sensor_obj, ICM426XX_RA_INT_SOURCE0, ICM426XX_UI_DRDY_INT1_EN_ENABLED) ||
       !DevICM426xx_Reg_Read(sensor_obj, ICM426XX_RA_INT_SOURCE0, &read_out))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    if(read_out != ICM426XX_UI_DRDY_INT1_EN_ENABLED)
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_INT_Set_Error, __FUNCTION__, __LINE__, ICM426XX_RA_INT_SOURCE0, read_out, ICM426XX_UI_DRDY_INT1_EN_ENABLED);
        return false;
    }

    if(!DevICM426xx_Reg_Read(sensor_obj, ICM426XX_RA_INT_CONFIG1, &intConfig1Value))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }
    // Datasheet says: "User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation"
    intConfig1Value &= ~(1 << ICM426XX_INT_ASYNC_RESET_BIT);
    intConfig1Value |= (ICM426XX_INT_TPULSE_DURATION_8 | ICM426XX_INT_TDEASSERT_DISABLED);

    if(!DevICM426xx_Reg_Write(sensor_obj, ICM426XX_RA_INT_CONFIG1, intConfig1Value) ||
       !DevICM426xx_Reg_Read(sensor_obj, ICM426XX_RA_INT_CONFIG1, &read_out))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    if(read_out != intConfig1Value)
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_INT_Set_Error, __FUNCTION__, __LINE__, ICM426XX_RA_INT_CONFIG1, read_out, intConfig1Value);
        return false;
    }

    /* turn acc gyro on */
    if(!DevICM426xx_TurnOn_AccGyro(sensor_obj))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_AccGyr_TurnOn_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    /* config sample data range and odr */
    if(!DevICM426xx_Reg_Write(sensor_obj, ICM426XX_RA_GYRO_CONFIG0, sensor_obj->GyrTrip))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }
    sensor_obj->delay(15);

    if(!DevICM426xx_Reg_Read(sensor_obj, ICM426XX_RA_GYRO_CONFIG0, &read_out))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    if(read_out != sensor_obj->GyrTrip)
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_GyrRangeOdr_Set_Error, __FUNCTION__, __LINE__, ICM426XX_RA_GYRO_CONFIG0, read_out, sensor_obj->GyrTrip);
        return false;
    }

    if(!DevICM426xx_Reg_Write(sensor_obj, ICM426XX_RA_ACCEL_CONFIG0, sensor_obj->AccTrip))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);

        return false;
    }
    sensor_obj->delay(15);

    if(!DevICM426xx_Reg_Read(sensor_obj, ICM426XX_RA_ACCEL_CONFIG0, &read_out))
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_Reg_RW_Error, __FUNCTION__, __LINE__, 0, 0, 0);
        return false;
    }

    if(read_out != sensor_obj->AccTrip)
    {
        IMUData_SetError(&(sensor_obj->error), ICM426xx_AccRangeOdr_Set_Error, __FUNCTION__, __LINE__, ICM426XX_RA_ACCEL_CONFIG0, read_out, sensor_obj->AccTrip);
        return false;
    }

    IMUData_SetError(&(sensor_obj->error), ICM426xx_No_Error, "", 0, 0, 0, 0);
    return true;
}

static bool DevICM426xx_Sample(DevICM426xxObj_TypeDef *sensor_obj)
{
    uint8_t Acc_Tx[6] = {0};
    uint8_t Gyr_Tx[6] = {0};
    uint8_t Rx[12] = {0};

    if ((sensor_obj->error.code == ICM426xx_No_Error) && (sensor_obj->drdy))
    {
        sensor_obj->OriData.time_stamp = sensor_obj->get_timestamp();

        DevICM426xx_Regs_Read(sensor_obj, ICM426XX_RA_ACCEL_DATA_X1, Acc_Tx, Rx, 6);
        DevICM426xx_Regs_Read(sensor_obj, ICM426XX_RA_GYRO_DATA_X1, Gyr_Tx, (Rx + 6), 6);

        for (uint8_t axis = Axis_X; axis < Axis_Sum; axis++)
        {
            sensor_obj->OriData.acc_int[axis] = (int16_t)((Rx[axis * 2] << 8) | Rx[axis * 2 + 1]);
            sensor_obj->OriData.gyr_int[axis] = (int16_t)((Rx[axis * 2 + 6] << 8) | Rx[axis * 2 + 7]);

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

static void DevICM426xx_SetDRDY(DevICM426xxObj_TypeDef *sensor_obj)
{
    if(sensor_obj)
        sensor_obj->drdy = true;
}

static bool DevICM426xx_GetDRDY(DevICM426xxObj_TypeDef *sensor_obj)
{
    if(!sensor_obj)
        return false;
    
    return sensor_obj->drdy;
}

static IMU_Error_TypeDef DevICM426xx_GetError(DevICM426xxObj_TypeDef *sensor_obj)
{
    return sensor_obj->error;
}

static IMUData_TypeDef DevICM426xx_Get_Data(DevICM426xxObj_TypeDef *sensor_obj)
{
    IMUData_TypeDef tmp;

    memset(&tmp, NULL, sizeof(tmp));
    if (sensor_obj->error.code == ICM426xx_No_Error)
        return sensor_obj->OriData;

    return tmp;
}

static IMUModuleScale_TypeDef DevICM426xx_Get_Scale(const DevICM426xxObj_TypeDef *sensor_obj)
{
    IMUModuleScale_TypeDef scale_tmp;

    scale_tmp.acc_scale = sensor_obj->acc_scale;
    scale_tmp.gyr_scale = sensor_obj->gyr_scale;

    return scale_tmp;
}

/* specified anguler speed per millsecond */
static float DevICM426xx_Get_Specified_AngularSpeed_Diff(const DevICM426xxObj_TypeDef *sensor_obj)
{
    return sensor_obj->PHY_GyrTrip_Val / 1000.0f;
}


