#include "Dev_MPU6000.h"

static bool DevMPU6000_Reg_Read(DevMPU6000Obj_TypeDef *sensor_obj, uint8_t addr, uint8_t *rx)
{
    uint8_t write_buff[2] = {0};
    uint8_t read_buff[2] = {0};
    bool state = false;

    if (sensor_obj == NULL || sensor_obj->cs_ctl == NULL || sensor_obj->bus_trans == NULL)
        return false;

    write_buff[0] = addr | MPU6000_WRITE_MASK;

    /* CS High */
    sensor_obj->cs_ctl(true);

    state = sensor_obj->bus_trans(write_buff, read_buff, 2);

    /* CS Low */
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

    /* CS High */
    sensor_obj->cs_ctl(true);

    state = sensor_obj->bus_trans(write_buff, NULL, 2);

    /* CS Low */
    sensor_obj->cs_ctl(false);

    return state;
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

    if (!sensor_obj->cs_init() || !sensor_obj->bus_init() || !sensor_obj->set_SPI_1MSpeed())
    {
        sensor_obj->error = MPU6000_Interface_Error;
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

    /* set SPI Bus Speed to 20M before inertial sensor sample */
    if (!sensor_obj->set_SPI_20MSpeed())
    {
        sensor_obj->error = MPU6000_BusSampleSpeed_Error;
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

static bool DevMPU6000_GetReady(DevMPU6000Obj_TypeDef *sensor_obj)
{
    return sensor_obj->drdy;
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
        sensor_obj->OriData_Lst = sensor_obj->OriData;
        return sensor_obj->OriData;
    }
    else
        return sensor_obj->OriData_Lst;
}

//     spiSetClkDivisor(&gyro->dev, spiCalculateDivider(MPU6000_MAX_SPI_INIT_CLK_HZ));

//     // Device was already reset during detection so proceed with configuration

//     // Clock Source PPL with Z axis gyro reference
//     spiWriteReg(&gyro->dev, MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
//     delayMicroseconds(15);

//     // Disable Primary I2C Interface
//     spiWriteReg(&gyro->dev, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
//     delayMicroseconds(15);

//     spiWriteReg(&gyro->dev, MPU_RA_PWR_MGMT_2, 0x00);
//     delayMicroseconds(15);

//     // Accel Sample Rate 1kHz
//     // Gyroscope Output Rate =  1kHz when the DLPF is enabled
//     spiWriteReg(&gyro->dev, MPU_RA_SMPLRT_DIV, gyro->mpuDividerDrops);
//     delayMicroseconds(15);

//     // Gyro +/- 2000 DPS Full Scale
//     spiWriteReg(&gyro->dev, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
//     delayMicroseconds(15);

//     // Accel +/- 16 G Full Scale
//     spiWriteReg(&gyro->dev, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
//     delayMicroseconds(15);

//     spiWriteReg(&gyro->dev, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR
//     delayMicroseconds(15);

// #ifdef USE_MPU_DATA_READY_SIGNAL
//     spiWriteReg(&gyro->dev, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
//     delayMicroseconds(15);
// #endif

//     spiSetClkDivisor(&gyro->dev, spiCalculateDivider(MPU6000_MAX_SPI_CLK_HZ));
//     delayMicroseconds(1);