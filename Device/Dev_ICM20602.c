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

static void DevICM20602_SetSampleRate(DevICM20602Obj_TypeDef *Obj, ICM20602_SampleRate_List rate)
{
    Obj->rate = rate;
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
