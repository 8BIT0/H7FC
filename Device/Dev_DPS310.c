#include "Dev_DPS310.h"

/* internal function */
static uint8_t DevDPS1310_ReadByteOnReg(DevDPS310Obj_TypeDef *obj, uint8_t reg_addr);
static bool DevDPS310_WriteByteToReg(DevDPS310Obj_TypeDef *obj, uint8_t reg_addr, uint8_t data);
static bool DevDPS310_ReadBlockOnReg(DevDPS310Obj_TypeDef *obj, DevDPS310_RegBlock_TypeDef block, uint8_t *p_data);


/* internal variable define */
typedef enum
{
    DevDPS310_PROD_ID = 0,
    DevDPS310_REV_ID,
    DevDPS310_TEMP_SENSOR,    // internal vs external
    DevDPS310_TEMP_SENSORREC, //temperature sensor recommendation
    DevDPS310_TEMP_SE,        //temperature shift enable (if temp_osr>3)
    DevDPS310_PRS_SE,         //pressure shift enable (if prs_osr>3)
    DevDPS310_FIFO_FL,        //FIFO flush
    DevDPS310_FIFO_EMPTY,     //FIFO empty
    DevDPS310_FIFO_FULL,      //FIFO full
    DevDPS310_INT_HL,
    DevDPS310_INT_SEL,         //interrupt select
    DevDPS310_Reg_Sum,
}DevDPS310_RegisterDefIndex_List;

const DevDPS310_RegMask_TypeDef DevDPS310_Reg[DevDPS310_Reg_Sum] = {
    {0x0D, 0x0F, 0}, // PROD_ID
    {0x0D, 0xF0, 4}, // REV_ID
    {0x07, 0x80, 7}, // TEMP_SENSOR
    {0x28, 0x80, 7}, // TEMP_SENSORREC
    {0x09, 0x08, 3}, // TEMP_SE
    {0x09, 0x04, 2}, // PRS_SE
    {0x0C, 0x80, 7}, // FIFO_FL
    {0x0B, 0x01, 0}, // FIFO_EMPTY
    {0x0B, 0x02, 1}, // FIFO_FULL
    {0x09, 0x80, 7}, // INT_HL
    {0x09, 0x70, 4}, // INT_SEL
};

const DevDPS310_RegBlock_TypeDef DevDPS310_CoeffBlock = {0x10, 18};

const int32_t DevDPS310_Scaling_Factory[DEV_DPS310_NUM_OF_SCAL_FACTS] = {
    524288,
    1572864,
    3670016,
    7864320,
    253952,
    516096,
    1040384,
    2088960
};

static bool DevDPS310_PreInit(DevDPS310Obj_TypeDef *obj, DevDPS310_BusWrite write, DevDPS310_BusRead read)
{
    if(obj && write && read)
    {
        obj->bus_tx = write;
        obj->bus_rx = read;

        return true;
    }

    return false;
}

static bool DevDPS310_Init(DevDPS310Obj_TypeDef *obj)
{
    if(obj && obj->bus_tx && obj->bus_rx)
    {
        return true;
    }

    return false;
}

static bool DevDPS310_StandBy(DevDPS310Obj_TypeDef *obj)
{
    if(obj)
    {

    }

    return false;
}

static uint8_t DevDPS310_Get_ProdID(DevDPS310Obj_TypeDef *obj)
{


    return 0;
}

static bool DevDPS310_ReadBlockOnReg(DevDPS310Obj_TypeDef *obj, DevDPS310_RegBlock_TypeDef block, uint8_t *p_data)
{
    uint8_t read_len = block.len + 1;
    uint8_t trans_tmp[read_len] = {0};

    if(obj && obj->bus_rx && p_data && block.len)
    {
        trans_tmp[0] = block.addr;

        if(obj->bus_rx(obj->DevAddr, trans_tmp, read_len, false))
        {
            for(uint8_t i = 0; i < block.len; i++)
            {
                p_data[i] = trans_tmp[i + 1];
            }

            return true;
        }
    }

    return false;
}

static uint8_t DevDPS1310_ReadByteOnReg(DevDPS310Obj_TypeDef *obj, uint8_t reg_addr)
{
    uint8_t trans_tmp[2] = {0};

    if(obj && obj->bus_rx)
    {
        /* first byte is register address */
        trans_tmp[0] = reg_addr;

        if(!obj->bus_rx(obj->DevAddr, trans_tmp, 2, false))
            return 0;

        return reg_addr[1];
    }

    return 0;
}

static bool DevDPS310_WriteByteToReg(DevDPS310Obj_TypeDef *obj, uint8_t reg_addr, uint8_t data)
{
    uint8_t trans_tmp[2] = {0};

    if(obj && obj->bus_tx)
    {
        trans_tmp[0] = reg_addr;
        trans_tmp[1] = data;

        if(obj->bus_tx(obj->DevAddr, trans_tmp, 2, true))
        {
            /* read out data from register */
            if (DevDPS1310_ReadByteOnReg(obj, reg_addr) == data)
                return true;
        }
    }

    return false;
}