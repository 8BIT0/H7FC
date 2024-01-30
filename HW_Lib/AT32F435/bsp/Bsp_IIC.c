#include "Bsp_IIC.h"

/* external function */
static bool BspIIC_Init(BspIICObj_TypeDef *obj);
static bool BspIIC_DeInit(BspIICObj_TypeDef *obj);
static bool BspIIC_Read(BspIICObj_TypeDef *obj, uint16_t dev_addr, uint16_t reg, uint8_t *p_buf, uint16_t len);
static bool BspIIC_Write(BspIICObj_TypeDef *obj, uint16_t dev_addr, uint16_t reg, uint8_t *p_buf, uint16_t len);

BspIIC_TypeDef BspIIC = {
    .init = BspIIC_Init,
    .de_init = BspIIC_DeInit,
    .read = BspIIC_Read,
    .write = BspIIC_Write,
};

static bool BspIIC_Init(BspIICObj_TypeDef *obj)
{
    if(obj)
    {

    }

    return false;
}

static bool BspIIC_DeInit(BspIICObj_TypeDef *obj)
{
    if(obj)
    {

    }

    return false;
}

static bool BspIIC_Read(BspIICObj_TypeDef *obj, uint16_t dev_addr, uint16_t reg, uint8_t *p_buf, uint16_t len)
{
    if(obj && p_buf && len)
    {

    }

    return false;
}

static bool BspIIC_Write(BspIICObj_TypeDef *obj, uint16_t dev_addr, uint16_t reg, uint8_t *p_buf, uint16_t len)
{
    if(obj && p_buf && len)
    {

    }

    return false;
}
