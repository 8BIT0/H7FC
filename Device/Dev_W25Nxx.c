#include "Dev_W25Nxx.h"

#define W25NXX_BUS_COMMU_TIMEOUT 100 /* unit: ms */

/* internal function */
static bool DevW25Nxx_Write(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint16_t len);
static bool DevW25Nxx_Read(DevW25NxxObj_TypeDef *dev, uint8_t *p_rx, uint16_t len);
static bool DevW25Nxx_Trans(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint8_t *p_rx, uint16_t len);
static DevW25Nxx_ProdType_List DevW25Nxx_Get_ProductID(DevW25NxxObj_TypeDef *dev);

/* external function */
static DevW25Nxx_Error_List DevW25Nxx_Init(DevW25NxxObj_TypeDef *dev);

DevW25Nxx_TypeDef DevW25Nxx = {
    .init = DevW25Nxx_Init,
};

static bool DevW25Nxx_Write(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint16_t len)
{
    uint16_t tx_out = 0;

    if ((dev == NULL) || \
        (dev->bus_tx == NULL) || \
        (dev->cs_ctl == NULL) || \
        (p_tx == NULL) || \
        (len == 0))
        return false;

    dev->cs_ctl(false);
    tx_out = dev->bus_tx(p_tx, len, W25NXX_BUS_COMMU_TIMEOUT);
    dev->cs_ctl(true);

    return tx_out ? true : false;
}

static bool DevW25Nxx_Read(DevW25NxxObj_TypeDef *dev, uint8_t *p_rx, uint16_t len)
{
    uint16_t rx_out = 0;

    if ((dev == NULL) || \
        (dev->bus_rx == NULL) || \
        (dev->cs_ctl == NULL) || \
        (p_rx == NULL) || \
        (len == 0))
        return false;

    dev->cs_ctl(false);
    rx_out = dev->bus_rx(p_rx, len, W25NXX_BUS_COMMU_TIMEOUT);
    dev->cs_ctl(true);

    return rx_out ? true : false;
}

static bool DevW25Nxx_Trans(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint8_t *p_rx, uint16_t len)
{
    uint16_t trans_out = 0;

    if ((dev == NULL) || \
        (dev->bus_trans == NULL) || \
        (dev->cs_ctl == NULL) || \
        (p_rx == NULL) || \
        (p_tx == NULL) || \
        (len == 0))
        return false;

    dev->cs_ctl(false);
    trans_out = dev->bus_trans(p_tx, p_rx, len, W25NXX_BUS_COMMU_TIMEOUT);
    dev->cs_ctl(true);

    return trans_out ? true : false;
}

static DevW25Nxx_Error_List DevW25Nxx_Init(DevW25NxxObj_TypeDef *dev)
{
    DevW25Nxx_ProdType_List ProdID = DevW25N_None;

    if (dev == NULL)
        return DevW25Nxx_Error;

    /* get product id */
    ProdID = DevW25Nxx_Get_ProductID(dev);
    if (ProdID == DevW25N_None)
        return DevW25Nxx_Error;
}

static DevW25Nxx_ProdType_List DevW25Nxx_Get_ProductID(DevW25NxxObj_TypeDef *dev)
{
    uint8_t tx_tmp[4] = {0};
    uint8_t rx_tmp[4] = {0};
    uint32_t ID = 0;
    bool trans_state = false;

    memset(tx_tmp, 0, sizeof(tx_tmp));
    memset(rx_tmp, 0, sizeof(rx_tmp));
    tx_tmp[0] = W25NXX_JEDEC_ID;

    if (dev == NULL)
        return DevW25N_None;

    trans_state = DevW25Nxx_Trans(dev, tx_tmp, rx_tmp, sizeof(rx_tmp));
    if (!trans_state)
        return DevW25N_None;

    memcpy(&ID, rx_tmp, sizeof(rx_tmp));
}


