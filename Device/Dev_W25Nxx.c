#include "Dev_W25Nxx.h"

/* test code */
#include "HW_Def.h"
#include "debug_util.h"

#define W25NXX_TAG "[ W25NXX INFO ]"
#define W25NXX_INFO(fmt, ...) Debug_Print(&DebugPort, W25NXX_TAG, fmt, ##__VA_ARGS__)
/* test code */

#define W25NXX_BUS_COMMU_TIMEOUT 100 /* unit: ms */

/* internal function */
static bool DevW25Nxx_Write(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint16_t len);
static bool DevW25Nxx_Read(DevW25NxxObj_TypeDef *dev, uint8_t *p_rx, uint16_t len);
static bool DevW25Nxx_Trans(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint8_t *p_rx, uint16_t len);
static DevW25Nxx_ProdType_List DevW25Nxx_Get_ProductID(DevW25NxxObj_TypeDef *dev);

/* external function */
static DevW25Nxx_Error_List DevW25Nxx_Init(DevW25NxxObj_TypeDef *dev);
static DevW25Nxx_DeviceInfo_TypeDef DevW25Nxx_Get_Info(DevW25NxxObj_TypeDef *dev);

DevW25Nxx_TypeDef DevW25Nxx = {
    .init = DevW25Nxx_Init,
    .info = DevW25Nxx_Get_Info,
};

static bool DevW25Nxx_Write(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint16_t len)
{
    uint16_t tx_out = 0;

    if ((dev == NULL) || \
        (dev->bus_tx == NULL) || \
        (p_tx == NULL) || \
        (len == 0))
        return false;

    tx_out = dev->bus_tx(p_tx, len, W25NXX_BUS_COMMU_TIMEOUT);

    return tx_out ? true : false;
}

static bool DevW25Nxx_Read(DevW25NxxObj_TypeDef *dev, uint8_t *p_rx, uint16_t len)
{
    uint16_t rx_out = 0;

    if ((dev == NULL) || \
        (dev->bus_rx == NULL) || \
        (p_rx == NULL) || \
        (len == 0))
        return false;

    rx_out = dev->bus_rx(p_rx, len, W25NXX_BUS_COMMU_TIMEOUT);

    return rx_out ? true : false;
}

static bool DevW25Nxx_Trans(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint8_t *p_rx, uint16_t len)
{
    uint16_t trans_out = 0;

    if ((dev == NULL) || \
        (dev->bus_trans == NULL) || \
        (p_rx == NULL) || \
        (p_tx == NULL) || \
        (len == 0))
        return false;

    trans_out = dev->bus_trans(p_tx, p_rx, len, W25NXX_BUS_COMMU_TIMEOUT);

    return trans_out ? true : false;
}

static DevW25Nxx_Error_List DevW25Nxx_Init(DevW25NxxObj_TypeDef *dev)
{
    DevW25Nxx_ProdType_List ProdID = DevW25N_None;

    if (dev->delay_ms == NULL)
        return DevW25Nxx_Error;

    /* soft reset */

    dev->delay_ms(100);

    /* get product id */
    ProdID = DevW25Nxx_Get_ProductID(dev);
    if (ProdID == DevW25N_None)
        return DevW25Nxx_Error;

    return DevW25Nxx_Ok;
}

static DevW25Nxx_ProdType_List DevW25Nxx_Get_ProductID(DevW25NxxObj_TypeDef *dev)
{
    uint8_t tx_tmp[2] = {0};
    uint8_t rx_tmp[4] = {0};
    uint32_t ID = 0;

    memset(tx_tmp, 0, sizeof(tx_tmp));
    memset(rx_tmp, 0, sizeof(rx_tmp));
    tx_tmp[0] = W25NXX_JEDEC_ID;

    dev->cs_ctl(false);
    DevW25Nxx_Write(dev, tx_tmp, sizeof(tx_tmp));
    DevW25Nxx_Read(dev, rx_tmp, sizeof(rx_tmp));
    dev->cs_ctl(true);

    memcpy(&ID, rx_tmp, sizeof(rx_tmp));
    W25NXX_INFO("ID 0x%08x\r\n", ID);

    return DevW25N_None;
}

static DevW25Nxx_DeviceInfo_TypeDef DevW25Nxx_Get_Info(DevW25NxxObj_TypeDef *dev)
{
    DevW25Nxx_DeviceInfo_TypeDef info_tmp;
    memset(&info_tmp, 0, sizeof(DevW25Nxx_DeviceInfo_TypeDef));
    return info_tmp;
}

