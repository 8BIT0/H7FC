#include "Storage_Bus_Port.h"
#include "HW_Def.h"

#if defined STM32H743xx
static SPI_HandleTypeDef ExtFlash_Bus_InstObj;
#elif defined AT32F435_437
void *ExtFlash_Bus_InstObj = NULL;
#endif

/* external function */
static void *Storage_External_Chip_Bus_Init(StorageBus_Malloc_Callback p_malloc, StorageBus_Free_Callback p_free);
static bool Storage_External_Chip_SelectPin_Ctl(bool state);
static uint16_t Storage_External_Chip_BusTx(uint8_t *p_data, uint16_t len, uint32_t time_out);
static uint16_t Storage_External_Chip_BusRx(uint8_t *p_data, uint16_t len, uint32_t time_out);
static uint16_t Storage_External_Chip_BusTrans(uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t time_out);

StorageBusApi_TypeDef StoragePort_Api = {
    .init = Storage_External_Chip_Bus_Init,
    .cs_ctl = Storage_External_Chip_SelectPin_Ctl,
    .bus_tx = Storage_External_Chip_BusTx,
    .bus_rx = Storage_External_Chip_BusRx,
    .bus_trans = Storage_External_Chip_BusTrans,
};

/************************************************** External Flash IO API Section ************************************************/
static void* Storage_External_Chip_Bus_Init(StorageBus_Malloc_Callback p_malloc, StorageBus_Free_Callback p_free)
{
    void *obj = NULL;

    if ((p_malloc == NULL) || \
        (p_free == NULL))
        return NULL;

#if (ExtFlash_Bus_Type == Storage_ChipBus_Spi)
        /* malloc bus object */
        obj = p_malloc(sizeof(BspSPI_Config_TypeDef));
        if (obj == NULL)
            return NULL;

        memset(obj, 0, sizeof(BspSPI_Config_TypeDef));

        To_NormalSPI_ObjPtr(obj)->BaudRatePrescaler = ExtFlash_Bus_Clock_Div;
        To_NormalSPI_ObjPtr(obj)->CLKPhase = ExtFlash_Bus_CLKPhase;
        To_NormalSPI_ObjPtr(obj)->CLKPolarity = ExtFlash_Bus_CLKPolarity;
        To_NormalSPI_ObjPtr(obj)->Instance = ExtFLash_Bus_Instance;
        To_NormalSPI_ObjPtr(obj)->Pin = ExtFlash_Bus_Pin;
        To_NormalSPI_ObjPtr(obj)->work_mode = BspSPI_Mode_Master;

        if (!ExtFlash_Bus_Api.init(To_NormalSPI_Obj(obj), &ExtFlash_Bus_InstObj) || \
            !BspGPIO.out_init(ExtFlash_CS_Pin))
            return NULL;

        return obj;
#elif (ExtFlash_Bus_Type == Storage_ChipBus_QSpi)
    /* still in developping */
    return NULL;
#else
    return NULL;
#endif
    return NULL;
}

static bool Storage_External_Chip_SelectPin_Ctl(bool state)
{
#if (ExtFlash_Bus_Type == Storage_ChipBus_Spi)
    BspGPIO.write(ExtFlash_CS_Pin, state);
#endif
    return true;
}

static uint16_t Storage_External_Chip_BusTx(uint8_t *p_data, uint16_t len, uint32_t time_out)
{
#if (ExtFlash_Bus_Type == Storage_ChipBus_Spi)
    if (p_data && len)
    {
        if (ExtFlash_Bus_Api.trans(&ExtFlash_Bus_InstObj, p_data, len, time_out))
            return len;
    }
#endif

    return 0;
}

static uint16_t Storage_External_Chip_BusRx(uint8_t *p_data, uint16_t len, uint32_t time_out)
{
#if (ExtFlash_Bus_Type == Storage_ChipBus_Spi)
    if (p_data && len)
    {
        if (ExtFlash_Bus_Api.receive(&ExtFlash_Bus_InstObj, p_data, len, time_out))
            return len;
    }
#endif

    return 0;
}

static uint16_t Storage_External_Chip_BusTrans(uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t time_out)
{
#if (ExtFlash_Bus_Type == Storage_ChipBus_Spi)
    if (tx && rx && len)
    {
        if (ExtFlash_Bus_Api.trans_receive(&ExtFlash_Bus_InstObj, tx, rx, len, time_out))
            return len;
    }
#endif

    return 0;
}
