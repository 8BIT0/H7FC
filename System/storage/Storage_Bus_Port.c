#include "Storage_Bus_Port.h"
#include "HW_Def.h"

#if defined STM32H743xx
static SPI_HandleTypeDef ExtFlash_Bus_InstObj;
#elif defined AT32F435_437
void *ExtFlash_Bus_InstObj = NULL;
#endif

/* external function */
static bool Storage_External_Chip_SelectPin_Ctl(bool state);
static uint16_t Storage_External_Chip_BusTx(uint8_t *p_data, uint16_t len, uint32_t time_out);
static uint16_t Storage_External_Chip_BusRx(uint8_t *p_data, uint16_t len, uint32_t time_out);
static uint16_t Storage_External_Chip_BusTrans(uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t time_out);

StorageBusApi_TypeDef StoragePort_Api = {
    .cs_ctl = Storage_External_Chip_SelectPin_Ctl,
    .bus_tx = Storage_External_Chip_BusTx,
    .bus_rx = Storage_External_Chip_BusRx,
    .bus_trans = Storage_External_Chip_BusTrans,
};

/************************************************** External Flash IO API Section ************************************************/
static bool Storage_External_Chip_Bus_Init(Storage_ExtFlash_BusType_List bus_type, StorageBus_Malloc_Callback p_malloc, StorageBus_Free_Callback p_free)
{
    if ((ExtFlash_Bus_InstObj == NULL) || \
        (p_malloc == NULL) || \
        (p_free == NULL))
        return false;

    if (!ExtFlash_Bus_Api.init(To_NormalSPI_Obj(bus_cfg), &ExtFlash_Bus_InstObj) || \
        !BspGPIO.out_init(ExtFlash_CS_Pin))
        return false;

    return true;
}

static bool Storage_External_Chip_SelectPin_Ctl(bool state)
{
    BspGPIO.write(ExtFlash_CS_Pin, state);
    return true;
}

static uint16_t Storage_External_Chip_BusTx(uint8_t *p_data, uint16_t len, uint32_t time_out)
{
    BspSPI_NorModeConfig_TypeDef *p_cfg = To_NormalSPI_ObjPtr(Storage_Monitor.ExtBusCfg_Ptr);

    if (p_data && len && p_cfg && p_cfg->Instance && ExtFlash_Bus_InstObj)
    {
        if (ExtFlash_Bus_Api.trans(&ExtFlash_Bus_InstObj, p_data, len, time_out))
            return len;
    }

    return 0;
}

static uint16_t Storage_External_Chip_BusRx(uint8_t *p_data, uint16_t len, uint32_t time_out)
{
    BspSPI_NorModeConfig_TypeDef *p_cfg = To_NormalSPI_ObjPtr(Storage_Monitor.ExtBusCfg_Ptr);

    if (p_data && len && p_cfg && p_cfg->Instance && ExtFlash_Bus_InstObj)
    {
        if (ExtFlash_Bus_Api.receive(&ExtFlash_Bus_InstObj, p_data, len, time_out))
            return len;
    }

    return 0;
}

static uint16_t Storage_External_Chip_BusTrans(uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t time_out)
{
    BspSPI_NorModeConfig_TypeDef *p_cfg = To_NormalSPI_ObjPtr(Storage_Monitor.ExtBusCfg_Ptr);

    if (tx && rx && len && p_cfg && p_cfg->Instance && ExtFlash_Bus_InstObj)
    {
        if (ExtFlash_Bus_Api.trans_receive(&ExtFlash_Bus_InstObj, tx, rx, len, time_out))
            return len;
    }

    return 0;
}
