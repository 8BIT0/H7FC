#include "Dev_Card.h"
#include "semaphore.h"
#include "Srv_OsCommon.h"
#include "IO_Definition.h"

/* External Function */
static DevCard_Error_List DevCard_Init(DevCard_Obj_TypeDef *Instance);
static DevCard_Error_List DevCard_GetError(DevCard_Obj_TypeDef *Instance);
static DevCard_Info_TypeDef DevCard_GetInfo(DevCard_Obj_TypeDef *Instance);
static bool DevCard_Read(DevCard_Obj_TypeDef *Instance, uint32_t block, uint8_t *p_data, uint16_t data_size, uint16_t block_num);
static bool DevCard_Write(DevCard_Obj_TypeDef *Instance, uint32_t block, uint8_t *p_data, uint16_t data_size, uint16_t block_num);
static void DevCard_Write_FinCallback(uint8_t *p_data, uint16_t len);
static void DevCard_Read_FinCallback(uint8_t *p_data, uint16_t len);
static void DevCard_Error_Callback(uint8_t *p_data, uint16_t len);

DevCard_TypeDef DevCard = {
    .Init = DevCard_Init,
    .Get_ErrorCode = DevCard_GetError,
    .Get_Info = DevCard_GetInfo,
    .read = DevCard_Read,
    .write = DevCard_Write,
};

static DevCard_Error_List DevCard_Init(DevCard_Obj_TypeDef *Instance)
{
    if (Instance == NULL)
        return DevCard_Obj_Error;

    if (!BspSDMMC.init(&(Instance->SDMMC_Obj)))
    {
        Instance->error_code = DevCard_Bus_Error;
        return DevCard_Bus_Error;
    }

    if (!BspSDMMC.info(&(Instance->SDMMC_Obj), &(Instance->SDMMC_Obj.info)))
    {
        Instance->info.valid = false;
        Instance->error_code = DevCard_Info_Error;
        return DevCard_Info_Error;
    }

    /* set irq callback */
    BspSDMMC.set_callback(&(Instance->SDMMC_Obj), BspSDMMC_Callback_Type_Write, DevCard_Write_FinCallback);
    BspSDMMC.set_callback(&(Instance->SDMMC_Obj), BspSDMMC_Callback_Type_Read, DevCard_Read_FinCallback);
    BspSDMMC.set_callback(&(Instance->SDMMC_Obj), BspSDMMC_Callback_Type_Error, DevCard_Error_Callback);

    /* create semaphore */

    Instance->info.BlockNbr = Instance->SDMMC_Obj.info.BlockNbr;
    Instance->info.BlockSize = Instance->SDMMC_Obj.info.BlockSize;
    Instance->info.CardSpeed = Instance->SDMMC_Obj.info.CardSpeed;
    Instance->info.CardType = Instance->SDMMC_Obj.info.CardType;
    Instance->info.CardVersion = Instance->info.CardVersion;
    Instance->info.Class = Instance->info.Class;
    Instance->info.LogBlockNbr = Instance->SDMMC_Obj.info.LogBlockNbr;
    Instance->info.LogBlockSize = Instance->SDMMC_Obj.info.LogBlockSize;
    Instance->info.RelCardAdd = Instance->SDMMC_Obj.info.RelCardAdd;

    Instance->info.valid = true;
    Instance->info.RmnByteInCurBlock = 0;
    Instance->info.UsdBlockNbr = 0;
    Instance->info.RmnBlockNbr = Instance->info.BlockNbr;

    Instance->error_code = DevCard_No_Error;

    return DevCard_No_Error;
}

static DevCard_Error_List DevCard_GetError(DevCard_Obj_TypeDef *Instance)
{
    if (Instance == NULL)
        return DevCard_Obj_Error;

    return Instance->error_code;
}

static DevCard_Info_TypeDef DevCard_GetInfo(DevCard_Obj_TypeDef *Instance)
{
    DevCard_Info_TypeDef info_tmp = {0};

    if (Instance != NULL)
    {
        info_tmp = Instance->info;
    }

    return info_tmp;
}

static bool DevCard_Write(DevCard_Obj_TypeDef *Instance, uint32_t block, uint8_t *p_data, uint16_t data_size, uint16_t block_num)
{
    bool state = false;
    if ((Instance == NULL) || (p_data == NULL) || (block_num == 0) || (block == 0) || (block > Instance->info.BlockNbr))
        return false;
    
    state = BspSDMMC.write(&(Instance->SDMMC_Obj), p_data, block, block_num);
    /* wait sdmmc write semaphore */

    return state;
}

static bool DevCard_Read(DevCard_Obj_TypeDef *Instance, uint32_t block, uint8_t *p_data, uint16_t data_size, uint16_t block_num)
{
    bool state = false;
    if ((Instance == NULL) || (p_data == NULL) || (block_num == 0) || (block > Instance->info.BlockNbr) || (data_size < block_num * Instance->info.BlockSize))
        return false;

    state = BspSDMMC.read(&(Instance->SDMMC_Obj), p_data, block, block_num);
    /* wait sdmmc read semaphore */

    return state;
}

static bool DevCard_Erase(DevCard_Obj_TypeDef *Instance, uint32_t block, uint16_t size)
{
    if ((Instance == NULL) || (size == 0))
        return false;

    return true;
}

static void DevCard_Write_FinCallback(uint8_t *p_data, uint16_t len)
{

}

static void DevCard_Read_FinCallback(uint8_t *p_data, uint16_t len)
{

}

static void DevCard_Error_Callback(uint8_t *p_data, uint16_t len)
{

}
