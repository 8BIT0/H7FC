#include "Dev_Card.h"
#include "Srv_OsCommon.h"
#include "IO_Definition.h"
#include "debug_util.h"

#define LOCK_TIMEOUT 500 // unit: ms

/* internal vriable */
static bool read_lock = false;
static bool write_lock = false;
static uint32_t read_cnt = 0;
static uint32_t read_fin_cnt = 0;
static uint32_t read_state_err_cnt = 0;
static uint32_t write_cnt = 0;
static uint32_t write_fin_cnt = 0;
static uint32_t write_state_err_cnt = 0;
static uint32_t err_cnt = 0;

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
    uint32_t lock_time = 0;
    volatile BspSDMMC_OperationState_List bus_state;
    if ((Instance == NULL) || (p_data == NULL) || (block_num == 0) || (block == 0) || (block > Instance->info.BlockNbr))
        return false;
    
    /* check state first */
    bus_state = BspSDMMC.get_opr_state(&(Instance->SDMMC_Obj));
    if(bus_state == BspSDMMC_Opr_State_READY)
    {
        DebugPin.ctl(Debug_PB4, true);
        state = BspSDMMC.write(&(Instance->SDMMC_Obj), p_data, block, block_num);

        if(state)
        {
            write_cnt ++;
            write_lock = true;
            lock_time = SrvOsCommon.get_os_ms();
            /* wait sdmmc write semaphore */
            while(write_lock)
            {
                if((SrvOsCommon.get_os_ms() - lock_time) >= LOCK_TIMEOUT)
                {
                    write_lock = false;
                    return false;
                }
            }
        }
        else
            write_state_err_cnt ++;
    }
 
    DebugPin.ctl(Debug_PB4, false);

    return state;
}

static bool DevCard_Read(DevCard_Obj_TypeDef *Instance, uint32_t block, uint8_t *p_data, uint16_t data_size, uint16_t block_num)
{
    bool state = false;
    uint32_t lock_time = 0;
    volatile BspSDMMC_OperationState_List bus_state;
    if ((Instance == NULL) || (p_data == NULL) || (block_num == 0) || (block > Instance->info.BlockNbr) || (data_size < block_num * Instance->info.BlockSize))
        return false;

    /* check state first */
    bus_state = BspSDMMC.get_opr_state(&(Instance->SDMMC_Obj));
    if(bus_state == BspSDMMC_Opr_State_READY)
    {
        state = BspSDMMC.read(&(Instance->SDMMC_Obj), p_data, block, block_num);

        if(state)
        {
            read_cnt ++;
            read_lock = true;
            lock_time = SrvOsCommon.get_os_ms();
            /* wait sdmmc read semaphore */
            while(read_lock)
            {
                if((SrvOsCommon.get_os_ms() - lock_time) >= LOCK_TIMEOUT)
                {
                    read_lock = false;
                    return false;
                }
            }
        }
        else
            read_state_err_cnt ++;
    }

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
    write_fin_cnt ++;
    if(write_lock)
        write_lock = false;
}

static void DevCard_Read_FinCallback(uint8_t *p_data, uint16_t len)
{
    read_fin_cnt ++;
    if(read_lock)
        read_lock = false;
}

static void DevCard_Error_Callback(uint8_t *p_data, uint16_t len)
{
    err_cnt++;
}
