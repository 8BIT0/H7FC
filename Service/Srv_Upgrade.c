/*
 *  Auther: 8_B!T0
 *  Noticed: this file use to upgrade App / Bootloader / Module Firmware 
 *  When at bootloader use this file can upgrade App and Module Firmware
 *  When at app use this file can upgrade Bootloader and Module Firmware
 *  
 *                      still in developping
 *                      F ------- Y ------ I
 *                 NOTHING BEING TESTED IN THIS FILE
 */
#include "Srv_Upgrade.h"
#include "util.h"
#include "../System/storage/Storage.h"
#include "Bsp_Flash.h"
#include "Srv_FileAdapter.h"

#define FIRMWARE_WAITTING_TIMEOUT   60000   /* unit: ms */
#define FIRMWARE_COMMU_TIMEOUT      1000    /* unit: ms */
#define DEFAULT_WINDOW_SIZE         100     /* unit: ms */

static const uint8_t AppVer[3] = {0, 0, 0};
#if defined MATEKH743_V1_5
static const uint8_t HWVer[3] = {0, 0, 1};
#elif defined BATEAT32F435_AIO
static const uint8_t HWVer[3] = {0, 0, 2};
#endif
#define AppBref "First Version of H7FC"
#define AppCompileData __DATA__
#define FIRMWARE_MAX_READ_SIZE (4 Kb)

#define UpgradeInfo_Sec  "Upgrade_Info"

typedef void (*Application_Func)(void);

static uint8_t upgrade_buf[FIRMWARE_MAX_READ_SIZE] = {0};

__attribute__((weak)) bool Write_OnChipFlash(uint32_t addr, uint8_t *p_data, uint16_t size){return false;}
__attribute__((weak)) bool Read_OnChipFlash(uint32_t addr, uint8_t *p_data, uint16_t size){return false;}
__attribute__((weak)) bool Erase_OnChipFlash(uint32_t addr, uint8_t *p_data, uint16_t size){return false;}

typedef enum
{
    UpgradeParam_None = 0,
    UpgradeParam_InValid,
    UpgradeParam_Valid,
} SrvUpgrade_ParamValid_List;

typedef struct
{
    bool access;
    uint16_t size;
    uint16_t total_size;
    uint8_t *p_buf;
} SrvUpgrade_Stream_TypeDef;

typedef struct
{
    bool init_state;

    SrvUpgrade_CodeStage_List CodeStage;
    SrvUpgrade_Stage_List PollingState;
    SrvUpgrade_PortDataProc_List PortDataState;
    
    uint32_t firmware_addr_s;   /* Application or Module firmware storaged address start pos */
    uint32_t firmware_addr_e;   /* Application or Module firmware storaged address end pos */
    uint32_t firmware_size;     /* total firmware size */
    uint32_t firmware_rw_size;  /* current read or write size */
    
    /* useless in app */
    uint32_t jump_time;
    uint32_t JumpAddr;
    uint32_t AppSize;

    uint32_t discard_time;

    uint32_t rec_timeout;
    uint32_t rec_time;
    
    Storage_ItemSearchOut_TypeDef UpgradeInfo_SO;
    
    SrvFileAdapterObj_TypeDef *adapter_obj;
    bool info_update;
    FileInfo_TypeDef FileInfo;

    SrvUpgrade_Stream_TypeDef proc_stream[2];

    uint8_t LogOut_Info[1024];
    uint16_t LogOut_Info_size;
} SrvUpgradeMonitor_TypeDef;

typedef struct
{
    uint32_t read_addr;
    uint32_t read_size;

    uint32_t write_addr;
    uint32_t write_size;

    uint32_t remain_size;
} FlashMonitor_TypeDef;

/* internal virable */
static SrvUpgradeMonitor_TypeDef Monitor = {
    .init_state = false,
};

/* internal function */
static void SrvUpgrade_Collect_Info(const char *format, ...);
static void SrvUpgrade_CheckUpgrade_OnBootUp(uint8_t code_stage);

/* external function */
static bool SrvUpgrade_Init(SrvUpgrade_CodeStage_List stage, uint32_t window_size);
static SrvUpgrade_Stage_List SrvUpgrade_StatePolling(uint32_t sys_time, SrvFileAdapter_Send_Func send);
static uint16_t SrvUpgrade_Get_Info(uint8_t *p_info, uint16_t len);
static void SrvUpgrade_ClearLog(void);
static void SrvUpgrade_JumpTo(void);
static bool SrvUpgrade_PushData(uint32_t sys_time, uint8_t *p_buf, uint16_t len);
static void SrvUpgrade_SetFileInfo(const FileInfo_TypeDef info);

/* external function */
SrvUpgrade_TypeDef SrvUpgrade = {
    .init = SrvUpgrade_Init,
    .polling = SrvUpgrade_StatePolling,
    .jump = SrvUpgrade_JumpTo,
    .get_log = SrvUpgrade_Get_Info,
    .clear_log = SrvUpgrade_ClearLog,
    .push_data = SrvUpgrade_PushData,
    .set_fileinfo = SrvUpgrade_SetFileInfo,
};

static bool SrvUpgrade_Init(SrvUpgrade_CodeStage_List stage, uint32_t window_size)
{
    if (sizeof(upgrade_buf) % 2)
        return false;

    /* reset file info */
    Monitor.info_update = false;
    memset(&Monitor.FileInfo, 0, sizeof(Monitor.FileInfo));

    /* get data from storage */
    Monitor.LogOut_Info_size = 0;

    memset(Monitor.LogOut_Info, 0, sizeof(Monitor.LogOut_Info));
    Monitor.LogOut_Info_size = 0;

    SrvUpgrade_Collect_Info("[SrvUpgrade Init]\r\n");
    SrvUpgrade_Collect_Info("\tOn Boot Stage\r\n");
    SrvUpgrade_Collect_Info("\tReading [Boot Info] from storage\r\n");
    
    /* set stream */
    Monitor.proc_stream[0].access = false;
    Monitor.proc_stream[0].size = 0;
    Monitor.proc_stream[0].total_size = sizeof(upgrade_buf) / 2;
    Monitor.proc_stream[0].p_buf = upgrade_buf;

    Monitor.proc_stream[1].access = false;
    Monitor.proc_stream[1].size = 0;
    Monitor.proc_stream[1].total_size = sizeof(upgrade_buf) / 2;
    Monitor.proc_stream[1].p_buf = &upgrade_buf[sizeof(upgrade_buf) / 2];
    Monitor.CodeStage = stage;

    if (stage == On_Boot)
    {
        Monitor.JumpAddr = Default_App_Address;
        Monitor.AppSize  = Default_App_Size;
        Monitor.jump_time = SrvOsCommon.get_os_ms();
        Monitor.jump_time += DEFAULT_WINDOW_SIZE;

        /* show jump time stamp */
        SrvUpgrade_Collect_Info("\tJump time: %d\r\n", Monitor.jump_time);
        SrvUpgrade_Collect_Info("\r\n");
    }

    Monitor.init_state = true;
    Monitor.PollingState = Stage_Init;
    Monitor.PortDataState = PortProc_None;
    
    /* 
     * whatever on boot or app
     * check upgrade on boot up
     */
    SrvUpgrade_CheckUpgrade_OnBootUp(stage);
    return true;
}

static void SrvUpgrade_CheckUpgrade_OnBootUp(uint8_t code_stage)
{
    SrvUpgradeInfo_TypeDef Info;

    memset(&Monitor.UpgradeInfo_SO, 0, sizeof(Monitor.UpgradeInfo_SO));
    memset(&Info, 0, sizeof(Info));

    /* check upgrade enable control first */
    Monitor.UpgradeInfo_SO = Storage.search(External_Flash, Para_Boot, UpgradeInfo_Sec);
    if (Monitor.UpgradeInfo_SO.item_addr && \
        (Storage.get(External_Flash, Para_Boot, Monitor.UpgradeInfo_SO.item, (uint8_t *)(&Info), sizeof(SrvUpgradeInfo_TypeDef)) == Storage_Error_None))
    {
        /* read parameter section */
        /* read boot firmware info */
        if ((code_stage == On_Boot) && Info.CTLReg.bit.App)
        {
            /* check hardware version */
            if (memcmp(Info.AF_Info.HW_Ver, HWVer, sizeof(HWVer)) == 0)
            {
                /* check app upgrade */
            }
        }
        else if ((code_stage == On_App) && Info.CTLReg.bit.Boot)
        {
            /* check hardware version */
            if (memcmp(Info.BF_Info.HW_Ver, HWVer, sizeof(HWVer)) == 0)
            {
                /* check boot upgrade */
            }
        }
    }
    else
    {
        memcpy(Info.AF_Info.HW_Ver, HWVer, sizeof(Info.AF_Info.HW_Ver));
        memcpy(Info.BF_Info.HW_Ver, HWVer, sizeof(Info.AF_Info.HW_Ver));
        Storage.create(External_Flash, Para_Boot, UpgradeInfo_Sec, (uint8_t *)(&Info), sizeof(SrvUpgrade_TypeDef));
        Monitor.UpgradeInfo_SO = Storage.search(External_Flash, Para_Boot, UpgradeInfo_Sec);
    }

    if (Monitor.UpgradeInfo_SO.item_addr == 0)
    {
        Monitor.init_state = false;
        Monitor.PollingState = Stage_UpgradeInfo_Error;
    }
}

static SrvUpgrade_PortDataProc_List SrvUpgrade_PortProcPolling(uint32_t sys_time)
{
    SrvUpgrade_PortDataProc_List ret;
    Adapter_Polling_State adapter_state; 

    if (Monitor.adapter_obj == NULL)
        return PortProc_Deal_Error;

    for (uint8_t i = 0; i < 2; i ++)
    {
        if (!Monitor.proc_stream[i].access && Monitor.proc_stream[i].size)
        {
            Monitor.proc_stream[i].access = true;

            if (SrvFileAdapter.push_to_stream(Monitor.proc_stream[i].p_buf, Monitor.proc_stream[i].size))
            {
                memset(Monitor.proc_stream[i].p_buf, 0, Monitor.proc_stream[i].size);
                Monitor.proc_stream[i].size = 0;
            }

            Monitor.proc_stream[i].access = false;
        }
    }
    
    switch((uint8_t) Monitor.PortDataState)
    {
        case PortProc_None:
            Monitor.rec_timeout = sys_time + FIRMWARE_COMMU_TIMEOUT;
        case PortProc_Deal_Pack:
            Monitor.PortDataState = PortProc_Deal_Pack;
            adapter_state = SrvFileAdapter.polling(sys_time, Monitor.adapter_obj);
            ret = PortProc_Deal_Pack;
            
            /* check adapter state */
            switch ((uint8_t)adapter_state)
            {
                case Adapter_Proc_Done: ret = ProtProc_Finish; break;
                case Adapter_Proc_Failed: ret = PortProc_Deal_Error; break;
                default: break;
            }
            break;

        case PortProc_Deal_Error:
            Monitor.PortDataState = PortProc_None;
            ret = PortProc_Deal_Error;
            break;

        default:
            ret = PortProc_Unknown;
            break;
    }

    /* check for time out */
    if ((ret != ProtProc_Finish) && (sys_time >= Monitor.rec_timeout))
    {
        Monitor.info_update = false;
        Monitor.PortDataState = PortProc_None;
        ret = PortProc_Deal_TimeOut;
    }

    return ret;
}

static void SrvUpgrade_App_Updating()
{
    // for (; ; )
    // {

    // }

    /* if upgrade successed clear flag */
}

static void SrvUpgrade_Boot_Updating()
{
    // for (; ; )
    // {

    // }
    
    /* if upgrade successed clear flag */
}

static SrvUpgrade_Stage_List SrvUpgrade_On_PortProc_Finish(void)
{
    SrvUpgradeInfo_TypeDef Info;
    memset(&Info, 0, sizeof(Info));

    /* all file data received */
    /* update upgrade info to storage */
    Storage.get(External_Flash, Para_Boot, Monitor.UpgradeInfo_SO.item, (uint8_t *)&Info, sizeof(SrvUpgradeInfo_TypeDef));

    if (Monitor.FileInfo.File_Type == FileType_APP)
    {
        SrvUpgrade_Collect_Info("\tApp firmware receive finished\r\n");
        /* update app firmware info */
        Info.CTLReg.bit.App = true;
        Info.AF_Info = SrvFileAdapter.get_file_info(Monitor.adapter_obj);
    }
    else if (Monitor.FileInfo.File_Type == FileType_Boot)
    {
        /* update boot firmware info */
        SrvUpgrade_Collect_Info("\tBoot firmware receive finished\r\n");
        Info.CTLReg.bit.Boot = true;
        Info.BF_Info = SrvFileAdapter.get_file_info(Monitor.adapter_obj);
    }

    Storage.update(External_Flash, Para_Boot, Monitor.UpgradeInfo_SO.item.data_addr, (uint8_t *)&Info, sizeof(SrvUpgradeInfo_TypeDef));

    /* destory adapter obj */
    SrvFileAdapter.destory(Monitor.adapter_obj);
    Monitor.adapter_obj = NULL;

    /* check for firmware upgrade */
    if (Monitor.CodeStage == On_App)
    {
        if (memcmp(Info.BF_Info.HW_Ver, HWVer, sizeof(Info.BF_Info.HW_Ver)) != 0)
            return Stage_Upgrade_Error;

        /* upgrade App */
        return Stage_Boot_Upgrading;
    }
    else if (Monitor.CodeStage == On_Boot)
    {
        if (memcmp(Info.AF_Info.HW_Ver, HWVer, sizeof(Info.AF_Info.HW_Ver)) != 0)
            return Stage_Upgrade_Error;

        /* upgrade boot */
        return Stage_App_Upgrading;
    }

    return Stage_Upgrade_Error;
}

static SrvUpgrade_Stage_List SrvUpgrade_StatePolling(uint32_t sys_time, SrvFileAdapter_Send_Func send)
{
    uint8_t i = 0;
    
    if ((Monitor.FileInfo.File_Type == FileType_None) || (Monitor.FileInfo.File_Type > FileType_Boot))
    {
        Monitor.PollingState = Stage_FileInfo_Error;
    }
    else
    {
        /* check file info */
        if (Monitor.info_update && (Monitor.adapter_obj == NULL))
        {
            Monitor.adapter_obj = SrvFileAdapter.create(Monitor.FileInfo.Adapter_Type, Monitor.FileInfo);
            if (Monitor.adapter_obj == NULL)
                Monitor.PollingState = Stage_Adapter_Error;
        }
    }

    if (Monitor.adapter_obj && send)
        SrvFileAdapter.set_send(Monitor.adapter_obj, send);

    switch ((uint8_t) Monitor.PollingState)
    {
        case Stage_Init:            
            Monitor.PollingState = Stage_Process_PortData;
            Monitor.discard_time = sys_time + FIRMWARE_WAITTING_TIMEOUT;
            return Monitor.PollingState;

        case Stage_Process_PortData:
            if ((Monitor.CodeStage == On_Boot) && \
                (sys_time >= Monitor.jump_time))
            {
                Monitor.PollingState = Stage_ReadyToJump;
                SrvUpgrade_Collect_Info("[Jump Preparetion]\r\n");
                SrvUpgrade_Collect_Info("\tDisabling Irq\r\n");

                return Stage_ReadyToJump;
            }
            else
            {
                /* check double stream data */
                for (i = 0; i < 2; i++)
                {
                    if (Monitor.proc_stream[i].size)
                    {
                        /* received data from port */
                        Monitor.discard_time = sys_time + FIRMWARE_WAITTING_TIMEOUT;
                        break;
                    }
                }
                
                switch (SrvUpgrade_PortProcPolling(sys_time))
                {
                    case ProtProc_Finish:
                        Monitor.PollingState = SrvUpgrade_On_PortProc_Finish();
                        Monitor.discard_time = sys_time + FIRMWARE_WAITTING_TIMEOUT;
                        break;

                    /* still developping this branch */
                    case PortProc_Deal_TimeOut: /* Monitor.PollingState = Stage_TimeOut;*/ break;
                    case PortProc_Deal_Error: Monitor.PollingState = Stage_PortData_Error; break;
                    default: break;
                }

                /* check for processing time out when at app */
                if ((Monitor.CodeStage == On_App) && (Monitor.discard_time <= sys_time))
                {
                    Monitor.PollingState = Stage_TimeOut;
                    if (Monitor.adapter_obj)
                    {
                        SrvFileAdapter.destory(Monitor.adapter_obj);
                        Monitor.adapter_obj = NULL;
                    }
                }
            }
            return Stage_Process_PortData;

        /* firmware upgrading */
        case Stage_App_Upgrading:
            SrvUpgrade_App_Updating();
            return Stage_App_Upgrading;

        case Stage_Boot_Upgrading:
            SrvUpgrade_Boot_Updating();
            return Stage_Boot_Upgrading;

        /* when at bootloader */
        case Stage_ReadyToJump:
            return Stage_ReadyToJump;

        /* error process */
        case Stage_FileInfo_Error:
            break;

        case Stage_Upgrade_Error:
        case Stage_Adapter_Error:
        case Stage_PortData_Error:
            Monitor.PollingState = Stage_Init;
            SrvFileAdapter.destory(Monitor.adapter_obj);
            Monitor.adapter_obj = NULL;
            return Stage_Adapter_Error;

        case Stage_TimeOut:
            Monitor.PollingState = Stage_Init;
            return Stage_TimeOut;
        
        case Stage_JumpError:
            return Stage_JumpError;

        case Stage_UpgradeInfo_Error:
            return Stage_UpgradeInfo_Error;

        default: return Stage_Unknow;
    }
}

static void SrvUpgrade_SetFileInfo(const FileInfo_TypeDef info)
{
    Monitor.info_update = true;
    memcpy(&Monitor.FileInfo, &info, sizeof(FileInfo_TypeDef));
}

/* call this function in receive thread or irq */
static bool SrvUpgrade_PushData(uint32_t sys_time, uint8_t *p_buf, uint16_t len)
{
    if (Monitor.init_state)
    {
        Monitor.rec_time = sys_time;
        Monitor.rec_timeout = Monitor.rec_time + FIRMWARE_COMMU_TIMEOUT;

        for (uint8_t i = 0; i < 2; i++)
        {
            if ((!Monitor.proc_stream[i].access) && \
                ((Monitor.proc_stream[i].size + len) <= Monitor.proc_stream[i].total_size))
            {
                Monitor.proc_stream[i].access = true;
                memcpy(&Monitor.proc_stream[i].p_buf[Monitor.proc_stream[i].size], p_buf, len);
                Monitor.proc_stream[i].size += len;
                Monitor.proc_stream[i].access = false;
                return true;
            }
        }
    }

    return false;
}

static void SrvUpgrade_Set_FileInfo(const FileInfo_TypeDef info)
{
    memcpy(&Monitor.FileInfo, &info, sizeof(FileInfo_TypeDef));
}

static bool SrvUpgrade_CheckAppAddr(uint32_t addr, uint32_t size)
{
    /* check app base address */
    if ((addr & 0xFF000000) != ((uint32_t)&__rom_s))
    {
        SrvUpgrade_Collect_Info("[ ------- Bad Jump Address 0x%08x -------]\r\n", addr);
        SrvUpgrade_Collect_Info("\r\n");
        /* error address */
        return false;
    }

    /* app start address check */
    if (addr < ((uint32_t)&__boot_e))
    {
        SrvUpgrade_Collect_Info("[ ------- Bad Jump Address 0x%08x ------- ]\r\n", addr);
        SrvUpgrade_Collect_Info("\r\n");
        /* app start address is lower then end of the boot section */
        return false;
    }

    /* app size range check */
    if ((addr + size) > ((uint32_t)&__rom_e))
    {
        SrvUpgrade_Collect_Info("[ ------- App Size Over Range ------- ]\r\n");
        SrvUpgrade_Collect_Info("\tApp Start Addr: 0x%08x\r\n", addr);
        SrvUpgrade_Collect_Info("\tApp End   Addr: 0x%08x\r\n", (addr + size));
        SrvUpgrade_Collect_Info("\tApp Size:       %d\r\n", size);
        SrvUpgrade_Collect_Info("\tFlash End Addr: 0x%08x\r\n", (uint32_t)&__rom_e);
        SrvUpgrade_Collect_Info("\r\n");
        /* end of app address is overrange */
        return false;
    }

    return true;
}

static void SrvUpgrade_JumpTo(void)
{
    uint32_t jump_addr = 0;

    if (Monitor.JumpAddr && Monitor.AppSize && SrvUpgrade_CheckAppAddr(Monitor.JumpAddr, Monitor.AppSize))
    {
        /* log out jump addr and app size */
        SrvUpgrade_Collect_Info("[Jump To App]\r\n");
        SrvUpgrade_Collect_Info("\tApp Address: 0x%08x\r\n", Monitor.JumpAddr);
        SrvUpgrade_Collect_Info("\tApp Size:    0x%08x\r\n", Monitor.AppSize);
        SrvOsCommon.delay_ms(10);

        /* disable all interrupt before jump to app */
        SrvOsCommon.disable_all_irq();

        jump_addr = *(volatile uint32_t *)(Monitor.JumpAddr + 4);
        __set_MSP(*(volatile uint32_t *)Monitor.JumpAddr);
        ((Application_Func)jump_addr)();
    }
    else
        Monitor.PollingState = Stage_JumpError;
}

static void SrvUpgrade_Collect_Info(const char *format, ...)
{
    va_list args;
    uint16_t buf_remain = 0;
    uint16_t buf_capacity = sizeof(Monitor.LogOut_Info);
    int16_t send_len = 0;

    if (format && (Monitor.LogOut_Info_size < buf_capacity))
    {
        buf_remain = buf_capacity - Monitor.LogOut_Info_size;

        va_start(args, format);

        send_len = vsnprintf((char *)(Monitor.LogOut_Info + Monitor.LogOut_Info_size), buf_remain, format, args);
        if (send_len > 0)
            Monitor.LogOut_Info_size += send_len;

        va_end(args);
    }
}

static uint16_t SrvUpgrade_Get_Info(uint8_t *p_info, uint16_t len)
{
    uint16_t log_size = 0;

    if (p_info && len && (len >= Monitor.LogOut_Info_size) && Monitor.init_state)
    {
        log_size = Monitor.LogOut_Info_size;
        memcpy(p_info, Monitor.LogOut_Info, Monitor.LogOut_Info_size);
        log_size = Monitor.LogOut_Info_size;
    }

    return log_size;
}

static void SrvUpgrade_ClearLog(void)
{
    memset(Monitor.LogOut_Info, 0, sizeof(Monitor.LogOut_Info));
    Monitor.LogOut_Info_size = 0;
}
