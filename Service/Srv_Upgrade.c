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

#define FIRMWARE_WAITTING_TIMEOUT   10000   /* unit: ms */
#define FIRMWARE_COMMU_TIMEOUT      1000    /* unit: ms */
#define DEFAULT_WINDOW_SIZE         100     /* unit: ms */

#define AppVer {0, 0, 0}
#define AppBref "First Version of H7FC"
#define AppCompileData __DATA__
#define FIRMWARE_MAX_READ_SIZE (4 Kb)

/* get virable from .ld file defined */
extern uint32_t __rom_s;
extern uint32_t __rom_e;
extern uint32_t __boot_s;
extern uint32_t __boot_e;

typedef void (*Application_Func)(void);

#define Boot_Address_Base   ((uint32_t)(&__boot_s))
#define Boot_Section_Size   ((uint32_t)&__boot_e - (uint32_t)&__boot_s)

#define Default_App_Address ((uint32_t)&__boot_e)
#define Default_App_Size    ((uint32_t)&__rom_e - Default_App_Address)

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
    SrvUpgrade_ParamValid_List ParamStatus;
    UpgradeInfo_TypeDef Info;
    
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
    
    SrvFileAdapterObj_TypeDef *adapter_obj;
    bool info_update;
    Upgrade_FileInfo_TypeDef FileInfo;

    SrvUpgrade_Stream_TypeDef proc_stream[2];

    uint8_t LogOut_Info[1024];
    uint16_t LogOut_Info_size;
} SrvUpgradeMonitor_TypeDef;

/* internal virable */
static SrvUpgradeMonitor_TypeDef Monitor = {
    .init_state = false,
};

/* internal function */
static void SrvUpgrade_Collect_Info(const char *format, ...);

/* external function */
static bool SrvUpgrade_Init(SrvUpgrade_CodeStage_List stage, uint32_t window_size);
static SrvUpgrade_Stage_List SrvUpgrade_StatePolling(uint32_t sys_time);
static uint16_t SrvUpgrade_Get_Info(uint8_t *p_info, uint16_t len);
static void SrvUpgrade_ClearLog(void);
static void SrvUpgrade_JumpTo(void);
static bool SrvUpgrade_PushData(uint32_t sys_time, uint8_t *p_buf, uint16_t len);
static void SrvUpgrade_SetFileInfo(const Upgrade_FileInfo_TypeDef info);

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
    Storage_ItemSearchOut_TypeDef search_out;

    if (sizeof(upgrade_buf) % 2)
        return false;

    /* reset file info */
    Monitor.info_update = false;
    memset(&Monitor.FileInfo, 0, sizeof(Monitor.FileInfo));

    /* get data from storage */
    memset(&Monitor.Info, 0, sizeof(UpgradeInfo_TypeDef));
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
 
    if (stage == On_Boot)
    {
        Monitor.CodeStage = On_Boot;
        search_out = Storage.search(External_Flash, Para_Boot, "Boot Info");
        SrvUpgrade_Collect_Info("\t\t[Boot Info] Addr  -> %d\r\n", search_out.item_addr);
        SrvUpgrade_Collect_Info("\t\t[Boot Info] index -> %d\r\n", search_out.item_index);
        
        Monitor.JumpAddr = Default_App_Address;
        Monitor.AppSize  = Default_App_Size;
        Monitor.ParamStatus = UpgradeParam_InValid;
        if ((search_out.item_addr != 0) && \
            (Storage.get(External_Flash, Para_Boot, search_out.item, (uint8_t *)&Monitor.Info, sizeof(UpgradeInfo_TypeDef)) == Storage_Error_None))
        {
            /* show boot item info */

            Monitor.ParamStatus = UpgradeParam_None;
            Monitor.jump_time = SrvOsCommon.get_os_ms();
            if (window_size >= DEFAULT_WINDOW_SIZE)
            {
                Monitor.jump_time += window_size;
            }
            else
                Monitor.jump_time += DEFAULT_WINDOW_SIZE;

            if (Monitor.Info.reg.bit.App || Monitor.Info.reg.bit.Module)
                Monitor.ParamStatus = UpgradeParam_Valid;
        }
        
        if (Monitor.ParamStatus == UpgradeParam_Valid)
        {
            SrvUpgrade_Collect_Info("\tBoot Parameter Valid\r\n");

            if (Monitor.Info.reg.bit.App)
                SrvUpgrade_Collect_Info("\t\t---[App] Got New Firmware\r\n");

            if (Monitor.Info.reg.bit.Module)
                SrvUpgrade_Collect_Info("\t\t---[External Module] Got New Firmware\r\n");
        }
        else if (Monitor.ParamStatus == UpgradeParam_InValid)
            SrvUpgrade_Collect_Info("\tBoot Parameter Invalid\r\n");

        /* show jump time stamp */
        SrvUpgrade_Collect_Info("\tJump time: %d\r\n", Monitor.jump_time);
        SrvUpgrade_Collect_Info("\r\n");
    }
    else if (stage == On_App)
    {
        Monitor.CodeStage = On_App;
        if (Monitor.Info.reg.bit.Boot || Monitor.Info.reg.bit.Module)
        {
            Monitor.ParamStatus = UpgradeParam_Valid;

            /* read parameter */
        }
    }
    else
        /* unknow stage */
        return false;

    Monitor.init_state = true;
    Monitor.PollingState = Stage_Init;
    Monitor.PortDataState = PortProc_None;
    return true;
}

static SrvUpgrade_PortDataProc_List SrvUpgrade_PortProcPolling(uint32_t sys_time)
{
    SrvUpgrade_PortDataProc_List ret;
    SrvUpgrade_Stream_TypeDef *inuse_stream = NULL;

    if (Monitor.adapter_obj == NULL)
        return PortProc_Deal_Error;

    for (uint8_t i = 0; i < 2; i ++)
    {
        if (!Monitor.proc_stream[i].access && Monitor.proc_stream[i].size)
        {
            inuse_stream = &Monitor.proc_stream[i];
            inuse_stream->access = true;
            break;
        }
    }

    switch((uint8_t) Monitor.PortDataState)
    {
        case PortProc_None:
            Monitor.rec_timeout = sys_time + FIRMWARE_COMMU_TIMEOUT;
        case PortProc_Deal_Pack:
            Monitor.PortDataState = PortProc_Deal_Pack;
            
            if (inuse_stream)
                SrvFileAdapter.polling(&Monitor.adapter_obj, inuse_stream->p_buf, &inuse_stream->size);
                inuse_stream->access = false;
            ret = PortProc_Deal_Pack;
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
    if (sys_time >= Monitor.rec_timeout)
    {
        memset(&Monitor.FileInfo, 0, sizeof(Monitor.FileInfo));
        Monitor.info_update = false;
        Monitor.PortDataState = PortProc_None;
        ret = PortProc_Deal_TimeOut;
    }

    return ret;
}

static SrvUpgrade_Stage_List SrvUpgrade_StatePolling(uint32_t sys_time)
{
    Storage_ItemSearchOut_TypeDef search_out;
    uint8_t i = 0;

    memset(&search_out, 0, sizeof(Storage_ItemSearchOut_TypeDef));

    /* check file info */
    if (Monitor.info_update && (Monitor.adapter_obj == NULL))
    {
        Monitor.adapter_obj = SrvFileAdapter.create(Monitor.FileInfo.Adapter_Type);
        if (Monitor.adapter_obj == NULL)
            Monitor.PollingState = Stage_Adapter_Error;
    }

    switch ((uint8_t) Monitor.PollingState)
    {
        case Stage_Init:
            if (Monitor.ParamStatus == UpgradeParam_Valid)
            {
                if ((Monitor.CodeStage == On_Boot) && Monitor.Info.reg.bit.App)
                {
                    Monitor.PollingState = Stage_Checking_App_Firmware;
                    return Stage_Checking_App_Firmware;
                }

                if ((Monitor.CodeStage == On_App) && Monitor.Info.reg.bit.Boot)
                {
                    Monitor.PollingState = Stage_Checking_Boot_Firmware;
                    return Stage_Checking_Boot_Firmware;
                }
            }
            
            Monitor.PollingState = Stage_Wait_PortData;
            Monitor.discard_time = sys_time + FIRMWARE_WAITTING_TIMEOUT;
            return Monitor.PollingState;

        case Stage_Checking_App_Firmware:
            return Stage_Checking_App_Firmware;

        case Stage_Checking_Boot_Firmware:
            return Stage_Checking_Boot_Firmware;

        case Stage_Wait_PortData:
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
                        Monitor.PollingState = Stage_Processing_PortData;
                        break;
                    }
                }

                /* check for processing time out when at app */
                if ((Monitor.CodeStage == On_App) && (Monitor.discard_time <= sys_time))
                {
                    Monitor.PollingState = Stage_Commu_TimeOut;
                    if (Monitor.adapter_obj)
                    {
                        SrvFileAdapter.destory(Monitor.adapter_obj);
                        Monitor.adapter_obj = NULL;
                    }
                }
            }
            return Stage_Wait_PortData;

        case Stage_Processing_PortData:
            Monitor.discard_time = sys_time + FIRMWARE_WAITTING_TIMEOUT;
            switch (SrvUpgrade_PortProcPolling(sys_time))
            {
                case PortProc_Deal_Pack:
                    break;

                case PortProc_Deal_TimeOut:
                case PortProc_Deal_Error:
                default:
                    /* clear stream */
                    for (i = 0; i < 2; i ++)
                    {
                        Monitor.proc_stream[i].size = 0;
                        memset(Monitor.proc_stream[i].p_buf, 0, Monitor.proc_stream[i].total_size);
                    }
                    Monitor.PollingState = Stage_Commu_TimeOut;
            }
            return Stage_Processing_PortData;

        /* when at bootloader */
        case Stage_ReadyToJump:
            return Stage_ReadyToJump;

        case Stage_Adapter_Error:
            Monitor.PollingState = Stage_Init;
            return Stage_Adapter_Error;

        case Stage_Commu_TimeOut:
            Monitor.PollingState = Stage_Init;
            return Stage_Commu_TimeOut;
        
        case Stage_JumpError:
            return Stage_JumpError;

        default: return Stage_Unknow;
    }
}

static void SrvUpgrade_SetFileInfo(const Upgrade_FileInfo_TypeDef info)
{
    Monitor.info_update = true;
    memcpy(&Monitor.FileInfo, &info, sizeof(Upgrade_FileInfo_TypeDef));
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

static void SrvUpgrade_Set_FileInfo(const Upgrade_FileInfo_TypeDef info)
{
    memcpy(&Monitor.FileInfo, &info, sizeof(Upgrade_FileInfo_TypeDef));
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
