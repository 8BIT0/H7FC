/*
 *  Auther: 8_B!T0
 *  Noticed: this file use to upgrade App / Module Firmware 
 *  When at bootloader use this file can upgrade App and Module Firmware
 *  When at app use this file can upgrade Bootloader and Module Firmware
 *  
 *                      F ------- Y ------ I
 *                      still in developping
 */
#include "Srv_Upgrade.h"
#include "util.h"
#include "../System/storage/Storage.h"
#include "Bsp_Flash.h"
#include "shell_port.h"
#include "HW_Def.h"
#include "debug_util.h"

#define FIRMWARE_WAITTING_TIMEOUT   60000   /* unit: ms */
#define FIRMWARE_COMMU_TIMEOUT      1000    /* unit: ms */
#define DEFAULT_WINDOW_SIZE         100     /* unit: ms */

const uint8_t AppVer[3] = {0, 0, 17};

#define AppBref "First Version of H7FC"
#define AppCompileData __DATA__
#define FIRMWARE_MAX_READ_SIZE (4 Kb)

#define UpgradeInfo_Sec  "Upgrade_Info"

#define UPGERADE_INFO(fmt,...) Debug_Print(&DebugPort, "[ UPGRADE INFO ] ", fmt, ##__VA_ARGS__)

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

    SrvUpgrade_Stage_List PollingState;
    SrvUpgrade_PortDataProc_List PortDataState;

    uint32_t discard_time;

    uint32_t rec_timeout;
    uint32_t rec_time;
    
    Storage_ItemSearchOut_TypeDef UpgradeInfo_SO;
    
    bool info_update;

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
static void SrvUpgrade_CheckUpgrade_OnBootUp(void);

/* external function */
static bool SrvUpgrade_Init(void);
static SrvUpgrade_Stage_List SrvUpgrade_StatePolling(uint32_t sys_time);
static uint16_t SrvUpgrade_Get_Info(uint8_t *p_info, uint16_t len);
static bool SrvUpgrade_PushData(uint32_t sys_time, uint8_t *p_buf, uint16_t len);

/* external function */
SrvUpgrade_TypeDef SrvUpgrade = {
    .init = SrvUpgrade_Init,
    .polling = SrvUpgrade_StatePolling,
    .push_data = SrvUpgrade_PushData,
};

static bool SrvUpgrade_Init(void)
{
    if (sizeof(upgrade_buf) % 2)
        return false;

    /* reset file info */
    Monitor.info_update = false;

    /* get data from storage */
    Monitor.LogOut_Info_size = 0;

    memset(Monitor.LogOut_Info, 0, sizeof(Monitor.LogOut_Info));
    Monitor.LogOut_Info_size = 0;
    
    /* set stream */
    Monitor.proc_stream[0].access = false;
    Monitor.proc_stream[0].size = 0;
    Monitor.proc_stream[0].total_size = sizeof(upgrade_buf) / 2;
    Monitor.proc_stream[0].p_buf = upgrade_buf;

    Monitor.proc_stream[1].access = false;
    Monitor.proc_stream[1].size = 0;
    Monitor.proc_stream[1].total_size = sizeof(upgrade_buf) / 2;
    Monitor.proc_stream[1].p_buf = &upgrade_buf[sizeof(upgrade_buf) / 2];

    Monitor.init_state = true;
    Monitor.PollingState = Stage_Init;
    Monitor.PortDataState = PortProc_None;
    
    /* 
     * whatever on boot or app
     * check upgrade on boot up
     */
    SrvUpgrade_CheckUpgrade_OnBootUp();
    return true;
}

static void SrvUpgrade_CheckUpgrade_OnBootUp(void)
{
    SrvUpgradeInfo_TypeDef Info;
    uint32_t file_size = 0;
    uint16_t update_size = 0;
    uint32_t addr_offset = 0;

    memset(&Monitor.UpgradeInfo_SO, 0, sizeof(Monitor.UpgradeInfo_SO));
    memset(&Info, 0, sizeof(Info));

    /* check upgrade enable control first */
    Monitor.UpgradeInfo_SO = Storage.search(Para_Boot, UpgradeInfo_Sec);
    if (Monitor.UpgradeInfo_SO.item_addr)
    {
        Storage.get(Para_Boot, Monitor.UpgradeInfo_SO.item, (uint8_t *)(&Info), sizeof(SrvUpgradeInfo_TypeDef));
        /* read parameter section */
        while (file_size)
        {
            update_size = file_size;
            if (file_size > 1024)
                update_size = 1024;

            /* read firmware from storage */
            memset(upgrade_buf, 0, update_size);
            Storage.read_firmware(addr_offset, upgrade_buf, update_size);

            SrvOsCommon.enter_critical();
            /* write firmware to boot flash */
            Storage.write_firmware(Internal_Flash, addr_offset, upgrade_buf, update_size);
            SrvOsCommon.exit_critical();

            file_size -= update_size;
            addr_offset += update_size;
        }
    }
    else
    {
        Storage.create(Para_Boot, UpgradeInfo_Sec, (uint8_t *)(&Info), sizeof(SrvUpgradeInfo_TypeDef));
        Monitor.UpgradeInfo_SO = Storage.search(Para_Boot, UpgradeInfo_Sec);
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

    for (uint8_t i = 0; i < 2; i ++)
    {
        if (!Monitor.proc_stream[i].access && Monitor.proc_stream[i].size)
        {
            Monitor.proc_stream[i].access = true;

            Monitor.proc_stream[i].access = false;
        }
    }
    
    switch((uint8_t) Monitor.PortDataState)
    {
        case PortProc_None:
            Monitor.rec_timeout = sys_time + FIRMWARE_COMMU_TIMEOUT;
        case PortProc_Deal_Pack:
            Monitor.PortDataState = PortProc_Deal_Pack;
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
    if ((ret != ProtProc_Finish) && (sys_time >= Monitor.rec_timeout))
    {
        Monitor.info_update = false;
        Monitor.PortDataState = PortProc_None;
        ret = PortProc_Deal_TimeOut;
    }

    return ret;
}

static SrvUpgrade_Stage_List SrvUpgrade_On_PortProc_Finish(void)
{
    SrvUpgradeInfo_TypeDef Info;

    memset(&Info, 0, sizeof(Info));

    /* all file data received */
    /* update upgrade info to storage */
    Storage.get(Para_Boot, Monitor.UpgradeInfo_SO.item, (uint8_t *)&Info, sizeof(SrvUpgradeInfo_TypeDef));
    
    /* update app firmware info */
    Info.CTLReg.bit.App = true;

    Storage.update(Para_Boot, Monitor.UpgradeInfo_SO.item.data_addr, (uint8_t *)&Info, sizeof(SrvUpgradeInfo_TypeDef));
    /* upgrade App */
    return Stage_Reboot;
}

static SrvUpgrade_Stage_List SrvUpgrade_StatePolling(uint32_t sys_time)
{
    uint8_t i = 0;

    switch ((uint8_t) Monitor.PollingState)
    {
        case Stage_Init:            
            Monitor.PollingState = Stage_Process_PortData;
            Monitor.discard_time = sys_time + FIRMWARE_WAITTING_TIMEOUT;
            return Monitor.PollingState;

        case Stage_Process_PortData:
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
                case PortProc_Deal_Error: 
                    Monitor.PollingState = Stage_PortData_Error;
                    break;

                default: break;
            }
            return Stage_Process_PortData;

        /* firmware upgrading */
        case Stage_Reboot:
            return Stage_Reboot;

        /* when at bootloader */
        case Stage_ReadyToJump:
            return Stage_ReadyToJump;

        /* error process */
        case Stage_FileInfo_Error:
            return Stage_FileInfo_Error;

        case Stage_Upgrade_Error:
        case Stage_Adapter_Error:
        case Stage_PortData_Error:
            return Stage_Adapter_Error;

        case Stage_TimeOut:
            Monitor.PollingState = Stage_Init;
            return Stage_TimeOut;

        case Stage_UpgradeInfo_Error:
            return Stage_UpgradeInfo_Error;

        default: return Stage_Unknow;
    }
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

static void SrvUpgrade_Show_CurAppVer(void)
{
    Shell *shell_obj = Shell_GetInstence();

    if (shell_obj)
    {
        shellPrint(shell_obj, "[ Current App ] SW Version: %d.%d.%d\r\n", AppVer[0], AppVer[1], AppVer[2]);
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, app_ver, SrvUpgrade_Show_CurAppVer, check app version);

