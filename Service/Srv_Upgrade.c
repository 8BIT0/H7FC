#include "Srv_Upgrade.h"
#include "util.h"
#include "../System/storage/Storage.h"
#include "Bsp_Flash.h"

#define FIRMWARE_WAITTING_TIMEOUT   5000    /* unit: ms */
#define FIRMWARE_PROTO_TIMEOUT      1000    /* unit: ms */
#define DEFAULT_WINDOW_SIZE         100     /* unit: ms */

#define BootVer {0, 0, 0}
#define BootBref "First Version of H7FC Bootloader"
#define BootCompileData __DATA__
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

typedef enum
{
    UpgradeParam_None = 0,
    UpgradeParam_InValid,
    UpgradeParam_Valid,
} SrvUpgrade_ParamValid_List;

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

    uint32_t jump_time;
    uint32_t JumpAddr;
    uint32_t AppSize;

    FirmwareInfo_TypeDef Firmware_Info;
    uint32_t rec_time;
    bool buf_accessing;
    uint8_t buf[FIRMWARE_MAX_READ_SIZE];
    uint16_t buf_size;

    uint8_t LogOut_Info[1024];
    uint16_t LogOut_Info_size;
} SrvUpgradeMonitor_TypeDef;

typedef struct
{
    PortData_DecodeState_List state;

    uint8_t *buf_head;
    uint16_t buf_size;

    uint8_t *p_buf;
    uint16_t len;
} SrvUpgrade_DecodeOut_TypeDef;

/* internal virable */
static SrvUpgradeMonitor_TypeDef Monitor = {
    .init_state = false,
};

/* internal function */
static void SrvUpgrade_Collect_Info(const char *format, ...);
static void SrvUpgrade_JumpTo(void);

/* external function */
static bool SrvUpgrade_Init(SrvUpgrade_CodeStage_List stage, uint32_t window_size);
static SrvUpgrade_Stage_List SrvUpgrade_StatePolling(void);
static uint16_t SrvUpgrade_Get_Info(uint8_t *p_info, uint16_t len);
static void SrvUpgrade_ClearLog(void);

/* external function */
SrvUpgrade_TypeDef SrvUpgrade = {
    .init = SrvUpgrade_Init,
    .polling = SrvUpgrade_StatePolling,
    .jump = SrvUpgrade_JumpTo,
    .get_log = SrvUpgrade_Get_Info,
    .clear_log = SrvUpgrade_ClearLog,
};

static bool SrvUpgrade_Init(SrvUpgrade_CodeStage_List stage, uint32_t window_size)
{
    Storage_ItemSearchOut_TypeDef search_out;

    /* init mcu internal flash */
    if (!BspFlash.init())
    {
        SrvUpgrade_Collect_Info("\r\n[MCU Internal Flash Init Error]\r\n");
        SrvUpgrade_Collect_Info("\tUpgrade Init Failed\r\n");
        return false;
    }
    SrvUpgrade_Collect_Info("[MCU Internal Flash Init Done]\r\n");

    /* get data from storage */
    memset(&Monitor.Info, 0, sizeof(UpgradeInfo_TypeDef));
    Monitor.LogOut_Info_size = 0;

    memset(Monitor.LogOut_Info, 0, sizeof(Monitor.LogOut_Info));
    Monitor.LogOut_Info_size = 0;

    memset(Monitor.buf, 0, FIRMWARE_MAX_READ_SIZE);
    Monitor.buf_size = 0;
    
    SrvUpgrade_Collect_Info("[SrvUpgrade Init]\r\n");
    SrvUpgrade_Collect_Info("\tOn Boot Stage\r\n");
    SrvUpgrade_Collect_Info("\tReading [Boot Info] from storage\r\n");
    
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
        switch ((uint8_t)stage)
        {
            case On_App:
                if (Monitor.Info.reg.bit.Boot || Monitor.Info.reg.bit.Module)
                    Monitor.ParamStatus = UpgradeParam_Valid;
                break;

            case On_Boot:
                if (Monitor.Info.reg.bit.App || Monitor.Info.reg.bit.Module)
                    Monitor.ParamStatus = UpgradeParam_Valid;

                /* update app address and app size */
                break;

            default:
                Monitor.ParamStatus = UpgradeParam_InValid;
                break;
        }
        
        Monitor.CodeStage = stage;
    }
    
    if (Monitor.ParamStatus == UpgradeParam_InValid)
    {
        SrvUpgrade_Collect_Info("\tBoot Parameter Invalid\r\n");
    }
    else if (Monitor.ParamStatus == UpgradeParam_Valid)
    {
        SrvUpgrade_Collect_Info("\tBoot Parameter Valid\r\n");

        if (Monitor.Info.reg.bit.App)
            SrvUpgrade_Collect_Info("\t\t---[App] Got New Firmware\r\n");

        if (Monitor.Info.reg.bit.Module)
            SrvUpgrade_Collect_Info("\t\t---[External Module] Got New Firmware\r\n");
    }

    Monitor.jump_time = SrvOsCommon.get_os_ms();
    if (window_size >= DEFAULT_WINDOW_SIZE)
    {
        Monitor.jump_time += window_size;
    }
    else
        Monitor.jump_time += DEFAULT_WINDOW_SIZE;

    /* show jump time stamp */
    SrvUpgrade_Collect_Info("\tJump time: %d\r\n", Monitor.jump_time);
    SrvUpgrade_Collect_Info("\r\n");

    Monitor.init_state = true;
    Monitor.PollingState = Stage_Init;
    Monitor.PortDataState = PortProc_None;
    return true;
}

static SrvUpgrade_DecodeOut_TypeDef SrvUpgrade_RecData_Decode(SrvUpgrade_PortDataProc_List stage)
{
    uint8_t id = 0;
    uint8_t module_id = 0;
    SrvUpgrade_DecodeOut_TypeDef decode_out;
    
    memset(&decode_out, 0, sizeof(SrvUpgrade_DecodeOut_TypeDef));
    if (Monitor.buf_size)
    {
        Monitor.buf_accessing = true;
        
        /* deal with buf */
        switch ((uint8_t) stage)
        {
            case PortProc_Check_FileAdapter_EnableSig:
                break;
            
            default:
                decode_out.state = Decode_Failed;
                break;
        }

        Monitor.buf_accessing = false;
    }

    return decode_out;
}

static SrvUpgrade_PortDataProc_List SrvUpgrade_PortProcPolling(uint32_t sys_time)
{
    SrvUpgrade_DecodeOut_TypeDef decode_out;
    memset(&decode_out, 0, sizeof(SrvUpgrade_DecodeOut_TypeDef));

    switch((uint8_t) Monitor.PortDataState)
    {
        case PortProc_None:
            Monitor.PortDataState = PortProc_Check_FileAdapter_EnableSig;
        
        case PortProc_Check_FileAdapter_EnableSig:
            decode_out = SrvUpgrade_RecData_Decode(PortProc_Check_FileAdapter_EnableSig);
            if (decode_out.p_buf && decode_out.len && (decode_out.state == Decode_Successed))
            {
                /* deal with buf */

                Monitor.PortDataState = PortProc_Check_FirmwareInfo;
            }
            else if (decode_out.state == Decode_Failed)
            {
                Monitor.PortDataState = PortProc_None;
                Monitor.PollingState = Stage_Wait_PortData;
            }
            else
            {
                /* check time out */
                if (sys_time >= Monitor.rec_time)
                {
                    /* process time out */
                    Monitor.PortDataState = PortProc_Deal_TimeOut;
                }
            }
            return Monitor.PortDataState;
        
        case PortProc_Check_FirmwareInfo:
            return PortProc_Check_FirmwareInfo;

        case PortProc_Deal_Error:
            Monitor.PortDataState = PortProc_None;
            Monitor.PollingState = Stage_Wait_PortData;

            /* clear firmware info in storag */
            return PortProc_Deal_Error;

        case PortProc_Deal_TimeOut:
            Monitor.PortDataState = PortProc_None;
            Monitor.PollingState = Stage_Wait_PortData;
            return PortProc_Deal_TimeOut;

        default: return PortProc_Unknown;
    }
}

static SrvUpgrade_Stage_List SrvUpgrade_StatePolling(void)
{
    Storage_ItemSearchOut_TypeDef search_out;
    FirmwareInfo_TypeDef FrimInfo;
    uint32_t sys_time = SrvOsCommon.get_os_ms();

    memset(&search_out, 0, sizeof(Storage_ItemSearchOut_TypeDef));
    memset(&FrimInfo, 0, sizeof(FirmwareInfo_TypeDef));

    switch ((uint8_t) Monitor.PollingState)
    {
        case Stage_Init:
            if (Monitor.ParamStatus == UpgradeParam_Valid)
            {
                if (Monitor.Info.reg.bit.App)
                {
                    Monitor.PollingState = Stage_Checking_App_Firmware;
                }

                if (Monitor.Info.reg.bit.Boot)
                {
                    Monitor.PollingState = Stage_Checking_Module_Firmware;
                }
            }
            else
                Monitor.PollingState = Stage_Wait_PortData;
            return Monitor.PollingState;

        case Stage_Checking_App_Firmware:
            return Stage_Checking_App_Firmware;

        case Stage_Checking_Module_Firmware:
            return Stage_Checking_Module_Firmware;

        case Stage_Wait_PortData:
            if (sys_time >= Monitor.jump_time)
            {
                Monitor.PollingState = Stage_ReadyToJump;
                SrvUpgrade_Collect_Info("[Jump Preparetion]\r\n");
                SrvUpgrade_Collect_Info("\tDisabling Irq\r\n");
            }
            else
            {
                /* check stream */
                if (Monitor.buf_size)
                    /* receive data from port */
                    Monitor.PollingState = Stage_Processing_PortData;
            }
            return Stage_Wait_PortData;

        case Stage_Processing_PortData:
            SrvUpgrade_PortProcPolling(sys_time);
            return Monitor.PollingState;

        case Stage_ReadyToJump:
            return Stage_ReadyToJump;

        case Stage_JumpError: return Stage_JumpError;
        default: return Stage_Unknow;
    }
}

static void SrvUpgrade_Parse(uint8_t *p_buf, uint16_t len)
{
    if (Monitor.init_state && p_buf && len)
    {
        if (!Monitor.buf_accessing)
        {
            Monitor.rec_time = SrvOsCommon.get_os_ms();

            if ((Monitor.buf_size + len) <= FIRMWARE_MAX_READ_SIZE)
            {
                memcpy(Monitor.buf + Monitor.buf_size, p_buf, len);
                Monitor.buf_size += len;
            }
        }
    }
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
