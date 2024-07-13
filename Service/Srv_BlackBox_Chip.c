#include "Srv_BlackBox_Def.h"
#include "../System/storage/Storage.h"

typedef struct
{
    bool init;
    bool enable;
    bool accessing;
    uint8_t *p_buf;
    uint32_t info_addr;
    uint32_t info_size;
    uint32_t log_cnt;
    uint32_t log_size;
    uint16_t log_unit;
    uint32_t rom_addr;
    uint32_t rom_size;
} Chip_BlackBox_Monitor_TypeDef;

#pragma pack(1)
typedef struct
{
    uint32_t cnt;
    uint32_t size;
} Chip_BlackBox_Info_TypeDef;
#pragma pack()

/* internal function */
static  Chip_BlackBox_Monitor_TypeDef ChipBlackBox_Monitor = {
    .init = false,
    .enable = false,
    .accessing = false,
    .info_addr = 0,
    .info_size = 0,
    .log_cnt = 0,
    .log_size = 0,
    .log_unit = 0,
    .rom_addr = 0,
    .rom_size = 0,
};

/* external function */
static uint32_t SrvChip_BlackBox_Init(SrvBlackBox_Log_Callback callback);
static bool SrvChip_BlackBox_PushData(uint8_t *p_data, uint16_t len);
static bool SrvChip_BlackBox_Read(uint32_t addr_offset, uint8_t *p_data, uint16_t len);
static bool SrvChip_BlackBox_GetInfo(uint32_t *cnt, uint32_t *size, bool *enable_state);
static bool SrvChip_BlackBox_Enable(void);
static bool SrvChip_BlackBox_Disable(void);

SrvBlackBox_TypeDef SrvChip_BlackBox = {
    .init = SrvChip_BlackBox_Init,
    .push = SrvChip_BlackBox_PushData,
    .enable = SrvChip_BlackBox_Enable,
    .disable = SrvChip_BlackBox_Disable,
    .read = SrvChip_BlackBox_Read,
    .get_info =SrvChip_BlackBox_GetInfo,
};

/* return log unit size */
static uint32_t SrvChip_BlackBox_Init(SrvBlackBox_Log_Callback callback)
{
    Storage_ExtFLashDevObj_TypeDef dev;
    memset(&dev, 0, sizeof(Storage_ExtFLashDevObj_TypeDef));

    ChipBlackBox_Monitor.init = false;
    /* get external storage flash info */
    if (Storage.get_dev_info(&dev) == Storage_Error_None)
    {
        if ((dev.total_size + dev.start_addr) > BlackBox_Storage_Start_Addr)
        {
            ChipBlackBox_Monitor.info_addr = BlackBox_Storage_Start_Addr;
            ChipBlackBox_Monitor.info_size = dev.sector_size;
            ChipBlackBox_Monitor.rom_addr = BlackBox_Storage_Start_Addr + dev.sector_size;
            ChipBlackBox_Monitor.rom_size = (dev.total_size + dev.start_addr) - (BlackBox_Storage_Start_Addr + dev.sector_size);
            ChipBlackBox_Monitor.log_unit = dev.sector_size;

            if (ChipBlackBox_Monitor.log_unit)
            {
                ChipBlackBox_Monitor.p_buf = SrvOsCommon.malloc(ChipBlackBox_Monitor.log_unit);
                if (ChipBlackBox_Monitor.p_buf == NULL)
                {
                    SrvOsCommon.free(ChipBlackBox_Monitor.p_buf);
                    return 0;
                }
            }
            else
                return 0;

            ChipBlackBox_Monitor.init = true;
        }
    }

    return ChipBlackBox_Monitor.log_unit;
}

static bool SrvChip_BlackBox_PushData(uint8_t *p_data, uint16_t len)
{
    uint32_t update_addr = 0;
    Chip_BlackBox_Info_TypeDef BlackBoxDataInfo;

    memset(&BlackBoxDataInfo, 0, sizeof(Chip_BlackBox_Info_TypeDef));
    if (ChipBlackBox_Monitor.init && ChipBlackBox_Monitor.enable)
    {
        if ((ChipBlackBox_Monitor.rom_size - ChipBlackBox_Monitor.log_size) < len)
        {
            /* disable log */
            ChipBlackBox_Monitor.enable = false;
            return false;
        }

        ChipBlackBox_Monitor.accessing = true;
        update_addr = ChipBlackBox_Monitor.rom_addr + ChipBlackBox_Monitor.log_size;

        /* write data to balckbox data section */
        if (!Storage.write_section(update_addr, p_data, len))
            return false;

        ChipBlackBox_Monitor.log_cnt ++;
        ChipBlackBox_Monitor.log_size += ChipBlackBox_Monitor.log_unit;

        BlackBoxDataInfo.cnt = ChipBlackBox_Monitor.log_cnt;
        BlackBoxDataInfo.size = ChipBlackBox_Monitor.log_size;
        memcpy(ChipBlackBox_Monitor.p_buf, &BlackBoxDataInfo, sizeof(Chip_BlackBox_Info_TypeDef));

        /* update info section */
        if (!Storage.write_section(ChipBlackBox_Monitor.info_addr, ChipBlackBox_Monitor.p_buf, ChipBlackBox_Monitor.info_size))
            return false;

        ChipBlackBox_Monitor.accessing = false;
    
        return true;
    }

    return false;
}

/* still in developping */
static bool SrvChip_BlackBox_Read(uint32_t addr_offset, uint8_t *p_data, uint16_t len)
{
    Chip_BlackBox_Info_TypeDef info;
    uint8_t read_cnt = 1;
    uint32_t read_addr = 0;

    memset(&info, 0, sizeof(Chip_BlackBox_Info_TypeDef));
    if (ChipBlackBox_Monitor.init && p_data && len && \
        ((addr_offset % ChipBlackBox_Monitor.log_unit) == 0) && \
        ((len % ChipBlackBox_Monitor.log_unit) == 0) && \
        (addr_offset < ChipBlackBox_Monitor.rom_size))
    {
        read_addr = ChipBlackBox_Monitor.rom_addr + addr_offset;

        /* get info first */
        if (!Storage.read_section(ChipBlackBox_Monitor.info_addr, ChipBlackBox_Monitor.p_buf, ChipBlackBox_Monitor.info_size))
            return false;

        memcpy(&info, ChipBlackBox_Monitor.p_buf, sizeof(Chip_BlackBox_Info_TypeDef));
        if ((info.cnt == 0) || (info.size == 0))
            return false;

        /* get balckbox data */
        if (!Storage.read_section(read_addr, p_data, len))
            return false;

        return true;
    }

    return false;
}

static bool SrvChip_BlackBox_Enable(void)
{
    if (ChipBlackBox_Monitor.init)
    {
        ChipBlackBox_Monitor.enable = true;
        return true;
    }

    return false;
}

static bool SrvChip_BlackBox_Disable(void)
{
    if (ChipBlackBox_Monitor.init)
    {
        ChipBlackBox_Monitor.enable = false;
        ChipBlackBox_Monitor.log_cnt = 0;
        ChipBlackBox_Monitor.log_size = 0;
        return true;
    }

    return false;
}

static bool SrvChip_BlackBox_GetInfo(uint32_t *cnt, uint32_t *size, bool *enable_state)
{
    Chip_BlackBox_Info_TypeDef info;

    memset(&info, 0, sizeof(Chip_BlackBox_Info_TypeDef));
    if (ChipBlackBox_Monitor.init && cnt && size && enable_state)
    {
        if (!Storage.read_section(ChipBlackBox_Monitor.info_addr, ChipBlackBox_Monitor.p_buf, ChipBlackBox_Monitor.info_size))
            return false;

        memcpy(&info, ChipBlackBox_Monitor.p_buf, sizeof(Chip_BlackBox_Info_TypeDef));
        *cnt = info.cnt;
        *size = info.size;
        *enable_state = ChipBlackBox_Monitor.enable;
    }

    return false;
}

