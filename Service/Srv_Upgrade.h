#ifndef __SRV_UPGRADE_H
#define __SRV_UPGRADE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "Srv_FileAdapter.h"

#define MAX_BOOTLOADER_FRIMWARE_SIZE (64 * 1024)
#define Max_App_Num 32

typedef enum
{
    On_Boot = 0,
    On_App,
} SrvUpgrade_CodeStage_List;

typedef union
{
    uint8_t val;

    struct
    {
        uint8_t Boot    : 1;
        uint8_t App     : 1;
        uint8_t Module  : 1;
    } bit;
} UpgradeReg_TypeDef;

typedef enum
{
    FileType_None = 0,
    FileType_APP,
    FileType_Boot,
    FileType_Module,
} Firmware_FileType_List;

typedef enum
{
    Stage_Init = 0,
    Stage_Wait_PortData,
    Stage_Checking_App_Firmware,
    Stage_Checking_Module_Firmware,
    Stage_FirmwareData_Error,
    Stage_Processing_PortData,
    Stage_PortData_Error,
    Stage_Proto_TimeOut,
    Stage_App_Upgrading,
    Stage_Module_Upgrading,
    Stage_ReadyToJump,
    Stage_JumpToTarget,
    Stage_JumpError,
    Stage_Unknow,
} SrvUpgrade_Stage_List;

typedef enum
{
    Decode_None = 0,
    Decode_Pack_Incompelet,
    Decode_Failed,
    Decode_Successed,
} PortData_DecodeState_List;

typedef enum
{
    PortProc_None = 0,
    PortProc_Check_FileAdapter_EnableSig,
    PortProc_Check_FirmwareInfo,
    /* receive firmware pack stage */
    PortProc_Deal_Error,
    PortProc_Deal_TimeOut,
    PortProc_InValid_Data,
    PortProc_Unknown,
} SrvUpgrade_PortDataProc_List;

typedef struct
{
    SrvUpgrade_Stage_List stage;
    bool All_Port_Disabled;
} SrvUpgrade_State_TypeDef;

#pragma pack(1)
typedef struct
{
    Firmware_FileType_List file_type;
    Adapter_ProtoType_List adapter_frame;
    uint32_t byte_sum;
    uint16_t pack_sum;

    /* only used by app firmware */
    uint32_t jump_addr;
} Upgrade_FileInfo_TypeDef;

typedef struct
{
    uint8_t  type;
    uint8_t id;
    uint32_t store_addr;
    uint32_t size;
    uint8_t ver[3];
} FirmwareInfo_TypeDef;

typedef struct
{
    UpgradeReg_TypeDef reg;
    uint32_t app_num;
    uint32_t app_addr_list[Max_App_Num];
    uint32_t jump_addr;
} UpgradeInfo_TypeDef;
#pragma pack()

typedef struct
{
    bool (*init)(SrvUpgrade_CodeStage_List stage, uint32_t window_size);
    SrvUpgrade_Stage_List (*polling)(void);
    void (*jump)(void);
    uint16_t (*get_log)(uint8_t *p_info, uint16_t len);
    void (*clear_log)(void);
} SrvUpgrade_TypeDef;

extern SrvUpgrade_TypeDef SrvUpgrade;

#endif
