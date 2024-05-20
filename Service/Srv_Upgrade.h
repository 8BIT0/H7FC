#ifndef __SRV_UPGRADE_H
#define __SRV_UPGRADE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "Srv_FileAdapter.h"

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
    Stage_Checking_App_Firmware,
    Stage_Checking_Boot_Firmware,
    Stage_FirmwareData_Error,
    Stage_Adapter_Error,
    Stage_Process_PortData,
    Stage_PortData_Error,
    Stage_Commu_TimeOut,
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
    PortProc_Deal_Pack,
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
    Firmware_FileType_List File_Type;
    Adapter_ProtoType_List Adapter_Type;
    uint8_t SW_Ver[3];
    uint8_t HW_Ver[3];
    uint32_t File_Size;
    uint16_t Pack_Size;
} Upgrade_FileInfo_TypeDef;

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
    SrvUpgrade_Stage_List (*polling)(uint32_t sys_time, SrvFileAdapter_Send_Func send);
    void (*set_fileinfo)(const Upgrade_FileInfo_TypeDef info);
    void (*jump)(void);
    uint16_t (*get_log)(uint8_t *p_info, uint16_t len);
    void (*clear_log)(void);
    bool (*push_data)(uint32_t sys_time, uint8_t *p_buf, uint16_t len);
} SrvUpgrade_TypeDef;

extern SrvUpgrade_TypeDef SrvUpgrade;

#endif
