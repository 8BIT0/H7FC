#ifndef __SRV_UPGRADE_H
#define __SRV_UPGRADE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "Srv_FileAdapter.h"

#define Max_App_Num 32

typedef enum
{
    Stage_Init = 0,
    Stage_Reboot,
    Stage_UpgradeInfo_Error,
    Stage_Adapter_Error,
    Stage_FileInfo_Error,
    Stage_Process_PortData,
    Stage_Upgrade_Error,
    Stage_PortData_Error,
    Stage_TimeOut,
    Stage_ReadyToJump,
    Stage_Upgrade_Finish,
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
    ProtProc_Finish,
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
typedef union
{
    uint8_t val;

    struct
    {
        uint8_t App    : 1;
        uint8_t res    : 7;
    } bit;
} SrvUpgrade_CTLReg_TypeDef;

typedef struct
{
    SrvUpgrade_CTLReg_TypeDef CTLReg;
    FileInfo_TypeDef AF_Info;   /* app firmware info */
} SrvUpgradeInfo_TypeDef;
#pragma pack()

typedef struct
{
    bool (*init)(void);
    SrvUpgrade_Stage_List (*polling)(uint32_t sys_time, SrvFileAdapter_Send_Func send);
    void (*set_fileinfo)(const FileInfo_TypeDef info);
    uint16_t (*get_log)(uint8_t *p_info, uint16_t len);
    void (*clear_log)(void);
    bool (*push_data)(uint32_t sys_time, uint8_t *p_buf, uint16_t len);
} SrvUpgrade_TypeDef;

extern const uint8_t HWVer[3];
extern const uint8_t AppVer[3];
extern SrvUpgrade_TypeDef SrvUpgrade;

#ifdef __cplusplus
}
#endif

#endif
