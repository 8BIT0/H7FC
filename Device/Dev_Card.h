#ifndef __DEV_CARD_H
#define __DEV_CARD_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "Bsp_SDMMC.h"

typedef enum
{
    DevCard_No_Error = 0,
    DevCard_Obj_Error,
    DevCard_Bus_Error,
    DevCard_Info_Error,
} DevCard_Error_List;

#pragma pack(1)
typedef struct
{
    bool valid;

    uint32_t CardType;
    uint32_t CardVersion;
    uint32_t Class;
    uint32_t RelCardAdd;
    uint32_t BlockNbr;
    uint32_t BlockSize;
    uint32_t LogBlockNbr;
    uint32_t LogBlockSize;
    uint32_t CardSpeed;

    uint32_t UsdBlockNbr;
    uint32_t RmnBlockNbr;
    uint16_t RmnByteInCurBlock;
} DevCard_Info_TypeDef;
#pragma pack()

typedef struct
{
    BspSDMMC_Obj_TypeDef SDMMC_Obj;
    DevCard_Error_List error_code;
    DevCard_Info_TypeDef info;
} DevCard_Obj_TypeDef;

typedef struct
{
    DevCard_Error_List (*Init)(DevCard_Obj_TypeDef *Obj);
    bool (*Insert)(DevCard_Obj_TypeDef *Obj);
    bool (*GetState)(DevCard_Obj_TypeDef *Obj);
    DevCard_Error_List (*Get_ErrorCode)(DevCard_Obj_TypeDef *Obj);
    DevCard_Info_TypeDef (*Get_Info)(DevCard_Obj_TypeDef *Obj);
    bool (*read)(DevCard_Obj_TypeDef *Instance, uint32_t block, uint8_t *p_data, uint16_t data_size, uint16_t block_num);
    bool (*write)(DevCard_Obj_TypeDef *Instance, uint32_t block, uint8_t *p_data, uint32_t data_sise, uint16_t block_num);
} DevCard_TypeDef;

extern DevCard_TypeDef DevCard;

#endif
