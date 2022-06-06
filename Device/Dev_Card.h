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
    DevCard_Bus_Error,
    DevCard_Info_Error,
} DevCard_Error_List;

#pragma pack(1)
typedef struct
{
    BspSDMMC_Obj_TypeDef SDMMC_Obj;
} DevCard_Obj_TypeDef;
#pragma pack()

typedef struct
{
    DevCard_Error_List (*Init)(DevCard_Obj_TypeDef *Obj);
    bool (*Insert)(DevCard_Obj_TypeDef *Obj);
    bool (*GetState)(DevCard_Obj_TypeDef *Obj);
} DevCard_TypeDef;

extern DevCard_TypeDef DevCard;

#endif
