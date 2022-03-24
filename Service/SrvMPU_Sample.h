#ifndef __SRVMPU_SAMPLE_H
#define __SRVMPU_SAMPLE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "runtime.h"

typedef enum
{
    SrvIMU_Sample_OverRange,
    SrvIMU_Sample_Blunt,
} SrvIMU_ErrorCode_List;

#pragma pack(1)
typedef struct
{
    SYSTEM_RunTime time;

    int16_t int_gyr[];

} SrvIMU_Data_TypeDef;
#pragma pack()

typedef struct
{
    bool (*init)(void);
    SrvIMU_ErrorCode_List (*sample)(void);
} SrvIMU_TypeDef;

#endif
