#ifndef __SRVMPU_SAMPLE_H
#define __SRVMPU_SAMPLE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "runtime.h"
#include "imu_data.h"

typedef enum
{
    SrvIMU_Sample_OverRange,
    SrvIMU_Sample_Blunt,
} SrvIMU_ErrorCode_List;

typedef struct
{
    bool (*init)(void);
    SrvIMU_ErrorCode_List (*sample)(void);
} SrvIMU_TypeDef;

bool SrvIMU_Init(void);

#endif
