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

int8_t SrvIMU_Init(void);
int8_t SrvIMU_GetPri_InitError(void);

#endif
