#ifndef __DEV_ICM20602_H
#define __DEV_ICM20602_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include "imu_data.h"

typedef enum
{
    ICM20602_SampleRate_8K = 0,
    ICM20602_SampleRate_4K,
    ICM20602_SampleRate_2K,
    ICM20602_SampleRate_1K,
} ICM20602_SampleRate_List;

typedef enum
{
    ICM20602_No_Error = 0,
} ICM20602_Error_List;

typedef struct
{
    bool drdy;
    ICM20602_SampleRate_List rate;
} DevICM20602Obj_TypeDef;

typedef struct
{
    void (*set_rate)(DevICM20602Obj_TypeDef *Obj, ICM20602_SampleRate_List rate);
    bool (*init)(DevICM20602Obj_TypeDef *Obj);
} DevICM20602_TypeDef;

#endif
