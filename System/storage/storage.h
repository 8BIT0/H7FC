#ifndef __STORAGE_H
#define __STORAGE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "system_cfg.h"

typedef enum
{
    OCS_System_Param = 1,
    OCS_Applic_Param,
    OCS_Custom_Param,
} OnChipStorage_Section_List;

#if (EXTERNAL_STORAGE_MODULE == 1)
#define EXS_SECTION_INFO_SIZE 1024 * 1024

typedef enum
{
    EXS_Section_Info = 1,
    EXS_Storage_Data,
} ExternalStorage_Section_List;
#endif

bool Storage_Init(void);

#endif
