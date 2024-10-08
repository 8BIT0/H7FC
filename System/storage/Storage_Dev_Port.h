#ifndef __STORAGE_DEV_PORT_H
#define __STORAGE_DEV_PORT_H

#include <stdint.h>
#include <string.h>
#include <stdint.h>

typedef struct
{
    bool (*set)();
    bool (*init)();
} StorageDevApi_TypeDef;

#endif
