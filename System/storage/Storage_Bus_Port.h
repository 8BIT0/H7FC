#ifndef __STORAGE_BUS_PORT_H
#define __STORAGE_BUS_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef void* (*StorageBus_Malloc_Callback)(uint32_t size);
typedef void (*StorageBus_Free_Callback)(void *ptr);

typedef struct
{
    void* (*init)(StorageBus_Malloc_Callback p_malloc, StorageBus_Free_Callback p_free);
    bool (*cs_ctl)(bool en);
    uint16_t (*bus_tx)(uint8_t *p_data, uint16_t len, uint32_t time_out);
    uint16_t (*bus_rx)(uint8_t *p_data, uint16_t len, uint32_t time_out);
    uint16_t (*bus_trans)(uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t time_out);
} StorageBusApi_TypeDef;

extern StorageBusApi_TypeDef StoragePort_Api;

#endif