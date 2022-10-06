#ifndef __TASK_LOG_H
#define __TASK_LOG_H

#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stddef.h>
#include "scheduler.h"

#define LOG_FUNCTIONAL_BYTE_SIZE 3

#define LOG_HEADER 0xBA
#define LOG_DATATYPE_IMU 0x00

#pragma pack(1)
typedef struct
{
    uint8_t header;
    uint8_t type;
    uint8_t size;
}LogData_Header_TypeDef;

typedef struct
{
    uint8_t *p_buf;
    uint16_t rem_size; /* remain size */
    uint16_t ocp_size; /* occupy size */

    uint8_t page;
    uint8_t id;
}LogCache_TypeDef;
#pragma pack()

typedef struct
{
    LogCache_TypeDef *cache_obj;
    LogCache_TypeDef *inuse_cache_page;
    LogCache_TypeDef *store_page;

    uint16_t single_log_size;
    uint16_t single_log_offset;

    uint8_t inuse_page_id;
    uint8_t cache_page_sum;
}Log_Monitor_TypeDef;

void TaskLog_Init(void);
void TaskLog_Core(Task_Handle hdl);

#endif
