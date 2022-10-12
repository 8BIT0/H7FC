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

typedef union
{
    struct
    {
        uint16_t IMU_Sec : 1;
        uint16_t Res_Sec : 15;
    } _sec;

    uint16_t reg_val;
} LogData_Reg_TypeDef;

#pragma pack(1)
typedef struct
{
    uint8_t header;
    uint8_t type;
    uint8_t size;
} LogData_Header_TypeDef;

struct LogCache_TypeDef
{
    uint8_t *p_buf;
    int16_t rem_size;  /* remain size */
    int16_t ocp_size;  /* occupy size */
    uint16_t tot_size; /* total  size */

    struct LogCache_TypeDef *nxt;
};
#pragma pack()

void TaskLog_Init(void);
void TaskLog_Core(Task_Handle hdl);

#endif
