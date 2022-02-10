#ifndef __ERROR_PROC_H
#define __ERROR_PROC_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef bool (*Error_Proc_Callback)(uint32_t *arg, uint16_t size);

#pragma pack(1)
typedef struct
{
    char *desc;
    uint8_t bit_code;
    uint16_t priority;
    Error_Proc_Callback callback;
} ErrorObj_TypeDef;
#pragma pack()

void Error_Regist();
void Error_Insert();
void Error_Polling(void);
void Error_CodeOut(void);
void Error_Assert(uint32_t arg);
uint32_t Error_GetTriggerCnt(void);

#endif
