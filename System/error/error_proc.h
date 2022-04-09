#ifndef __ERROR_PROC_H
#define __ERROR_PROC_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "linked_list.h"
#include "binary_tree.h"

typedef bool (*Error_Proc_Callback)(uint32_t *arg, uint16_t size);

typedef enum
{
    Error_Proc_Immd,
    Error_Proc_Poll,
} ErrorProc_Type_List;

#pragma pack(1)
typedef struct
{
    char *desc;
    ErrorProc_Type_List type;
    uint8_t code;
    Error_Proc_Callback callback;
} ErrorObj_TypeDef;

typedef struct
{
    char *node_name;
    uint8_t limb_num;
    node_template *root;
} ErrorTree_TypeDef;
#pragma pack()

void Error_Regist();
void Error_Insert();
void Error_Polling(void);
void Error_CodeOut(void);
void Error_Assert(uint32_t arg);
uint32_t Error_GetTriggerCnt(void);

#endif
