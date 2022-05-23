#ifndef __ERROR_LOG_H
#define __ERROR_LOG_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "binary_tree.h"
#include "linked_list.h"

#define ERROR_DESC_BUFFSIZE 1024

#define ErrorHandleToObj(x) ((ErrorTree_TypeDef *)x)
#define ErrorTreeDataToObj(x) ((Error_Obj_Typedef *)x)

typedef void (*error_port_callback)(uint8_t *p_data, uint16_t size);
typedef void (*error_proc_callback)(int16_t code, uint8_t *p_data, uint16_t size);

typedef uint32_t Error_Handler;

typedef enum
{
    Error_Proc_Immd = 0,
    Error_Proc_Next,
    Error_Proc_Ignore,
} Error_Proc_List;

typedef enum
{
    Error_OutFree = 0,
    Error_OutFailed,
    Error_OutDone,
    Error_OutBusy,
    Error_Out_NullCallback,
} Error_OutState_List;

typedef enum
{
    Error_LogFree = 0,
    Error_LogFailed,
    Error_LogDone,
    Error_LogBusy,
    Error_Log_NullCallback,
} Error_LogState_List;

#pragma pack(1)
typedef union
{
    struct
    {
        uint8_t out_reg : 4;
        uint8_t log_reg : 4;
    } section;
    uint8_t val;
} Error_Port_Reg;

typedef struct
{
    int16_t code;
    Error_Proc_List proc_type;
    bool triggered;
    char *desc;
    bool out;
    bool log;
    error_proc_callback prc_callback;
    item_obj *item;
} Error_Obj_Typedef;

typedef struct
{
    uint16_t reg_num;
    Tree_TypeDef *tree;
    list_obj *link_node;
} ErrorTree_TypeDef;
#pragma pack()

Error_Handler ErrorTree_Create(char *name);
bool Error_Register(Error_Handler hdl, Error_Obj_Typedef *obj, uint16_t num);
bool Error_Trigger(Error_Handler hdl, int16_t code, uint8_t *p_arg, uint16_t size);
bool Error_Proc(Error_Handler hdl);
void Error_Set_OutCallback(error_port_callback out);
void Error_Log_OutCallback(error_port_callback log);

#endif
