#ifndef __ERROR_LOG_H
#define __ERROR_LOG_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include "binary_tree.h"
#include "Srv_OsCommon.h"
#include "linked_list.h"

#define ERROR_DESC_BUFFSIZE 1024
#define ERROR_LOG_MALLOC(x) SrvOsCommon.malloc(x)
#define ERROR_LOG_FREE(x) SrvOsCommon.free(x)

#define ErrorHandleToObj(x) ((ErrorTree_TypeDef *)x)
#define ErrorTreeDataToObj(x) ((Error_Obj_Typedef *)x)

typedef int8_t (*error_port_callback)(uint8_t *p_data, uint16_t size);
typedef void (*error_proc_callback)(int16_t code, uint8_t *p_data, uint16_t size);

typedef uint32_t Error_Handler;

typedef enum
{
    Error_Out_Callback = 1,
    Error_Log_Callback,
} ErrorLog_Callback_Type_List;

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
        uint16_t len : 16;
    } section;
    uint32_t val;
} Error_Port_Reg;

typedef struct
{
    uint8_t *p_data;
    uint16_t size;
} ErrorStream_TypeDef;

typedef struct
{
    int16_t code;
    Error_Proc_List proc_type;
    bool triggered;
    char *desc;
    bool out;
    bool log;
    error_proc_callback prc_callback;
    ErrorStream_TypeDef prc_data_stream;
    item_obj *item;
} Error_Obj_Typedef;

typedef struct
{
    uint16_t reg_num;
    Tree_TypeDef *tree;
    list_obj *link_node;
} ErrorTree_TypeDef;
#pragma pack()

typedef struct
{
    Error_Handler (*create)(char *name);
    bool (*registe)(Error_Handler hdl, Error_Obj_Typedef *obj, uint16_t num);
    bool (*trigger)(Error_Handler hdl, int16_t code, uint8_t *p_arg, uint16_t size);
    bool (*proc)(Error_Handler hdl);
    void (*set_callback)(ErrorLog_Callback_Type_List type, error_port_callback callback);
    uint32_t (*add_desc)(const char *str, ...);
} ErrorLog_TypeDef;

extern ErrorLog_TypeDef ErrorLog;

#endif
