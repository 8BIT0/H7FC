#ifndef __ERROR_LOG_H
#define __ERROR_LOG_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "binary_tree.h"

#define ErrorHanleToObj(x) ((ErrorTree_TypeDef *)x)
#define ErrorTreeDataToObj(x) ((Error_Obj_Typedef *)x)

typedef void (*error_proc_callback)(uint8_t *p_data, uint16_t size);

typedef uint32_t Error_Handler;

typedef enum
{
    Error_Proc_Immd = 0,
    Error_Proc_Next,
    Error_Proc_Ignore,
} Error_Proc_List;

#pragma pack(1)
typedef struct
{
    uint16_t id;
    int16_t code;
    Error_Proc_List proc_type;
    bool triggered;
    bool out;
    char *desc;
    error_proc_callback callback;
} Error_Obj_Typedef;

typedef struct
{
    char *name;
    uint16_t limb_num;
    node_template *root;
} ErrorTree_TypeDef;
#pragma pack()

Error_Handler Error_Register(char *ErrorTree_Name, Error_Obj_Typedef *Obj_List, uint16_t num);
bool Error_Trigger(Error_Handler hdl, int16_t code);
bool Error_Proc(Error_Handler hdl);

#endif
