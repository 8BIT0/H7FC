#ifndef __ERROR_LOG_H
#define __ERROR_LOG_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "binary_tree.h"
#include "linked_list.h"

#define ErrorHandleToObj(x) ((ErrorTree_TypeDef *)x)
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
    int16_t code;
    Error_Proc_List proc_type;
    bool triggered;
    char *desc;
    error_proc_callback prc_callback;
    error_proc_callback out_callback;
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

#endif
