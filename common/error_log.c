#include "error_log.h"
#include "mmu.h"

Error_Handler Error_Register(char *ErrorTree_Name, Error_Obj_Typedef *Obj_List, uint16_t num)
{
    ErrorTree_TypeDef *ErrorTree_Tmp = NULL;
    uint16_t highest_ID = num;

    ErrorTree_Tmp = (ErrorTree_TypeDef *)MMU_Malloc(sizeof(ErrorTree_TypeDef));

    if (ErrorTree_Tmp == NULL)
        return NULL;

    ErrorTree_Tmp->sum = num;
    ErrorTree_Tmp->Tree_Name = ErrorTree_Name;

    /* init error tree root node */
    Tree_Node_Init(ErrorTree_Tmp->root, ErrorTree_Name, NULL);

    for (uint16_t i = 0; i < num; i++)
    {
    }

    return (uint32_t)ErrorTree_Tmp;
}

bool Error_Proc(Error_Handler hdl)
{
    if (hdl == 0)
        return false;

    return true;
}
