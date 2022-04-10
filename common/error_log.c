#include "error_log.h"
#include "mmu.h"

static uint32_t ErrorPriority_Compare(uint32_t l_addr, uint32_t r_addr)
{
    if (ErrorTreeDataToObj(l_addr)->code > ErrorTreeDataToObj(r_addr)->code)
        return l_addr;

    return r_addr;
}

Error_Handler ErrorTree_Create(char *name)
{
    ErrorTree_TypeDef *Error_Tmp = NULL;

    Error_Tmp = (ErrorTree_TypeDef *)MMU_Malloc(sizeof(ErrorTree_TypeDef));

    if (Error_Tmp == NULL)
        return NULL;

    Error_Tmp->name = name;
    Error_Tmp->limb_num = 0;
    Error_Tmp->link_node = NULL;
    Error_Tmp->tree_node = NULL;

    return (uint32_t)Error_Tmp;
}

bool Error_Register(Error_Handler hdl, Error_Obj_Typedef *Obj_List, uint16_t num)
{
    item_obj *linked_item = NULL;
    node_template *tree_node = NULL;

    for (uint16_t i = 0; i < num; i++)
    {
        tree_node = (node_template *)MMU_Malloc(sizeof(node_template));
        linked_item = (item_obj *)MMU_Malloc(sizeof(item_obj));

        if ((tree_node == NULL) || (linked_item == NULL))
            return false;

        Tree_Node_Init(tree_node, Obj_List[i].desc, &Obj_List[i]);
        List_ItemInit(linked_item, &Obj_List[i]);

        if (ErrorHandleToObj(hdl)->tree_node != NULL)
        {
            Tree_InsertNode(ErrorHandleToObj(hdl)->tree_node, tree_node, ErrorPriority_Compare);
        }
        else
            ErrorHandleToObj(hdl)->tree_node = tree_node;
    }

    ErrorHandleToObj(hdl)->limb_num = num;

    return true;
}

bool Error_Trigger(Error_Handler hdl, int16_t code)
{
    Error_Obj_Typedef *ErrorObj_Tmp;

    if (hdl == NULL)
        return false;

    ErrorObj_Tmp = Tree_Search(ErrorHandleToObj(hdl)->tree_node, );

    return true;
}

bool Error_Proc(Error_Handler hdl)
{
    if (hdl == 0)
        return false;

    return true;
}
