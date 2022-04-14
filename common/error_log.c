#include "error_log.h"
#include "mmu.h"

/* return low priority one */
static uint32_t Error_InsertPriority_Compare(uint32_t l_addr, uint32_t r_addr)
{
    if (ErrorTreeDataToObj(l_addr)->code > ErrorTreeDataToObj(r_addr)->code)
        return r_addr;

    if (ErrorTreeDataToObj(l_addr)->code < ErrorTreeDataToObj(r_addr)->code)
        return l_addr;

    return 0;
}

Error_Handler ErrorTree_Create(char *name)
{
    volatile ErrorTree_TypeDef *Error_Tmp = NULL;

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

        Obj_List[i].triggered = false;

        if (ErrorHandleToObj(hdl)->tree_node != NULL)
            Tree_InsertNode(ErrorHandleToObj(hdl)->tree_node, tree_node, Error_InsertPriority_Compare);
        else
            ErrorHandleToObj(hdl)->tree_node = tree_node;
    }

    ErrorHandleToObj(hdl)->tree_node = Tree_ReSetRoot(ErrorHandleToObj(hdl)->tree_node);

    ErrorHandleToObj(hdl)->limb_num = num;
    return true;
}

static uint32_t Error_TriggerCompareCallback(node_template *node, void *code_addr)
{
    Error_Obj_Typedef *Obj = (Error_Obj_Typedef *)(node->data_ptr);
    int16_t error_code = *((int16_t *)code_addr);

    if (Obj == NULL)
        return 0;

    if (Obj->code == error_code)
        return 1;

    if (Obj->code > error_code)
        return (uint32_t)(node->L_Node);

    if (Obj->code < error_code)
        return (uint32_t)(node->R_Node);
}

bool Error_Trigger(Error_Handler hdl, int16_t code)
{
    uint32_t ErrorNodeObj_Tmp;
    item_obj *node_item = NULL;
    int16_t code_tmp = code;
    Error_Obj_Typedef *errorobj_tmp = NULL;

    if (hdl == NULL)
        return false;

    ErrorNodeObj_Tmp = Tree_Search(ErrorHandleToObj(hdl)->tree_node, (uint32_t)&code_tmp, NULL, Error_TriggerCompareCallback);

    if (ErrorNodeObj_Tmp != ERROR_MATCH)
    {
        errorobj_tmp = (Error_Obj_Typedef *)(((node_template *)ErrorNodeObj_Tmp)->data_ptr);

        if (errorobj_tmp->proc_type != Error_Proc_Immd)
        {
            node_item = errorobj_tmp->item;

            if (ErrorHandleToObj(hdl)->link_node == NULL)
            {
                List_Init(ErrorHandleToObj(hdl)->link_node, node_item, by_order, Error_InsertPriority_Compare);
            }
            else
            {
                List_Insert_Item(ErrorHandleToObj(hdl)->link_node, node_item);
            }
        }
        else
        {
            /* trigger error process immediately */
            // if ()
        }

        return true;
    }

    return false;
}

bool Error_Proc(Error_Handler hdl)
{
    if (hdl == 0)
        return false;

    return true;
}
