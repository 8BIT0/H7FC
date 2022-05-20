#include "error_log.h"
#include "mmu.h"

static data_handle Error_InsertPriority_Compare(data_handle l_addr, data_handle r_addr)
{
    volatile int16_t l_code = 0;
    volatile int16_t r_code = 0;

    if ((l_addr == 0) && (r_addr == 0) && (l_addr != r_addr))
        return 0;

    l_code = ErrorTreeDataToObj(l_addr)->code;
    r_code = ErrorTreeDataToObj(r_addr)->code;

    if (ErrorTreeDataToObj(l_addr)->code > ErrorTreeDataToObj(r_addr)->code)
        return r_addr;

    if (ErrorTreeDataToObj(l_addr)->code < ErrorTreeDataToObj(r_addr)->code)
        return l_addr;

    return 0;
}

static uint8_t Error_Search(data_handle l_addr, data_handle r_addr)
{
    volatile int16_t node_code = ErrorTreeDataToObj(l_addr)->code;
    volatile int16_t code = *((int16_t *)((uint32_t)r_addr));

    if (node_code > code)
        return Tree_Search_L;

    if (node_code < code)
        return Tree_Search_R;

    if (node_code == code)
        return Tree_Search_D;
}

Error_Handler ErrorTree_Create(char *name)
{
    volatile ErrorTree_TypeDef *Error_Tmp = NULL;

    Error_Tmp = (ErrorTree_TypeDef *)MMU_Malloc(sizeof(ErrorTree_TypeDef));

    if (Error_Tmp == NULL)
        return NULL;

    Error_Tmp->tree = BalanceTree.Create(name, Error_InsertPriority_Compare, Error_Search, Error_InsertPriority_Compare);

    if (Error_Tmp == NULL)
        return 0;

    Error_Tmp->reg_num = 0;
    Error_Tmp->link_node = NULL;

    return (Error_Handler)Error_Tmp;
}

bool Error_Register(Error_Handler hdl, Error_Obj_Typedef *Obj_List, uint16_t num)
{
    item_obj *linked_item = NULL;

    for (uint16_t i = 0; i < num; i++)
    {
        BalanceTree.Insert(ErrorHandleToObj(hdl)->tree, Obj_List[i].desc, (data_handle)(&Obj_List[i]));
    }

    ErrorHandleToObj(hdl)->reg_num = num;
    return true;
}

/* still in half way */
bool Error_Trigger(Error_Handler hdl, int16_t code, uint8_t *p_arg, uint16_t size)
{
    int16_t code_tmp = code;
    data_handle match_data = 0;

    if (hdl == 0)
        return false;

    TreeNode_Handle search_handle = BalanceTree.Search(ErrorHandleToObj(hdl)->tree, (data_handle)&code_tmp);

    /* find target node */
    if (search_handle)
    {
        if (TreeNodeHandleToObj(search_handle)->data)
        {
            if (ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->proc_type == Error_Proc_Immd)
            {
                /* trigger process callback */
                if (ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->prc_callback)
                    ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->prc_callback(NULL, 0);
            }
            else if (ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->proc_type == Error_Proc_Next)
            {
                /* add error into linked list */
            }
            else if (ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->proc_type == Error_Proc_Ignore)
            {
                /* reserve */
            }

            if (ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->out_callback)
            {
                /* out put error */
                ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->out_callback(p_arg, size);
            }
        }
    }

    return true;
}

/* still in half way */
bool Error_Proc(Error_Handler hdl)
{
    if (hdl == 0)
        return false;

    if (ErrorHandleToObj(hdl)->link_node)
    {
    }

    return true;
}
