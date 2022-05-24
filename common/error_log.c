#include "error_log.h"
#include "queue.h"
#include "mmu.h"

/*
error data in queue
--------------------------------------------------------------------------------------------
| out state | log state | info size |                    error describe                    |
--------------------------------------------------------------------------------------------
*/

/* internal vriable */
static bool ErrorQueue_Init = false;
static bool ErrorQueue_CreateState = false;
static QueueObj_TypeDef ErrorQueue;
static Error_OutState_List ErrorOut_State = Error_OutFree;
static Error_LogState_List ErrorLog_State = Error_LogFree;
static error_port_callback out_callback = NULL;
static error_port_callback log_callback = NULL;

/* internal function */
static bool Error_Out(void);
static bool Error_Log(void);

/* external function */
static Error_Handler ErrorTree_Create(char *name);
static bool Error_Register(Error_Handler hdl, Error_Obj_Typedef *obj, uint16_t num);
static bool Error_Trigger(Error_Handler hdl, int16_t code, uint8_t *p_arg, uint16_t size);
static bool Error_Proc(Error_Handler hdl);
static void Error_Set_Callback(ErrorLog_Callback_Type_List type, error_port_callback callback);

ErrorLog_TypeDef ErrorLog = {
    .create = ErrorTree_Create,
    .proc = Error_Proc,
    .registe = Error_Register,
    .trigger = Error_Trigger,
    .set_callback = Error_Set_Callback,
};

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

static Error_Handler ErrorTree_Create(char *name)
{
    volatile ErrorTree_TypeDef *Error_Tmp = NULL;

    if (!ErrorQueue_Init)
    {
        ErrorQueue_CreateState = Queue.create(&ErrorQueue, "ErrorQueue", ERROR_DESC_BUFFSIZE);
        ErrorQueue_Init = true;
    }

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

static bool Error_Register(Error_Handler hdl, Error_Obj_Typedef *Obj_List, uint16_t num)
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
static bool Error_Trigger(Error_Handler hdl, int16_t code, uint8_t *p_arg, uint16_t size)
{
    int16_t code_tmp = code;
    data_handle match_data = 0;
    Error_Port_Reg port_reg;

    port_reg.val = 0;

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
                    ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->prc_callback(code, NULL, 0);
            }
            else if (ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->proc_type == Error_Proc_Next)
            {
                /* add error into linked list */
                /* reserve */
            }
            else if (ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->proc_type == Error_Proc_Ignore)
            {
                /* reserve */
            }

            port_reg.section.log_reg = ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->log;
            port_reg.section.out_reg = ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->out;
            port_reg.section.len = strlen(ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->desc);

            if (ErrorQueue_CreateState && port_reg.val)
            {
                /* Push Error describe into Error_Queue */
                /* push out or log state */
                Queue.push(&ErrorQueue, &port_reg.val, sizeof(port_reg.val));
                /* push error decribe */
                Queue.push(&ErrorQueue, ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->desc, strlen(ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->desc));
            }
        }
    }

    return true;
}

static void Error_Set_Callback(ErrorLog_Callback_Type_List type, error_port_callback callback)
{
    switch (type)
    {
    case Error_Out_Callback:
        out_callback = callback;
        break;

    case Error_Log_Callback:
        log_callback = callback;
        break;

    default:
        break;
    }
}
/* still in half way */
static bool Error_Proc(Error_Handler hdl)
{
    ErrorStream_TypeDef stream;
    Error_Port_Reg ErrorQueue_Head_State;
    uint8_t *p_data = NULL;

    memset(&ErrorQueue_Head_State, NULL, sizeof(Error_Port_Reg));
    memset(&stream, NULL, sizeof(ErrorStream_TypeDef));

    if (hdl == 0 || !ErrorQueue_CreateState)
        return false;

    /* reserved still developing */
    if (ErrorHandleToObj(hdl)->link_node)
    {
    }

    Queue.check(&ErrorQueue, 0, &ErrorQueue_Head_State.val, sizeof(ErrorQueue_Head_State.val));

    if ((ErrorQueue_Head_State.section.out_reg || ErrorQueue_Head_State.section.log_reg) && (out_callback || log_callback))
    {
        p_data = (uint8_t *)MMU_Malloc(ErrorQueue_Head_State.section.len);

        if (p_data)
        {
            if ((Queue.state(ErrorQueue) == Queue_full) || (Queue.state(ErrorQueue) == Queue_ok))
            {
                Queue.pop(&ErrorQueue, &ErrorQueue_Head_State.val, sizeof(ErrorQueue_Head_State.val));
                Queue.pop(&ErrorQueue, p_data, ErrorQueue_Head_State.section.len);

                if (ErrorQueue_Head_State.section.out_reg && out_callback && p_data)
                {
                    out_callback(p_data, ErrorQueue_Head_State.section.len);
                }

                if (ErrorQueue_Head_State.section.log_reg && log_callback && p_data)
                {
                    log_callback(p_data, ErrorQueue_Head_State.section.len);
                }
            }

            MMU_Free(p_data);
        }
    }

    return true;
}
