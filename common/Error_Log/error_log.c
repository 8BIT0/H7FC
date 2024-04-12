#include "error_log.h"
#include "../DataStructure/CusQueue.h"
#include <stdarg.h>

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
static error_port_callback out_callback = NULL;
static error_port_callback log_callback = NULL;
static Error_Port_Reg port_reg;

/* external function */
static Error_Handler ErrorTree_Create(char *name);
static bool Error_Register(Error_Handler hdl, Error_Obj_Typedef *obj, uint16_t num);
static bool Error_Trigger(Error_Handler hdl, int16_t code, uint8_t *p_arg, uint16_t size);
static bool Error_Proc(Error_Handler hdl);
static void Error_Set_Callback(ErrorLog_Callback_Type_List type, error_port_callback callback);
static uint32_t Error_DescToQueue(const char *str, ...);

ErrorLog_TypeDef ErrorLog = {
    .create = ErrorTree_Create,
    .proc = Error_Proc,
    .registe = Error_Register,
    .trigger = Error_Trigger,
    .set_callback = Error_Set_Callback,
    .add_desc = Error_DescToQueue,
};

static data_handle Error_InsertPriority_Compare(data_handle l_addr, data_handle r_addr)
{
    if ((l_addr == 0) && (r_addr == 0) && (l_addr != r_addr))
        return 0;

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

    return 0;
}

static Error_Handler ErrorTree_Create(char *name)
{
    volatile ErrorTree_TypeDef *Error_Tmp = NULL;

    if (!ErrorQueue_Init)
    {
        ErrorQueue_CreateState = Queue.create_auto(&ErrorQueue, "ErrorQueue", ERROR_DESC_BUFFSIZE);
        ErrorQueue_Init = true;
    }

    Error_Tmp = (ErrorTree_TypeDef *)ERROR_LOG_MALLOC(sizeof(ErrorTree_TypeDef));

    if (Error_Tmp == NULL)
        return 0;

    Error_Tmp->tree = BalanceTree.Create(name, Error_InsertPriority_Compare, Error_Search, Error_InsertPriority_Compare);

    if (Error_Tmp == NULL)
        return 0;

    Error_Tmp->reg_num = 0;
    Error_Tmp->link_node = NULL;

    return (Error_Handler)Error_Tmp;
}

static bool Error_Register(Error_Handler hdl, Error_Obj_Typedef *Obj_List, uint16_t num)
{
    if (hdl == 0)
        return false;

    for (uint16_t i = 0; i < num; i++)
    {
        BalanceTree.Insert(ErrorHandleToObj(hdl)->tree, Obj_List[i].desc, (data_handle)(&Obj_List[i]));
    }

    ErrorHandleToObj(hdl)->reg_num = num;
    return true;
}

static uint32_t Error_DescToQueue(const char *str, ...)
{
    uint32_t length = 0;
    char p_data[128] = {0};
    va_list arp;
    va_start(arp, str);

    if (ErrorQueue_CreateState)
    {
        length = vsnprintf((char *)p_data, sizeof(p_data), (char *)str, arp);

        port_reg.section.len = length;
        Queue.push(&ErrorQueue, (uint8_t *)&port_reg.val, sizeof(port_reg.val));
        Queue.push(&ErrorQueue, (uint8_t *)p_data, length);
    }

    va_end(arp);
    return length;
}

/* still in half way */
static bool Error_Trigger(Error_Handler hdl, int16_t code, uint8_t *p_arg, uint16_t size)
{
    int16_t code_tmp = code;
    ErrorStream_TypeDef data_stream;
    char *letter = NULL;

    port_reg.val = 0;

    if (hdl == 0)
        return false;

    TreeNode_Handle search_handle = BalanceTree.Search(ErrorHandleToObj(hdl)->tree, (data_handle)&code_tmp);

    if(!search_handle)
        return false;

    data_stream.p_data = p_arg;
    data_stream.size = size;

    (ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->prc_data_stream).p_data = p_arg;
    (ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->prc_data_stream).size = size;

    /* find target node */
    if (search_handle)
    {
        if (TreeNodeHandleToObj(search_handle)->data)
        {
            port_reg.section.log_reg = ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->log;
            port_reg.section.out_reg = ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->out;
            port_reg.section.len = strlen(ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->desc) + strlen(ErrorHandleToObj(hdl)->tree->name) + strlen("[  ] ");

            if (ErrorQueue_CreateState && port_reg.val)
            {
                /* Push Error describe into Error_Queue */
                /* push out or log state */
                Queue.push(&ErrorQueue, (uint8_t *)&port_reg.val, sizeof(port_reg.val));
                /* push error tree name */
                letter = "[ ";
                Queue.push(&ErrorQueue, (uint8_t *)letter, strlen(letter));

                Queue.push(&ErrorQueue, (uint8_t *)ErrorHandleToObj(hdl)->tree->name, strlen(ErrorHandleToObj(hdl)->tree->name));

                letter = " ] ";
                Queue.push(&ErrorQueue, (uint8_t *)letter, strlen(letter));

                /* push error decribe */
                Queue.push(&ErrorQueue, (uint8_t *)ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->desc, strlen(ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->desc));
            }

            if (ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->proc_type == Error_Proc_Immd)
            {
                /* trigger process callback */
                if (ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->prc_callback)
                {
                    ErrorTreeDataToObj(TreeNodeHandleToObj(search_handle)->data)->prc_callback(code, data_stream.p_data, data_stream.size);
                }
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
        }
    }

    port_reg.val = 0;

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

    memset(&ErrorQueue_Head_State, 0, sizeof(Error_Port_Reg));
    memset(&stream, 0, sizeof(ErrorStream_TypeDef));

    if (hdl == 0 || !ErrorQueue_CreateState)
        return false;

    /* reserved still developing */
    if (ErrorHandleToObj(hdl)->link_node)
    {
    }

    Queue.check(&ErrorQueue, 0, (uint8_t *)&ErrorQueue_Head_State.val, sizeof(ErrorQueue_Head_State.val));

    if ((ErrorQueue_Head_State.section.out_reg || ErrorQueue_Head_State.section.log_reg) && (out_callback || log_callback))
    {
        p_data = (uint8_t *)ERROR_LOG_MALLOC(ErrorQueue_Head_State.section.len);

        if (p_data)
        {
            if ((Queue.state(ErrorQueue) == Queue_full) || (Queue.state(ErrorQueue) == Queue_ok))
            {
                Queue.pop(&ErrorQueue, (uint8_t *)&ErrorQueue_Head_State.val, sizeof(ErrorQueue_Head_State.val));
                Queue.pop(&ErrorQueue, (uint8_t *)p_data, ErrorQueue_Head_State.section.len);

                if (ErrorQueue_Head_State.section.out_reg && out_callback && p_data)
                {
                    out_callback(p_data, ErrorQueue_Head_State.section.len);
                }

                if (ErrorQueue_Head_State.section.log_reg && log_callback && p_data)
                {
                    log_callback(p_data, ErrorQueue_Head_State.section.len);
                }
            }

            ERROR_LOG_FREE(p_data);
        }
    }

    return true;
}
