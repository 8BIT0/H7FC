#include "CusQueue.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include "Srv_OsCommon.h"

#define Queue_Mem_Malloc(x) SrvOsCommon.malloc(x)
#define Queue_Mem_Free(x) SrvOsCommon.free(x)

/* internal function */
static Queue_state Queue_UpdateState(QueueObj_TypeDef *obj);

/* external function */
static bool Queue_Create_Auto(QueueObj_TypeDef *obj, char *name, uint16_t len);
static bool Queue_Create_WithCertainBuff(QueueObj_TypeDef *obj, char *name, uint8_t *buff, uint16_t len);
static bool Queue_Reset(QueueObj_TypeDef *obj);
static Queue_state Queue_GetState(QueueObj_TypeDef obj);
static Queue_state Queue_Push(QueueObj_TypeDef *obj, uint8_t *data, uint16_t size);
static Queue_state Queue_Pop(QueueObj_TypeDef *obj, uint8_t *data, uint16_t size);
static bool Queue_Check(QueueObj_TypeDef *obj, uint16_t index, uint8_t *data, uint16_t size);
static uint16_t Queue_GetSize(QueueObj_TypeDef obj);
static uint16_t Queue_GetRemain(QueueObj_TypeDef obj);
static bool Queue_PopTo(QueueObj_TypeDef *src, QueueObj_TypeDef *dst);

/* extern virable */
Queue_TypeDef Queue = {
    .create_auto = Queue_Create_Auto,
    .create_with_buf = Queue_Create_WithCertainBuff,
    .reset = Queue_Reset,
    .push = Queue_Push,
    .pop = Queue_Pop,
    .check = Queue_Check,
    .state = Queue_GetState,
    .size = Queue_GetSize,
    .remain = Queue_GetRemain,
    .pop_to_queue = Queue_PopTo,
};

static bool Queue_Create_WithCertainBuff(QueueObj_TypeDef *obj, char *name, uint8_t *buff, uint16_t len)
{
    if ((obj == NULL) || (len == 0))
        return false;

    obj->name = name;
    obj->end_pos = 0;
    obj->head_pos = 0;
    obj->size = 0;
    obj->lenth = len;

    obj->buff = buff;
    memset(obj->buff, 0, obj->lenth);

    if (obj->buff == NULL)
        return false;

    obj->state = Queue_empty;

    return true;  
}

static bool Queue_Create_Auto(QueueObj_TypeDef *obj, char *name, uint16_t len)
{
    if ((obj == NULL) || (len == 0))
        return false;

    obj->name = name;
    obj->end_pos = 0;
    obj->head_pos = 0;
    obj->size = 0;
    obj->lenth = len;

    obj->buff = (uint8_t *)Queue_Mem_Malloc(len);
    memset(obj->buff, 0, obj->lenth);

    if (obj->buff == NULL)
        return false;

    obj->state = Queue_empty;

    return true;
}

static bool Queue_Reset(QueueObj_TypeDef *obj)
{
    if ((obj == NULL) || (obj->buff == NULL) || (obj->lenth == 0))
        return false;

    obj->end_pos = 0;
    obj->head_pos = 0;
    obj->size = 0;

    obj->state = Queue_empty;
    memset(obj->buff, 0, obj->lenth);

    return true;
}

static Queue_state Queue_UpdateState(QueueObj_TypeDef *obj)
{
    if ((obj->head_pos == obj->end_pos) && (obj->size == 0))
    {
        obj->state = Queue_empty;
        return Queue_empty;
    }

    if ((((obj->end_pos + 1) % obj->lenth) == obj->head_pos) || (obj->size == obj->lenth))
    {
        obj->state = Queue_full;
        return Queue_full;
    }

    obj->state = Queue_ok;
    return obj->state;
}

static bool Queue_PopTo(QueueObj_TypeDef *src, QueueObj_TypeDef *dst)
{
    uint16_t dst_remain_size = 0;
    uint16_t src_size = 0;
    uint16_t copy_size = 0;

    if((src == NULL) || (dst == NULL) || (src->size == 0) || (dst->lenth == dst->size))
        return false;

    dst_remain_size = Queue_GetRemain(*dst);
    src_size = Queue_GetSize(*src);

    copy_size = (dst_remain_size <= src_size) ? dst_remain_size : src_size;

    for(uint16_t i = 0; i < copy_size; i++)
    {
        dst->end_pos %= dst->lenth;
        src->head_pos %= src->lenth;

        dst->buff[dst->end_pos] = src->buff[src->head_pos];
        src->buff[src->head_pos] = 0;

        src->head_pos++;
        src->size--;
        
        dst->end_pos++;
        dst->size++;

        Queue_UpdateState(dst);
        Queue_UpdateState(src);
    }

    return true;
}

static Queue_state Queue_Push(QueueObj_TypeDef *obj, uint8_t *data, uint16_t size)
{
    if ((obj == NULL) || (obj->lenth == 0))
        return Queue_obj_error;

    if ((obj->state == Queue_ok) || (obj->state == Queue_empty))
    {
        if (size <= (obj->lenth - obj->size))
        {
            for (uint16_t i = 0; i < size; i++)
            {
                obj->end_pos %= obj->lenth;
                obj->buff[obj->end_pos] = data[i];
                obj->end_pos++;

                obj->size++;

                Queue_UpdateState(obj);
            }
        }
        else
            return Queue_overflow_w;
    }

    return obj->state;
}

/* still can be optimize */
static Queue_state Queue_Pop(QueueObj_TypeDef *obj, uint8_t *data, uint16_t size)
{
    if ((obj == NULL) || (obj->lenth == 0))
        return Queue_obj_error;

    if ((obj->state == Queue_ok) || (obj->state == Queue_full))
    {
        if (size <= obj->size)
        {
            for (uint16_t i = 0; i < size; i++)
            {
                obj->head_pos %= obj->lenth;
                data[i] = obj->buff[obj->head_pos];
                obj->buff[obj->head_pos] = 0;
                obj->head_pos++;

                obj->size--;

                Queue_UpdateState(obj);
            }
        }
        else
            return Queue_overflow_r;
    }

    return obj->state;
}

static uint16_t Queue_GetRemain(QueueObj_TypeDef obj)
{
    return obj.lenth - obj.size;
}

static bool Queue_Check(QueueObj_TypeDef *obj, uint16_t index, uint8_t *data, uint16_t size)
{
    if (obj == NULL || (size == 0) || (data == NULL))
        return false;

    for (uint8_t i = 0; i < size; i++)
    {
        data[i] = obj->buff[(obj->head_pos + index + i) % obj->lenth];
    }

    return true;
}

static Queue_state Queue_GetState(QueueObj_TypeDef obj)
{
    return obj.state;
}

static uint16_t Queue_GetSize(QueueObj_TypeDef obj)
{
    return obj.size;
}
