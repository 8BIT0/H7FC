#include "queue.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include "mmu.h"

#define Queue_Mem_Malloc(x) MMU_Malloc(x)
#define Queue_Mem_Free(x) MMU_Free(x)

/* internal function */
static Queue_state Queue_UpdateState(QueueObj_TypeDef *obj);

/* external function */
static bool Queue_Create(QueueObj_TypeDef *obj, char *name, uint16_t len);
static bool Queue_Reset(QueueObj_TypeDef *obj);
static Queue_state Queue_GetState(QueueObj_TypeDef obj);
static Queue_state Queue_Push(QueueObj_TypeDef *obj, uint8_t *data, uint16_t size);
static Queue_state Queue_Pop(QueueObj_TypeDef *obj, uint8_t *data, uint16_t size);
static bool Queue_Check(QueueObj_TypeDef *obj, uint16_t index, uint8_t *data, uint16_t size);
static uint16_t Queue_GetSize(QueueObj_TypeDef obj);

/* extern virable */
Queue_TypeDef Queue = {
    .create = Queue_Create,
    .reset = Queue_Reset,
    .push = Queue_Push,
    .pop = Queue_Pop,
    .check = Queue_Check,
    .state = Queue_GetState,
    .size = Queue_GetSize,
};

static bool Queue_Create(QueueObj_TypeDef *obj, char *name, uint16_t len)
{
    if ((obj == NULL) || (len == 0))
        return false;

    obj->name = name;
    obj->end_pos = 0;
    obj->head_pos = 0;
    obj->size = 0;
    obj->lenth = len;

    obj->buff = (uint8_t *)Queue_Mem_Malloc(len);
    memset(obj->buff, NULL, obj->lenth);

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
    memset(obj->buff, NULL, obj->lenth);

    return true;
}

static Queue_state Queue_UpdateState(QueueObj_TypeDef *obj)
{
    if ((obj->head_pos == obj->end_pos) || (obj->size == 0))
    {
        obj->state = Queue_empty;
        return Queue_empty;
    }

    if (((obj->end_pos + 1) % obj->lenth) == obj->head_pos)
    {
        obj->state = Queue_full;
        return Queue_full;
    }

    obj->state = Queue_ok;
    return obj->state;
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
                obj->buff[obj->head_pos] = NULL;
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
