#include "queue.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>

/* internal function */
static Queue_state Queue_UpdateState(QueueObj_TypeDef *obj);

/* external function */
static bool Queue_Create(QueueObj_TypeDef *obj, char *name, uint16_t len);
static bool Queue_Reset(QueueObj_TypeDef *obj);
static Queue_state Queue_Push(QueueObj_TypeDef *obj, uint8_t *data, uint16_t len);
static Queue_state Queue_Pop(QueueObj_TypeDef *obj, uint8_t *data, uint16_t size);

/* extern virable */
Queue_TypeDef Queue = {
    .create = Queue_Create,
    .reset = Queue_Reset,
    .push = Queue_Push,
    .pop = Queue_Pop,
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

static Queue_state Queue_Push(QueueObj_TypeDef *obj, uint8_t *data, uint16_t len)
{
    if ((obj->state == Queue_ok) || (obj->state == Queue_empty))
    {
        if (len <= (obj->lenth - obj->size))
        {
            for (uint16_t i = 0; i < len; i++)
            {
                obj->end_pos = (obj->end_pos++) % obj->lenth;

                obj->buff[obj->end_pos] = data[i];
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
    if ((obj->state == Queue_ok) || (obj->state == Queue_full))
    {
        for (uint16_t i = 0; i < size; i++)
        {
            obj->head_pos = (obj->head_pos++) % obj->lenth;

            data[i] = obj->buff[obj->head_pos];
            obj->buff[obj->head_pos] = NULL;
            obj->size--;

            Queue_UpdateState(obj);
        }
    }
    else
        return Queue_overflow_r;

    return obj->state;
}
