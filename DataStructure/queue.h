#ifndef __QUEUE_H
#define __QUEUE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef enum
{
    Queue_ok,
    Queue_overflow_w,
    Queue_overflow_r,
    Queue_empty,
    Queue_full,
    Queue_obj_error,
} Queue_state;

typedef union
{
    uint16_t output;

    struct
    {
        Queue_state state : 8;
        uint8_t value : 8;
    } reg;
} Queue_CheckOut_u;

#pragma pack(1)
typedef struct
{
    char *name;
    uint16_t lenth; // total queue size
    uint16_t size;  // current data size in queue
    Queue_state state;
    uint16_t head_pos;
    uint16_t end_pos;
    uint8_t *buff;
} QueueObj_TypeDef;

#pragma pack()

typedef struct
{
    bool (*create)(QueueObj_TypeDef *obj, char *name, uint16_t len);
    bool (*reset)(QueueObj_TypeDef *obj);
    bool (*check)(QueueObj_TypeDef *obj, uint16_t index, uint8_t *data, uint16_t size);
    Queue_state (*push)(QueueObj_TypeDef *obj, uint8_t *data, uint16_t size);
    Queue_state (*pop)(QueueObj_TypeDef *obj, uint8_t *data, uint16_t size);
    Queue_state (*state)(QueueObj_TypeDef obj);
} Queue_TypeDef;

extern Queue_TypeDef Queue;

#endif
