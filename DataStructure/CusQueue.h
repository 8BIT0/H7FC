#ifndef __CUSQUEUE_H
#define __CUSQUEUE_H

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
    uint8_t *ptr;
    uint16_t size;
} QueueDump_DataObj_TypeDef;

typedef struct
{
    uint16_t (*size)(QueueObj_TypeDef obj);
    uint16_t (*remain)(QueueObj_TypeDef obj);
    bool (*create_auto)(QueueObj_TypeDef *obj, char *name, uint16_t len);
    bool (*create_with_buf)(QueueObj_TypeDef *obj, char *name, uint8_t *buff, uint16_t len);
    bool (*reset)(QueueObj_TypeDef *obj);
    bool (*check)(QueueObj_TypeDef *obj, uint16_t index, uint8_t *data, uint16_t size);
    Queue_state (*push)(QueueObj_TypeDef *obj, uint8_t *data, uint16_t size);
    Queue_state (*pop)(QueueObj_TypeDef *obj, uint8_t *data, uint16_t size);
    Queue_state (*state)(QueueObj_TypeDef obj);
    bool (*pop_to_queue)(QueueObj_TypeDef *src, QueueObj_TypeDef *dst);
} Queue_TypeDef;

extern Queue_TypeDef Queue;

#endif
