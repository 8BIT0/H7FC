#ifndef __QUEUE_H
#define __QUEUE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define QUEUE_MAX_SIZE 512
#define MAX_QUEUE_NUM 10

typedef enum
{
    Queue_FIFO,
    Queue_LIFO,
} QueueData_OutType;

typedef enum
{
    Queue_ok,
    Queue_CheckErrorPos,
    Queue_overflow_w,
    Queue_overlimit_r,
    Queue_empty,
    Queue_full,
    Queue_CreateFailed,
} Queue_state;

#define QUEUE_ERROR_TYPENUM Queue_CreateFailed - Queue_overflow_w
#define GET_QUEUE_ERROR_INDEX(x) ((x - Queue_overflow_w) >= 0) ? (((x - Queue_overflow_w) < QUEUE_ERROR_TYPENUM) ? (x - Queue_overflow_w) : -2) : -1

typedef union
{
    uint16_t output;

    struct
    {
        Queue_state state : 8;
        uint8_t value : 8;
    } reg;
} Queue_CheckOut_u;

typedef struct
{
    char *name;
    uint16_t size;
    Queue_state state;
    uint16_t head_pos;
    uint16_t end_pos;
    uint8_t buff[QUEUE_MAX_SIZE];
    QueueData_OutType output_type;
    uint32_t error_times[QUEUE_ERROR_TYPENUM];
    uint32_t total_error_times;
} queue_s;

Queue_state Queue_Init(queue_s *queue, char *name, QueueData_OutType type);
Queue_state Queue_Reset(queue_s *queue);
Queue_state Queue_Dump(queue_s *queue, char *out_data);
Queue_state Queue_PushChar(queue_s *queue, char data);
Queue_state Queue_PushLenChar(queue_s *queue, uint16_t len, char *data);
Queue_state Queue_PopCharFromFront(queue_s *queue, char *out_data);
Queue_state Queue_PopCharFromBack(queue_s *queue, char *out_data);
Queue_state Queue_PopLenCharFromFront(queue_s *queue, uint16_t len, char *out_buff);
Queue_state Queue_PopLenCharFromBack(queue_s *queue, uint16_t len, char *out_buff);
Queue_CheckOut_u Queue_CheckData(queue_s queue, uint16_t index);
void Queue_Output_ErrorTimes(queue_s queue);
#endif
