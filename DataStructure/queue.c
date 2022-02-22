#include "queue.h"
#include <stdio.h>
#include <string.h>

static uint8_t Cur_Crtd_Queue = 0;

Queue_state Queue_Init(queue_s *queue, char *name, QueueData_OutType type)
{
    if (Cur_Crtd_Queue < MAX_QUEUE_NUM)
    {
        Cur_Crtd_Queue++;
        queue->state = Queue_ok;
    }
    else
        queue->state = Queue_CreateFailed;

    memset(queue->buff, NULL, QUEUE_MAX_SIZE);
    queue->name = name;
    queue->end_pos = 0;
    queue->head_pos = 0;
    queue->size = 0;
    queue->output_type = type;

    for (uint8_t i = 0; i < QUEUE_ERROR_TYPENUM; i++)
    {
        queue->error_times[i] = 0;
    }
    queue->total_error_times = 0;

    return queue->state;
}

Queue_state Queue_Reset(queue_s *queue)
{
    return Queue_init(queue, queue->name);
}

static Queue_state Queue_GetState(queue_s *queue)
{
    if ((queue->head_pos == queue->end_pos) || (queue->size == 0))
    {
        queue->error_times[GET_QUEUE_ERROR_INDEX(Queue_empty)]++;
        queue->total_error_times++;
        queue->state = Queue_empty;
        return Queue_empty;
    }

    if (((queue->end_pos + 1) % QUEUE_MAX_SIZE) == queue->head_pos)
    {
        queue->error_times[GET_QUEUE_ERROR_INDEX(Queue_full)]++;
        queue->total_error_times++;
        queue->state = Queue_full;
        return Queue_full;
    }

    queue->state = Queue_ok;
    return Queue_ok;
}

Queue_state Queue_PushChar(queue_s *queue, char data)
{
    if (queue->state == Queue_ok)
    {
        queue->buff[queue->end_pos] = data;

        queue->end_pos = (queue->end_pos + 1) % QUEUE_MAX_SIZE;
        queue->size++;

        Queue_GetState(queue);
    }

    return queue->state;
}

Queue_state Queue_PushLenChar(queue_s *queue, uint16_t len, char *data)
{
    if ((queue->state == Queue_ok) || (queue->state == Queue_empty))
    {
        if (len <= (QUEUE_MAX_SIZE - queue->size))
        {
            for (uint16_t index = 0; index < len; index++)
            {
                queue->buff[queue->end_pos + index] = data[index];
            }

            queue->end_pos = (queue->end_pos + len) % QUEUE_MAX_SIZE;
            queue->size += len;
            Queue_GetState(queue);
        }
        else
        {
            queue->error_times[GET_QUEUE_ERROR_INDEX(Queue_overflow_w)]++;
            queue->total_error_times++;
            return Queue_overflow_w;
        }
    }

    return queue->state;
}

Queue_state Queue_PopCharFromFront(queue_s *queue, char *out_data)
{
    if ((queue->state == Queue_ok) || (queue->state == Queue_full))
    {
        *out_data = queue->buff[queue->head_pos];
        queue->buff[queue->head_pos] = NULL;

        queue->head_pos == (queue->head_pos + 1) % QUEUE_MAX_SIZE;
        queue->size--;

        Queue_GetState(queue);
    }

    return queue->state;
}

//still have bug in this func fix when i start to use it hahahahha boooyaaaaa
Queue_state Queue_PopCharFromBack(queue_s *queue, char *out_data)
{
    if ((queue->state == Queue_ok) || (queue->state == Queue_full))
    {
        *out_data = queue->buff[queue->end_pos - 1];
        queue->buff[queue->end_pos - 1] = NULL;

        queue->end_pos = (queue->end_pos - 1) % QUEUE_MAX_SIZE;
        queue->size--;

        Queue_GetState(queue);
    }

    return queue->state;
}

Queue_state Queue_Dump(queue_s *queue, char *out_data)
{

    for (uint16_t index = 0; index < queue->size; index++)
    {
        if (queue->output_type == Queue_FIFO)
        {
            out_data[index] = queue->buff[index];
        }
        else
        {
            out_data[index] = queue->buff[queue->size - 1 - index];
        }

        queue->buff[index] = NULL;
    }

    queue->size = 0;
    queue->end_pos = 0;
    queue->head_pos = 0;
    queue->state = Queue_empty;

    return queue->state;
}

Queue_state Queue_PopLenCharFromFront(queue_s *queue, uint16_t len, char *out_buff)
{
    static uint32_t test = 0;
    if ((queue->state == Queue_ok) || (queue->state == Queue_full))
    {
        if (len <= queue->size)
        {
            for (uint16_t index = 0; index < len; index++)
            {
                if (queue->output_type == Queue_FIFO)
                {
                    //FIFO
                    out_buff[index] = queue->buff[queue->head_pos + index];
                    queue->buff[queue->head_pos + index] = NULL;
                }
                else
                {
                    //LIFO
                    out_buff[index] = queue->buff[queue->head_pos + len - 1 - index];
                    queue->buff[queue->head_pos + len - 1 - index] = NULL;
                }
            }

            queue->size -= len;
            queue->head_pos = (queue->head_pos + len) % QUEUE_MAX_SIZE;

            Queue_GetState(queue);
            return Queue_ok;
        }
        else
        {
            test++;
            //count error num
            queue->error_times[GET_QUEUE_ERROR_INDEX(Queue_overlimit_r)]++;
            queue->total_error_times++;
            return Queue_overlimit_r;
        }
    }
    else
    {
        return Queue_empty;
    }
}

//func describe same as Queue_PopCharFromBack
Queue_state Queue_PopLenCharFromBack(queue_s *queue, uint16_t len, char *out_buff)
{
    if ((queue->state == Queue_ok) || (queue->state == Queue_full))
    {
        if (len < queue->size)
        {
            for (uint16_t index = 0; index < len; index++)
            {
                if (queue->output_type == Queue_LIFO)
                {
                    out_buff[index] = queue->buff[(queue->end_pos - 1) - index];
                    queue->buff[(queue->end_pos - 1) - index] = NULL;
                }
                else
                {
                    out_buff[index] = queue->buff[(queue->end_pos - 1 - len) + index];
                    queue->buff[(queue->end_pos - 1 - len) + index] = NULL;
                }
            }

            queue->size -= len;
            queue->end_pos = (queue->end_pos - len) % QUEUE_MAX_SIZE;

            Queue_GetState(queue);
        }
        else
        {
            queue->error_times[GET_QUEUE_ERROR_INDEX(Queue_overlimit_r)]++;
            queue->total_error_times++;
            return Queue_overlimit_r;
        }
    }

    return queue->state;
}

Queue_CheckOut_u Queue_CheckData(queue_s queue, uint16_t index)
{
    Queue_CheckOut_u Data_tmp;
    Data_tmp.reg.value = 0;

    if (queue.state != Queue_empty)
    {
        if ((index < QUEUE_MAX_SIZE) && (index <= queue.size))
        {
            Data_tmp.reg.state = Queue_ok;
            Data_tmp.reg.value = queue.buff[(index + queue.head_pos) % QUEUE_MAX_SIZE];
        }
        else
        {
            Data_tmp.reg.state = Queue_CheckErrorPos;
        }
    }
    else
    {
        Data_tmp.reg.state = Queue_empty;
    }

    return Data_tmp;
}

void Queue_Output_ErrorTimes(queue_s queue)
{
    if (queue.total_error_times)
    {
        for (uint8_t i = 0; i < QUEUE_ERROR_TYPENUM; i++)
        {
            if (queue.error_times[i])
            {
                switch (i + Queue_overflow_w)
                {
                case Queue_overflow_w:
                    break;

                case Queue_overlimit_r:
                    break;

                case Queue_empty:
                    break;

                case Queue_full:
                    break;

                case Queue_CreateFailed:
                    break;
                }
            }
        }
    }
}
