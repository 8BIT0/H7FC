#ifndef __DATAPIPE_H
#define __DATAPIPE_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

typedef uint32_t DataPipe_Handle;
typedef void (*Pipe_TransFinish_Callback)(void);
typedef void (*Pipe_TransError_Callback)(void);

#define DataPipeHandleToObj(x) ((DataPipeObj_TypeDef *)x)

typedef struct
{
    volatile uint32_t data_addr;
    uint16_t data_size;

    Pipe_TransFinish_Callback trans_finish_cb;
    Pipe_TransError_Callback trans_error_cb;
} DataPipeObj_TypeDef;

#endif
