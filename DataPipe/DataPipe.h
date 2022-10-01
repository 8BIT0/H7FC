#ifndef __DATAPIPE_H
#define __DATAPIPE_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

typedef uint32_t DataPipe_Handle;
typedef void (*Pipe_TransFinish_Callback)(void);
typedef void (*Pipe_TransError_Callback)(void);
typedef void (*Pipe_TimeOutProc_Callback)(void);

#define DataPipeHandleToObj(x) ((DataPipeObj_TypeDef *)x)

typedef enum
{
    Pipe_UnReady = 0,
    Pipe_Ready,
    Pipe_Busy,
    Pipe_Error,
}DataPipe_State_List;

#pragma pack(1)
typedef struct
{
    volatile uint32_t data_addr;
    uint16_t data_size;

    uint32_t time_out;

    Pipe_TransFinish_Callback trans_finish_cb;
    Pipe_TransError_Callback trans_error_cb;
    Pipe_TimeOutProc_Callback trans_timeout_cb;

    uint64_t tx_cnt;
    uint64_t rx_cnt;
    uint64_t er_cnt;
} DataPipeObj_TypeDef;
#pragma pack()

typedef struct
{
    DataPipeObj_TypeDef *org;
    DataPipeObj_TypeDef *dst;
}Data_PlugedPipeObj_TypeDef;

#endif
