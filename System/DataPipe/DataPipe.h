#ifndef __DATAPIPE_H
#define __DATAPIPE_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

typedef void (*Pipe_TransFinish_Callback)(void *pipe_obj);
typedef void (*Pipe_TransError_Callback)(void *pipe_obj);
typedef void (*Pipe_TimeOutProc_Callback)(void *pipe_obj);

#define DataPipeHandleToObj(x) ((DataPipeObj_TypeDef *)x)
#define DataPipe_CreateDataObj(type, name) static type name##_##PipeDataObj __attribute__((section(".Perph_Section")))
#define DataPipe_DataObjAddr(name) (uint32_t)(&name##_##PipeDataObj)
#define DataPipe_DataObj(name) name##_##PipeDataObj

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
    uint32_t data_addr;
    uint16_t data_size;

    Pipe_TransFinish_Callback trans_finish_cb;
    Pipe_TransError_Callback trans_error_cb;

    uint32_t tx_cnt;
    uint32_t rx_cnt;
    uint32_t er_cnt;

    uint8_t *ptr_tmp;
} DataPipeObj_TypeDef;
#pragma pack()

typedef struct
{
    DataPipeObj_TypeDef *org;
    DataPipeObj_TypeDef *dst;
}Data_PlugedPipeObj_TypeDef;

bool DataPipe_Init(void);
bool DataPipe_SendTo(DataPipeObj_TypeDef *p_org, DataPipeObj_TypeDef *p_dst);
bool DataPipe_DealError(void);

extern DataPipeObj_TypeDef IMU_Smp_DataPipe;
extern DataPipeObj_TypeDef IMU_Log_DataPipe;
extern DataPipeObj_TypeDef IMU_Ptl_DataPipe;

#endif
