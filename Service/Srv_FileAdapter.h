#ifndef __SRV_FILEADAPTER_H
#define __SRV_FILEADAPTER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "YModem.h"

#define SrvFileAdapter_TimeOut (10 * 1000)  /* time out 10s */

typedef struct
{
    uint16_t total_size;
    uint16_t cur_size;
    uint8_t *buf;
} SrvFileAdapter_Stream_TypeDef;

typedef void (*SrvFileAdapter_Send_Func)(uint8_t *p_buf, uint16_t len);

typedef enum
{
    Adapter_FC_APP = 0,
    Adapter_FC_Boot,
    Adapter_FC_TelemtryReceiver,
} SrvFileAdapter_FileType_List;

typedef enum
{
    SrvFileAdapter_Frame_None = 0,
    SrvFileAdapter_Frame_YModem,
    SrvFileAdapter_Frame_Sum,
} SrvFileAdapter_ProtoFrameType_List;

typedef struct
{
    uint32_t port_addr;
    SrvFileAdapter_FileType_List file_type;
    SrvFileAdapter_ProtoFrameType_List frame_type;

    void *FrameObj;
    void *FrmaeApi;

    bool is_actived;
    bool chancel;
    bool ready_to_rec;
    
    SrvFileAdapter_Send_Func send;
} SrvFileAdapterObj_TypeDef;

typedef struct
{
    SrvFileAdapterObj_TypeDef* (*create)(SrvFileAdapter_ProtoFrameType_List frame_type, uint32_t stream_size);
    bool (*destory)(SrvFileAdapterObj_TypeDef *p_Adapter);
    void (*set_send)(SrvFileAdapterObj_TypeDef *p_Adapter, SrvFileAdapter_Send_Func send_cb);
    bool (*is_active)(SrvFileAdapterObj_TypeDef *p_Adapter);
    bool (*bind_port)(SrvFileAdapterObj_TypeDef *p_Adapter, uint32_t port_addr);
    void (*polling)(SrvFileAdapterObj_TypeDef *p_Adapter);
    void (*parse)(SrvFileAdapterObj_TypeDef *p_Adapter, const SrvFileAdapter_Stream_TypeDef stream);
} SrvFileAdapter_TypeDef;

extern SrvFileAdapter_TypeDef SrvFileAdapter;

#endif

