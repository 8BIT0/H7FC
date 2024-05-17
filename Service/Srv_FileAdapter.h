#ifndef __SRV_FILEADAPTER_H
#define __SRV_FILEADAPTER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "YModem.h"

#define SrvFileAdapter_TimeOut (10 * 1000)  /* time out 10s */

typedef void (*SrvFileAdapter_Send_Func)(uint8_t *p_buf, uint16_t len);

typedef enum
{
    SrvFileAdapter_Frame_None = 0,
    SrvFileAdapter_Frame_YModem,
    SrvFileAdapter_Frame_Sum,
} Adapter_ProtoType_List;

typedef struct
{
    uint32_t port_addr;
    Adapter_ProtoType_List frame_type;

    void *FrameObj;
    void *FrmaeApi;

    bool chancel;
    bool ready_to_rec;
    
    SrvFileAdapter_Send_Func send;
} SrvFileAdapterObj_TypeDef;

typedef struct
{
    SrvFileAdapterObj_TypeDef* (*create)(Adapter_ProtoType_List proto_type, uint32_t stream_size);
    bool (*destory)(SrvFileAdapterObj_TypeDef *p_Adapter);
    void (*set_send)(SrvFileAdapterObj_TypeDef *p_Adapter, SrvFileAdapter_Send_Func send_cb);
    void (*polling)(SrvFileAdapterObj_TypeDef *p_Adapter);
    void (*parse)(SrvFileAdapterObj_TypeDef *p_Adapter, uint8_t *p_buf, uint16_t len);
} SrvFileAdapter_TypeDef;

extern SrvFileAdapter_TypeDef SrvFileAdapter;

#endif

