#ifndef __SRV_FILEADAPTER_H
#define __SRV_FILEADAPTER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "YModem.h"

typedef struct
{
    uint16_t total_size;
    uint16_t cur_size;
    uint8_t *buf;
} SrvFileAdapter_Stream_TypeDef;

typedef void (*SrvFileAdapter_Send_Func)(uint8_t *p_buf, uint16_t len);

typedef enum
{
    SrvFileAdapter_Frame_None = 0,
    SrvFileAdapter_Frame_YModem,
    SrvFileAdapter_Frame_Sum,
} SrvFileAdapter_ProtoFrameType_List;

typedef struct
{
    uint32_t port_addr;
    uint8_t frame_type;
    void *FrameObj;
    void *FrmaeApi;

    bool is_actived;
    bool ready_to_rec;
    SrvFileAdapter_Send_Func send;
} SrvFileAdapterObj_TypeDef;

typedef struct
{
    bool (*init)(uint16_t stream_size);
    void (*set_send)(SrvFileAdapter_Send_Func send_cb);
    bool (*is_active)(void);
    void (*polling)(void);
    void (*parse)(SrvFileAdapter_Stream_TypeDef *p_stream);
} SrvFileAdapter_TypeDef;

extern SrvFileAdapter_TypeDef SrvFileAdapter;

#endif

