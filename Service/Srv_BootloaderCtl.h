#ifndef __SRV_BOOTLOADERCTL_H
#define __SRV_BOOTLOADERCTL_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "YModem.h"

typedef struct
{
    uint16_t total_size;
    uint16_t cur_size;
    uint8_t *buf;
} SrvBootloader_Stream_TypeDef;

typedef void (*SrvBootloader_Send_Func)(uint8_t *p_buf, uint16_t len);

typedef enum
{
    SrvBootloader_Reboot_None = 0,
    SrvBootloader_Reboot_Immediately,
    SrvBootloader_OnRepower,
} SrvBootloader_Reboot_Type_List;

typedef struct
{
    bool is_actived;
    bool ready_to_rec;
    SrvBootloader_Reboot_Type_List reboot_type;
    SrvBootloader_Send_Func send;
} SrvBootloader_State_TypeDef;

typedef enum
{
    SrvBootloader_Frame_None = 0,
    SrvBootloader_Frame_YModem,
    SrvBootloader_Frame_Sum,
} SrvBootloader_ProtoFrameType_List;

typedef struct
{
    uint32_t port_addr;
    uint8_t frame_type;
    void *FrameObj;
    void *FrmaeApi;
} SrvBootloader_Adapter_TypeDef;

typedef struct
{
    bool (*init)(uint16_t stream_size);
    void (*set_send)(SrvBootloader_Send_Func send_cb);
    bool (*is_active)(void);
    void (*polling)(void);
    void (*parse)(SrvBootloader_Stream_TypeDef *p_stream);
} SrvBootloaderCtl_TypeDef;

extern SrvBootloaderCtl_TypeDef SrvBoot;

#endif

