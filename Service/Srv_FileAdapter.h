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
    FileType_None = 0,
    FileType_APP,
    FileType_Boot,
} Firmware_FileType_List;

typedef enum
{
    SrvFileAdapter_Frame_None = 0,
    SrvFileAdapter_Frame_YModem,
    SrvFileAdapter_Frame_Sum,
} Adapter_ProtoType_List;

typedef enum
{
    Adapter_Idle = 0,
    Adapter_Processing,
    Adapter_Proc_Done,
    Adapter_Proc_Failed,
} Adapter_Polling_State;

typedef enum
{
    Pack_Invalid = 0,
    Pack_InCompelete,
    Pack_Compelete,
    Pack_Unknow_State,
} Adapter_PackState_List;

typedef struct
{
    Firmware_FileType_List File_Type;
    Adapter_ProtoType_List Adapter_Type;
    uint8_t SW_Ver[3];
    uint8_t HW_Ver[3];
    uint32_t File_Size;
} FileInfo_TypeDef;

typedef struct
{
    uint32_t sys_time;
    Adapter_ProtoType_List frame_type;

    void *FrameObj;
    void *FrameApi;

    bool chancel;
    bool ready_to_rec;
    void *stream_out;
    
    FileInfo_TypeDef file_info;
    uint32_t store_addr_offset;
} SrvFileAdapterObj_TypeDef;

typedef struct
{
    SrvFileAdapterObj_TypeDef* (*create)(Adapter_ProtoType_List proto_type, FileInfo_TypeDef file_info);
    bool (*push_to_stream)(uint8_t *p_buf, uint16_t size);
    bool (*destory)(SrvFileAdapterObj_TypeDef *p_Adapter);
    void (*set_send)(SrvFileAdapterObj_TypeDef *p_Adapter, SrvFileAdapter_Send_Func send_cb);
    Adapter_Polling_State (*polling)(uint32_t sys_time, SrvFileAdapterObj_TypeDef *p_Adapter);
    FileInfo_TypeDef (*get_file_info)(SrvFileAdapterObj_TypeDef *p_Adapter);
} SrvFileAdapter_TypeDef;

extern SrvFileAdapter_TypeDef SrvFileAdapter;

#endif

