/* 
 * Auther: 8_B!T0
 * bref: Use YMODEM receive firmware
 * 
 */
#include "Srv_FileAdapter.h"
#include "Srv_OsCommon.h"
#include "../System/storage/Storage.h"
#include "YModem.h"

#define ADAPTER_INPUT_BUFF_SIZE 2048

typedef struct
{
    uint8_t buf[ADAPTER_INPUT_BUFF_SIZE];
    uint16_t size;
    uint16_t total_size;
} Adapter_InputStream_TypeDef;

/* internal vriable */
static Adapter_InputStream_TypeDef AdapterInStream = {
    .size = 0,
    .total_size = ADAPTER_INPUT_BUFF_SIZE,
};

/* external function */
static SrvFileAdapterObj_TypeDef* SrvFileAdapter_Create_AdapterObj(Adapter_ProtoType_List proto_type);
static bool SrvFileAdapter_Destory_AdapterObj(SrvFileAdapterObj_TypeDef *p_Adapter);
static void SrvFileAdapter_Set_SendCallback(SrvFileAdapterObj_TypeDef *p_Adapter, SrvFileAdapter_Send_Func send);
static Adapter_Polling_State SrvFileAdapter_Polling(uint32_t sys_time, SrvFileAdapterObj_TypeDef *p_Adapter);
static bool SrvFileAdapter_PushToStream(uint8_t *p_buf, uint16_t size);

/* external virable */
SrvFileAdapter_TypeDef SrvFileAdapter = {
    .create = SrvFileAdapter_Create_AdapterObj,
    .destory = SrvFileAdapter_Destory_AdapterObj,
    .push_to_stream = SrvFileAdapter_PushToStream,
    .polling = SrvFileAdapter_Polling,
    .set_send = SrvFileAdapter_Set_SendCallback,
};

static SrvFileAdapterObj_TypeDef* SrvFileAdapter_Create_AdapterObj(Adapter_ProtoType_List proto_type)
{
    SrvFileAdapterObj_TypeDef *p_AdapterObj = NULL;
    
    p_AdapterObj = SrvOsCommon.malloc(sizeof(SrvFileAdapterObj_TypeDef));
    if (p_AdapterObj == NULL)
        SrvOsCommon.free(p_AdapterObj);
    
    p_AdapterObj->port_addr = 0;
    p_AdapterObj->frame_type = proto_type;

    switch (proto_type)
    {
        case SrvFileAdapter_Frame_YModem:
            /* create YModem Object */
            p_AdapterObj->FrameObj = SrvOsCommon.malloc(YModemObj_size);
            if (p_AdapterObj->FrameObj == NULL)
            {
                SrvOsCommon.free(p_AdapterObj);
                p_AdapterObj = NULL;
            }
            
            p_AdapterObj->FrameApi = (void *)&YModem;
            break;

        default:
            SrvOsCommon.free(p_AdapterObj);
            p_AdapterObj = NULL;
            break;
    }

    return p_AdapterObj;
}

static bool SrvFileAdapter_Destory_AdapterObj(SrvFileAdapterObj_TypeDef *p_Adapter)
{
    if (p_Adapter && p_Adapter->FrameObj)
    {
        SrvOsCommon.free(p_Adapter->FrameObj);
        memset(p_Adapter, 0, sizeof(SrvFileAdapterObj_TypeDef));
        SrvOsCommon.free(p_Adapter);
        return true;
    }

    return false;
}

static void SrvFileAdapter_Set_SendCallback(SrvFileAdapterObj_TypeDef *p_Adapter,  SrvFileAdapter_Send_Func send)
{
    if (p_Adapter && p_Adapter->FrameObj)
    {
        switch ((uint8_t) p_Adapter->frame_type)
        {
            case SrvFileAdapter_Frame_YModem:
                To_YModem_Api(p_Adapter->FrameApi)->set_callback(To_YModem_Obj(p_Adapter->FrameObj), YModem_Callback_Type_Send, send);
                break;

            default: break;
        }
    }
}

static bool SrvFileAdapter_PushToStream(uint8_t *p_buf, uint16_t size)
{
    if (p_buf && size && (AdapterInStream.size + size <= AdapterInStream.total_size))
    {
        /* push data into adapter input stream */
        memcpy(&AdapterInStream.buf[AdapterInStream.size], p_buf, size);
        AdapterInStream.size += size;

        return true;
    }

    return false;
}

static Adapter_Polling_State SrvFileAdapter_Polling(uint32_t sys_time, SrvFileAdapterObj_TypeDef *p_Adapter)
{
    void *p_api = NULL;
    void *p_obj = NULL;
    bool clear_stream = false;

    if (p_Adapter && p_Adapter->FrameObj && p_Adapter->FrameApi)
    {
        p_api = p_Adapter->FrameApi;
        p_obj = p_Adapter->FrameObj;
        
        switch ((uint8_t)p_Adapter->frame_type)
        {
            case SrvFileAdapter_Frame_YModem:
                if (p_Adapter->stream_out == NULL)
                {
                    p_Adapter->stream_out = SrvOsCommon.malloc(YModem_Stream_Size);
                    if (p_Adapter->stream_out == NULL)
                    {
                        SrvOsCommon.free(p_Adapter->stream_out);
                        return Adapter_Proc_Failed;
                    }
                }

                if (To_YModem_Api(p_api)->polling)
                {
                    To_YModem_Api(p_api)->polling(sys_time, To_YModem_Obj(p_obj), AdapterInStream.buf, AdapterInStream.size, To_YModem_Stream(p_Adapter->stream_out));
                    switch ((uint8_t)To_YModem_Stream(p_Adapter->stream_out)->valid)
                    {
                        case YModem_Pack_Compelete:       
                            /* if stream valid write file to storage section */
                            
                            clear_stream = true;
                            break;

                        default: break;
                    }
                }
                break;

            default: break;
        }

        if (clear_stream)
        {
            memset(AdapterInStream.buf, 0, AdapterInStream.size);
            AdapterInStream.size = 0;
        }
    }

    return Adapter_Proc_Failed;
}

