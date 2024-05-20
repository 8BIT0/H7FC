/* 
 * Auther: 8_B!T0
 * bref: Use YMODEM receive firmware
 * 
 */
#include "Srv_FileAdapter.h"
#include "Srv_OsCommon.h"
#include "../System/storage/Storage.h"
#include "YModem.h"

/* external function */
static SrvFileAdapterObj_TypeDef* SrvFileAdapter_Create_AdapterObj(Adapter_ProtoType_List proto_type);
static bool SrvFileAdapter_Destory_AdapterObj(SrvFileAdapterObj_TypeDef *p_Adapter);
static void SrvFileAdapter_Set_SendCallback(SrvFileAdapterObj_TypeDef *p_Adapter, SrvFileAdapter_Send_Func send);
static Adapter_Polling_State SrvFileAdapter_Polling(SrvFileAdapterObj_TypeDef *p_Adapter, uint8_t *p_buf, uint16_t size, Adapter_Stream_TypeDef *p_stream);

/* external virable */
SrvFileAdapter_TypeDef SrvFileAdapter = {
    .create = SrvFileAdapter_Create_AdapterObj,
    .destory = SrvFileAdapter_Destory_AdapterObj,
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

static Adapter_Polling_State SrvFileAdapter_Polling(SrvFileAdapterObj_TypeDef *p_Adapter, uint8_t *p_buf, uint16_t size, Adapter_Stream_TypeDef *p_stream)
{
    void *p_api = NULL;
    void *p_obj = NULL;
    YModem_Stream_TypeDef stream_tmp;

    if (p_Adapter && p_Adapter->FrameObj && p_Adapter->FrameApi && p_buf && size)
    {
        p_api = p_Adapter->FrameApi;
        p_obj = p_Adapter->FrameObj;
        
        switch ((uint8_t)p_Adapter->frame_type)
        {
            case SrvFileAdapter_Frame_YModem:
                memset(&stream_tmp, 0, sizeof(YModem_Stream_TypeDef));
                if (To_YModem_Api(p_api)->polling)
                    To_YModem_Api(p_api)->polling(To_YModem_Obj(p_obj), p_buf, size, &stream_tmp);
                break;

            default: break;
        }
    }

    return Adapter_Proc_Failed;
}

