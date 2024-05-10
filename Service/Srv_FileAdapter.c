/* 
 * Auther: 8_B!T0
 * bref: Use YMODEM receive firmware
 * 
 */
#include "Srv_FileAdapter.h"
#include "Srv_OsCommon.h"
#include "../System/storage/Storage.h"
#include "YModem.h"
#include "reboot.h"

/* internal function */
static bool SrvFileAdapterObj_Check(SrvFileAdapterObj_TypeDef *p_Adapter);

/* external function */
static SrvFileAdapterObj_TypeDef* SrvFileAdapter_Create_AdapterObj(Adapter_ProtoType_List proto_type, uint32_t stream_size);
static bool SrvFileAdapter_Destory_AdapterObj(SrvFileAdapterObj_TypeDef *p_Adapter);
static void SrvFileAdapter_Set_SendCallback(SrvFileAdapterObj_TypeDef *p_Adapter, SrvFileAdapter_Send_Func send);
static bool SrvFileAdapter_Get_ActiveState(SrvFileAdapterObj_TypeDef *p_Adapter);
static bool SrvFileAdapter_BindToPort(SrvFileAdapterObj_TypeDef *p_Adapter, uint32_t port_addr);
static void SrvFileAdapter_Parse(SrvFileAdapterObj_TypeDef *p_Adapter, const SrvFileAdapter_Stream_TypeDef stream);
static void SrvFileAdapter_Polling(SrvFileAdapterObj_TypeDef *p_Adapter);

/* external virable */
SrvFileAdapter_TypeDef SrvFileAdapter = {
    .create = SrvFileAdapter_Create_AdapterObj,
    .destory = SrvFileAdapter_Destory_AdapterObj,
    .bind_port = SrvFileAdapter_BindToPort,
    .parse = SrvFileAdapter_Parse,
    .polling = SrvFileAdapter_Polling,
    .set_send = SrvFileAdapter_Set_SendCallback,
    .is_active = SrvFileAdapter_Get_ActiveState,
};

static bool SrvFileAdapterObj_Check(SrvFileAdapterObj_TypeDef *p_Adapter)
{
    if ((p_Adapter == NULL) || \
        (p_Adapter->frame_type >= SrvFileAdapter_Frame_Sum) || \
        (p_Adapter->FrameObj == NULL) || \
        (p_Adapter->FrmaeApi == NULL))
        return false;

    return true;
}

static SrvFileAdapterObj_TypeDef* SrvFileAdapter_Create_AdapterObj(Adapter_ProtoType_List proto_type, uint32_t stream_size)
{
    SrvFileAdapterObj_TypeDef *p_AdapterObj = NULL;

    if (SrvFileAdapterObj_Check(p_AdapterObj))
    {
        p_AdapterObj = SrvOsCommon.malloc(sizeof(SrvFileAdapterObj_TypeDef));
        if (p_AdapterObj == NULL)
        {
            SrvOsCommon.free(p_AdapterObj);
        }
        else
        {
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
                    
                    p_AdapterObj->FrmaeApi = (void *)&YModem;
                    break;

                default:
                    SrvOsCommon.free(p_AdapterObj);
                    p_AdapterObj = NULL;
                    break;
            }
        }
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
        p_Adapter = NULL;
        return true;
    }

    return false;
}

static bool SrvFileAdapter_Get_ActiveState(SrvFileAdapterObj_TypeDef *p_Adapter)
{
    /* already bind to port */
    if (p_Adapter && p_Adapter->FrameObj && p_Adapter->port_addr)
        return true;

    return false;
}

static void SrvFileAdapter_Set_SendCallback(SrvFileAdapterObj_TypeDef *p_Adapter,  SrvFileAdapter_Send_Func send)
{
    if (p_Adapter && p_Adapter->FrameObj)
        p_Adapter->send = send;
}

static bool SrvFileAdapter_BindToPort(SrvFileAdapterObj_TypeDef *p_Adapter, uint32_t port_addr)
{
    if (p_Adapter && \
        (p_Adapter->port_addr == 0) && \
        (p_Adapter->frame_type > SrvFileAdapter_Frame_None) && \
        (p_Adapter->frame_type < SrvFileAdapter_Frame_Sum) && \
        (p_Adapter->FrameObj != NULL) && \
        (p_Adapter->FrmaeApi != NULL) && \
        port_addr)
    {
        p_Adapter->port_addr = port_addr;
        return true;
    }

    return false;
}

static void SrvFileAdapter_Parse(SrvFileAdapterObj_TypeDef *p_Adapter, const SrvFileAdapter_Stream_TypeDef stream)
{
    if (p_Adapter && p_Adapter->FrameObj && stream.buf && stream.total_size)
    {

    }
}

static void SrvFileAdapter_Polling(SrvFileAdapterObj_TypeDef *p_Adapter)
{
    if (p_Adapter && p_Adapter->FrameObj)
    {

    }
}

