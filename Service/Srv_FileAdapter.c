/* 
 * Auther: 8_B!T0
 * bref: Use YMODEM receive firmware
 * 
 */
#include "Srv_FileAdapter.h"
#include "DataPipe.h"
#include "Srv_OsCommon.h"
#include "../System/storage/Storage.h"
#include "YModem.h"
#include "reboot.h"

/* internal function */
static bool SrvFileAdapterObj_Check(SrvFileAdapterObj_TypeDef *p_Adapter);

/* external function */
static bool SrvFileAdapter_Init(uint16_t stream_size);
static void SrvFileAdapter_Set_Send(SrvFileAdapter_Send_Func send);
static bool SrvFileAdapter_Get_ActiveState(SrvFileAdapterObj_TypeDef *p_Adapter);

/* external virable */
SrvFileAdapter_TypeDef SrvFileAdapter = {
    .init = SrvFileAdapter_Init,
    .set_send = SrvFileAdapter_Set_Send,
    .is_active = SrvFileAdapter_Get_ActiveState,
};

static bool SrvFileAdapterObj_Check(SrvFileAdapterObj_TypeDef *p_Adapter)
{
    if (p_Adapter && \
        (p_Adapter->frame_type > SrvFileAdapter_Frame_None) && \
        (p_Adapter->frame_type < SrvFileAdapter_Frame_Sum) && 
        (p_Adapter->FrameObj == NULL) && \
        (p_Adapter->FrmaeApi == NULL))
        return false;

    return true;
}

static bool SrvFileAdapter_Init(uint16_t stream_size)
{
    if (stream_size == 0)
        return false;



    return true;
}

static SrvFileAdapterObj_TypeDef* SrvFileAdapter_Create_AdapterObj(SrvFileAdapter_ProtoFrameType_List frame_type)
{
    SrvFileAdapterObj_TypeDef *p_AdapterObj = NULL;

    p_AdapterObj = SrvOsCommon.malloc(sizeof(SrvFileAdapterObj_TypeDef));
    if (p_AdapterObj == NULL)
    {
        SrvOsCommon.free(p_AdapterObj);
    }
    else
    {
        p_AdapterObj->port_addr = 0;
        p_AdapterObj->frame_type = frame_type;

        switch (frame_type)
        {
            case SrvFileAdapter_Frame_YModem:

                break;

            default:
                SrvOsCommon.free(p_AdapterObj);
                p_AdapterObj = NULL;
                break;
        }
    }

    return p_AdapterObj;
}

static bool SrvFileAdapter_Distory_AdapterObj(SrvFileAdapterObj_TypeDef *p_Adapter)
{
    if (p_Adapter)
    {
    
    }

    return false;
}

static bool SrvFileAdapter_Get_ActiveState(SrvFileAdapterObj_TypeDef *p_Adapter)
{
    if (p_Adapter)
    {

    }

    return false;
}

static void SrvFileAdapter_Set_Send(SrvFileAdapter_Send_Func send)
{
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

