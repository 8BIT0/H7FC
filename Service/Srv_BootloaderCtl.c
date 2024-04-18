/* 
 * Auther: 8_B!T0
 * bref: Use YMODEM receive firmware
 * 
 */
#include "Srv_BootloaderCtl.h"
#include "DataPipe.h"
#include "Srv_OsCommon.h"
#include "../System/storage/Storage.h"
#include "YModem.h"
#include "reboot.h"

/* internal virable */
static SrvBootloader_Stream_TypeDef bootloader_stream = {
    .cur_size = 0,
    .total_size = 0,
    .buf = NULL,
};

static SrvBootloader_State_TypeDef SrvBootloader_Monitor = {
    .is_actived = false,
    .ready_to_rec = false,
    .reboot_type = SrvBootloader_Reboot_None,
    .send = NULL,
};

/* external function */
static bool SrvBootloaderCtl_Init(uint16_t stream_size);
static void SrvBootloader_Set_Send(SrvBootloader_Send_Func send);
static bool SrvBootloader_Get_ActiveState(void);

/* external virable */
SrvBootloaderCtl_TypeDef SrvBoot = {
    .init = SrvBootloaderCtl_Init,
    .set_send = SrvBootloader_Set_Send,
    .is_active = SrvBootloader_Get_ActiveState,
};

static bool SrvBootloaderCtl_Init(uint16_t stream_size)
{
    if (stream_size == 0)
        return false;

    bootloader_stream.buf = SrvOsCommon.malloc(stream_size);
    if (bootloader_stream.buf)
    {
        bootloader_stream.total_size = stream_size;
        bootloader_stream.cur_size = 0;
    }
    else
    {
        SrvOsCommon.free(bootloader_stream.buf);
        return false;
    }

    return true;
}

static SrvBootloader_Adapter_TypeDef* SrvBootloader_Create_AdapterObj(SrvBootloader_ProtoFrameType_List frame_type)
{
    SrvBootloader_Adapter_TypeDef *p_AdapterObj = NULL;

    p_AdapterObj = SrvOsCommon.malloc(sizeof(SrvBootloader_Adapter_TypeDef));
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
            case SrvBootloader_Frame_YModem:

                break;

            default:
                SrvOsCommon.free(p_AdapterObj);
                p_AdapterObj = NULL;
                break;
        }
    }

    return p_AdapterObj;
}

static bool SrvBootloader_Distory_AdapterObj(SrvBootloader_Adapter_TypeDef *p_Adapter)
{
    if (p_Adapter)
    {
        SrvOsCommon.free(p_Adapter->YModemObj);
        p_Adapter->YModemObj = NULL;
        SrvOsCommon.free(p_Adapter);
        p_Adapter = NULL;   
    }

    return false;
}

static bool SrvBootloader_Get_ActiveState(void)
{
    return SrvBootloader_Monitor.is_actived;
}

static void SrvBootloader_Set_Send(SrvBootloader_Send_Func send)
{
    SrvBootloader_Monitor.send = send;    
}

static bool SevBootloader_BindToPort(uint32_t port_addr)
{

}

