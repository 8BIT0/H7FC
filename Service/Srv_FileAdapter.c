/* 
 * Auther: 8_B!T0
 * bref: Use YMODEM receive firmware
 * 
 */
#include "Srv_FileAdapter.h"
#include "Srv_OsCommon.h"
#include "../System/storage/Storage.h"
#include "YModem.h"
#include "HW_Def.h"
#include "debug_util.h"

#define ADAPTER_INPUT_BUFF_SIZE 2048

#define ADAPTER_INFO(fmt,...) Debug_Print(&DebugPort, "[ ADAPTER INFO ] ", fmt, ##__VA_ARGS__)

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

/* internal function */
static Adapter_Polling_State SrvAdapter_YModem_Polling(SrvFileAdapterObj_TypeDef *p_Adapter, bool *clear_stream);

/* external function */
static SrvFileAdapterObj_TypeDef* SrvFileAdapter_Create_AdapterObj(Adapter_ProtoType_List proto_type, FileInfo_TypeDef file_info);
static bool SrvFileAdapter_Destory_AdapterObj(SrvFileAdapterObj_TypeDef *p_Adapter);
static void SrvFileAdapter_Set_SendCallback(SrvFileAdapterObj_TypeDef *p_Adapter, SrvFileAdapter_Send_Func send);
static Adapter_Polling_State SrvFileAdapter_Polling(uint32_t sys_time, SrvFileAdapterObj_TypeDef *p_Adapter);
static bool SrvFileAdapter_PushToStream(uint8_t *p_buf, uint16_t size);
static FileInfo_TypeDef SrvFileAdapter_GetFileInfo(SrvFileAdapterObj_TypeDef *p_Adapter);

/* external virable */
SrvFileAdapter_TypeDef SrvFileAdapter = {
    .create = SrvFileAdapter_Create_AdapterObj,
    .destory = SrvFileAdapter_Destory_AdapterObj,
    .push_to_stream = SrvFileAdapter_PushToStream,
    .polling = SrvFileAdapter_Polling,
    .set_send = SrvFileAdapter_Set_SendCallback,
    .get_file_info = SrvFileAdapter_GetFileInfo,
};

static SrvFileAdapterObj_TypeDef* SrvFileAdapter_Create_AdapterObj(Adapter_ProtoType_List proto_type, FileInfo_TypeDef file_info)
{
    SrvFileAdapterObj_TypeDef *p_AdapterObj = NULL;
    
    p_AdapterObj = SrvOsCommon.malloc(sizeof(SrvFileAdapterObj_TypeDef));
    if (p_AdapterObj == NULL)
        SrvOsCommon.free(p_AdapterObj);
    
    p_AdapterObj->frame_type = proto_type;
    p_AdapterObj->file_info = file_info;

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
        if (p_Adapter->frame_type == SrvFileAdapter_Frame_YModem)
            memset(p_Adapter->FrameObj, 0, sizeof(YModemObj_TypeDef));

        SrvOsCommon.free(p_Adapter->FrameObj);
        memset(p_Adapter, 0, sizeof(SrvFileAdapterObj_TypeDef));
        p_Adapter->FrameApi = NULL;
        p_Adapter->FrameObj = NULL;
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
            case SrvFileAdapter_Frame_YModem: To_YModem_Obj(p_Adapter->FrameObj)->send_callback = send; break;
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

static Adapter_Polling_State SrvFileAdapter_Store_Firmware(SrvFileAdapterObj_TypeDef *p_Adapter, uint8_t *p_data, uint16_t size)
{
    uint32_t max_file_size = 0;
    Storage_FirmwareType_List firmware_type = Firmware_None;

    if ((p_Adapter == NULL) || \
        (p_Adapter->file_info.File_Type == FileType_None) || \
        (p_Adapter->file_info.File_Type > FileType_Boot) || \
        (p_data == NULL) || \
        (size == 0))
        return Adapter_Proc_Failed;

    if (p_Adapter->file_info.File_Type == FileType_Boot)
    {
        max_file_size = Boot_Section_Size;
        firmware_type = Firmware_Boot;
    }
    else if (p_Adapter->file_info.File_Type == FileType_APP)
    {
        max_file_size = App_Section_Size;
        firmware_type = Firmware_App;
    }

    /* check received file size */
    if (p_Adapter->file_info.File_Size >= max_file_size)
        /* abort file receive */
        return Adapter_Proc_Failed;
    
    /* write stream data to storage section */
    if (!Storage.write_firmware(External_Flash, firmware_type, p_Adapter->store_addr_offset, p_data, size))
        return Adapter_Proc_Failed;

    p_Adapter->file_info.File_Size += size;
    p_Adapter->store_addr_offset += size;
    return Adapter_Processing;
}

static Adapter_Polling_State SrvFileAdapter_Polling(uint32_t sys_time, SrvFileAdapterObj_TypeDef *p_Adapter)
{
    bool clear_stream = false;
    Adapter_Polling_State state = Adapter_Idle;

    if (p_Adapter && p_Adapter->FrameObj && p_Adapter->FrameApi)
    {
        p_Adapter->sys_time = sys_time;
        switch ((uint8_t)p_Adapter->frame_type)
        {
            case SrvFileAdapter_Frame_YModem:
                state = SrvAdapter_YModem_Polling(p_Adapter, &clear_stream);
                break;

            default: break;
        }
        
        if (clear_stream)
        {
            memset(AdapterInStream.buf, 0, AdapterInStream.size);
            AdapterInStream.size = 0;
        }
    }

    return state;
}

static FileInfo_TypeDef SrvFileAdapter_GetFileInfo(SrvFileAdapterObj_TypeDef *p_Adapter)
{
    FileInfo_TypeDef FileInfo;

    memset(&FileInfo, 0, sizeof(FileInfo_TypeDef));

    if (p_Adapter)
        FileInfo = p_Adapter->file_info;

    return FileInfo;
}

/******************************************************************* YModem API ***********************************************************************/
static Adapter_Polling_State SrvAdapter_YModem_Polling(SrvFileAdapterObj_TypeDef *p_Adapter, bool *clear_stream)
{
    YModem_TypeDef *p_api = NULL;
    YModemObj_TypeDef *p_obj = NULL;
    YModem_Stream_TypeDef *p_stream = NULL;

    p_api = To_YModem_Api(p_Adapter->FrameApi);
    p_obj =  To_YModem_Obj(p_Adapter->FrameObj);
    p_stream = To_YModem_Stream(p_Adapter->stream_out);

    if (p_Adapter && p_api && p_obj && clear_stream && p_api->polling)
    {
        *clear_stream = false;
        if (p_Adapter->stream_out == NULL)
        {
            p_Adapter->stream_out = SrvOsCommon.malloc(YModem_Stream_Size);
            if (p_Adapter->stream_out == NULL)
            {
                SrvOsCommon.free(p_Adapter->stream_out);
                return Adapter_Proc_Failed;
            }
        }

        switch (p_api->polling(p_Adapter->sys_time, p_obj, AdapterInStream.buf, AdapterInStream.size, p_stream))
        {
            case YModem_State_Tx: return Adapter_Processing;

            case YModem_State_Rx:
                if (p_stream->valid == YModem_Pack_InCompelete)
                    return Adapter_Processing;

                if ((p_stream->valid == YModem_Pack_Compelete) && \
                    p_stream->p_buf && \
                    (p_stream->size > 1) && \
                    p_stream->file_data)
                {
                    if (SrvFileAdapter_Store_Firmware(p_Adapter, p_stream->p_buf, p_stream->size) == Adapter_Proc_Failed)
                    {
                        /* abort YModem */
                        /* clear app storage section */
                        p_api->abort(p_obj);
                        if (p_Adapter->file_info.File_Type == FileType_APP)
                        {
                            Storage.format_firmware(Firmware_App);
                        }
                        else if (p_Adapter->file_info.File_Type == FileType_Boot)
                            Storage.format_firmware(Firmware_Boot);

                        *clear_stream = true;
                        return Adapter_Proc_Failed;
                    }
                            
                    p_stream->size = 0;
                    p_stream->valid = YModem_Pack_Default;
                }

                *clear_stream = true;
                return Adapter_Processing;

            case YModem_State_Finish:
                SrvOsCommon.free(p_Adapter->stream_out);
                p_Adapter->stream_out = NULL;
                *clear_stream = true;
                return Adapter_Proc_Done;

            case YModem_State_Error:
            default: break;
        }
    }

    return Adapter_Proc_Failed;
}
