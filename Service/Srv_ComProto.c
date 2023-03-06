#include "Srv_ComProto.h"

SrvComProto_Monitor_TypeDef monitor;

/* external function */
static bool Srv_ComProto_Init(SrvComProto_Type_List type, uint8_t *arg);

SrvComProto_TypeDef SrvComProto = {
    .init = Srv_ComProto_Init,
};

static bool Srv_ComProto_Init(SrvComProto_Type_List type, uint8_t *arg)
{
    UNUSED(arg);

    memset(&monitor, 0, sizeof(monitor));
    monitor.Proto_Type = type;

    return true;
}

static bool Srv_ComProto_MsgObj_Init(SrvComProto_MsgInfo_TypeDef *msg,
                                     uint32_t msg_id, uint32_t period,
                                     SrvComProto_IOType_List io_dir,
                                     SrvComProto_Stream_TypeDef tar_stream)
{
    if ((msg == NULL) ||
        (period == 0) ||
        (tar_stream.p_buf == NULL) ||
        (tar_stream.size == 0))
        return false;

    return true;
}
