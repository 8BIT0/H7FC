#include "Srv_ComProto.h"

SrvComProto_Monitor_TypeDef monitor;

static bool Srv_ComProto_Init(SrvComProto_Type_List type, uint8_t *arg)
{
    memset(&monitor, 0, sizeof(monitor));
    monitor.Proto_Type = type;

    return true;
}