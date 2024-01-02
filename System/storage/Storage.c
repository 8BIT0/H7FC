#include "Storage.h"

/* internal vriable */
Storage_Monitor_TypeDef Storage_Monitor;

static bool Storage_Init(Storage_ModuleState_TypeDef enable)
{
    memset(&Storage_Monitor, 0, sizeof(Storage_Monitor));

    Storage_Monitor.module_enable_reg.val = enable.val;

    if(enable.bit.internal)
    {
        /* on chip flash init */
    }

    if(enable.bit.external)
    {    
        /* external flash init */
    }

    return ;
}


