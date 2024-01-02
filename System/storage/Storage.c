#include "Storage.h"

/* internal vriable */
Storage_Monitor_TypeDef Storage_Monitor;

static bool Storage_Init(Storage_ModuleState_TypeDef enable)
{
    memset(&Storage_Monitor, 0, sizeof(Storage_Monitor));

    Storage_Monitor.module_enable_reg.val = enable.val;
    Storage_Monitor.module_init_reg.val = 0;

    if(enable.bit.internal && BspFlash.init && BspFlash.init())
    {
        /* on chip flash init */
    }

    /* still in developping */
    if(enable.bit.external)
    {
        /* external flash init */
    }

    Storage_Monitor.init_state = Storage_Monitor.module_init_reg.bit.external | Storage_Monitor.module_init_reg.bit.internal;

    return Storage_Monitor.init_state;
}

static bool Storage_Build_StorageInfo()
{
    
}

