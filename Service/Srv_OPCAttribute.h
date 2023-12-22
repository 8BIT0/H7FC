#ifndef __SRV_OPCATTRIBUTE_H
#define __SRV_OPCATTRIBUTE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "Srv_DataHub.h"
#include "Bsp_Uart.h"

typedef struct
{
    bool init_state;
}SrvOPC_Monitor_TypeDef;

typedef struct
{
    bool (*init)(void *port_obj, void *port_api);
    bool (*taking_over_req)(void);
}SrvOPC_TypeDef;

extern SrvOPC_TypeDef SrvOPC;

#endif
