#include "Srv_Flow.h"

/* external function */
static bool SrvFlow_Init(SrvFlowObj_TypeDef *obj);

SrvFlow_TypeDef SrvFlow = {
    .init =  SrvFlow_Init,
    .get_pos = NULL,
    .get_vel = NULL,
};

static bool SrvFlow_Init(SrvFlowObj_TypeDef *obj)
{
    if(obj && (obj->type != SrvFlow_None))
    {
        switch((uint8_t)obj->type)
        {
            case SrvFlow_MATEK_3901_L0X:
                break;

            default:
                return false;
        }
    }

    return false;
}


