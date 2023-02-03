#include "Srv_Actuator.h"
#include "datapath.h"
#include "mmu.h"

static bool SrvActuator_SetModel(SrvActuator_Model_List model)
{
    switch(model)
    {
        case Model_Quad:
            break;

        case Model_Hex:
            break;

        case Model_Oct:
            break;

        case Model_X8:
            break;

        case Model_Y6:
            break;
            
        case Model_Tri:
            break;

        case Model_TDrone:
            break;

        default:
            return false;
    }

    return true;
}



