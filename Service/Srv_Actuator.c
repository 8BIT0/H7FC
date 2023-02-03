#include "Srv_Actuator.h"
#include "datapath.h"
#include "mmu.h"

SrvActuatorObj_TypeDef SrvActuator_Obj;

static bool SrvActuator_SetModel(SrvActuator_Model_List model)
{
    uint8_t i = 0;
    memset(&SrvActuator_Obj, 0, sizeof(SrvActuator_Obj));

    switch(SrvActuator_Obj.model)
    {
        case Model_Quad:
            SrvActuator_Obj.drive_component = QUAD_CONTROL_COMPONENT;
            break;

        case Model_Hex:
            SrvActuator_Obj.drive_component = HEX_CONTROL_COMPONENT;
            break;

        case Model_Oct:
            SrvActuator_Obj.drive_component = OCT_CONTROL_COMPONENT;
            break;

        case Model_X8:
            SrvActuator_Obj.drive_component = X8_CONTROL_COMPONENT;
            break;

        case Model_Y6:
            SrvActuator_Obj.drive_component = Y6_CONTROL_CONPONENT;
            break;

        case Model_Tri:
            SrvActuator_Obj.drive_component = TRI_CONTROL_COMPONENT;
            break;

        case Model_TDrone:
            SrvActuator_Obj.drive_component = TDRONE_CONTROL_COMPONENT;
            break;

        default:
            return false;
    }

    /* read in storage */
    /* current use default */
    SrvActuator_Obj.model = model;

    return true;
}



