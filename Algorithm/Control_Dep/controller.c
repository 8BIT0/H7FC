#include "controller.h"
#include "storage.h"
#include "shell.h"
#include "Att_Casecade_PID.h"

/* internal function */
static bool Check_Controller_Mode(ControlMode_List mode);
static bool Check_ParamStream(ControlMode_List mode, ControlParam_TypeDef para_stream);

static bool Check_Controller_Mode(ControlMode_List mode)
{
    /* in developping currently only support PID */
    // if (mode >= CtlM_All)
    if (mode >= CtlM_LADRC)
        return false;

    return true;
}

static bool Check_ParamStream(ControlMode_List mode, ControlParam_TypeDef para_stream)
{
    if (para_stream.p_para == NULL)
        return false;

    /* check size */
    switch ((uint8_t)mode)
    {
        /* in developping currently only support PID */
        case CtlM_PID: return (para_stream.size == ATT_CASECADE_PID_PARAM_SIZE) ? true : false;
        default: return false;
    }
}

/* load parameter from storage */
static bool Controller_Load_Param(AttControl_DataObj_TypeDef *AttCtl_Obj)
{
    if (AttCtl_Obj == NULL)
        return false;

    switch ((uint8_t)AttCtl_Obj->CtlMode)
    {
        case CtlM_PID:
            break;
    
        default: return false;
    }

    return true;
}

static bool Controller_Att_Init(AttControl_DataObj_TypeDef *AttCtl_Obj)
{
    if ((AttCtl_Obj == NULL) || \
        !Check_Controller_Mode(AttCtl_Obj->CtlMode) || \
        !Check_ParamStream(AttCtl_Obj->CtlMode, AttCtl_Obj->p_para) || \
        !Controller_Load_Param(AttCtl_Obj))
        return false;

    return true;
}
