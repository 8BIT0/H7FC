#include "controller.h"

/* internal function */
static bool Check_Controller_Mode(ControlMode_List mode);

static bool Check_Controller_Mode(ControlMode_List mode)
{
    /* in developping currently only support PID */
    // if (mode >= CtlM_All)
    if (mode >= CtlM_LADRC)
        return false;

    return true;
}

