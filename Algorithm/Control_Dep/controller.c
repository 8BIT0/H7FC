#include "controller.h"
#include "../../System/storage/Storage.h"
#include "shell_port.h"
#include "Att_Casecade_PID.h"

#define ATTITUDE_PID_PARAM_SEC_NAME "pid_att"
#define ALTITUDE_PID_PARAM_SEC_NAME "pid_alt"

typedef struct
{
    Storage_ItemSearchOut_TypeDef Att_SSO;  /* attitude control parameter section search out */
    Storage_ItemSearchOut_TypeDef Alt_SSO;  /* altitude control parameter section search out */
} ControllerMonitor_TypeDef;

/* internanl vriable */
ControllerMonitor_TypeDef ControllerMonitor;

/* internal function */
/* PID controller section */
static bool Controller_PID_AttControl_ParamLoad(void);

/* external function */
/* attitude section */
static bool Controller_Att_Init(ControlMode_List mode);
static bool Controller_AttControl(ControlMode_List mode, uint32_t sys_ms, bool angular_only, AttControl_In_TypeDef exp, AttControl_In_TypeDef mea, AngControl_Out_TypeDef *out);

/* altitude section */
static bool Controller_Alt_Init(ControlMode_List mode);

Control_TypeDef Controller = {
    .att_ctl_init = Controller_Att_Init,
    .alt_ctl_init = Controller_Alt_Init,
    
    .att_ctl = Controller_AttControl,
};

static bool Controller_Att_Init(ControlMode_List mode)
{
    memset(&ControllerMonitor.Att_SSO, 0, sizeof(Storage_ItemSearchOut_TypeDef));

    switch ((uint8_t) mode)
    {
        case CtlM_PID: return Controller_PID_AttControl_ParamLoad();
        default: return false;
    }

    return false;
}

static bool Controller_AttControl(ControlMode_List mode, uint32_t sys_ms, bool angular_only, AttControl_In_TypeDef exp, AttControl_In_TypeDef mea, AngControl_Out_TypeDef *out)
{
    switch ((uint8_t) mode)
    {
        case CtlM_PID: return Att_CasecadePID_Controller.process(sys_ms, angular_only, exp, mea, out);
        default: return false;
    }

    return false;
}

/****************************************************************** pid controller section *****************************************************************************/
static bool Controller_PID_AttControl_ParamLoad(void)
{
    Storage_ErrorCode_List stor_err = Storage_Error_None;
    AttCaseCadePID_Param_TypeDef pid_param;

    if (Att_CasecadePID_Controller.default_param == NULL)
        return false;

    pid_param = Att_CasecadePID_Controller.default_param();

    /* load parameter */
    ControllerMonitor.Att_SSO = Storage.search(Para_User, ATTITUDE_PID_PARAM_SEC_NAME);
    if (ControllerMonitor.Att_SSO.item_addr == 0)
    {
        /* no section found */
        /* create pid attitude controller parameter section in storage */
        stor_err = Storage.create(Para_User, ATTITUDE_PID_PARAM_SEC_NAME, (uint8_t *)&pid_param, ATT_CASECADE_PID_PARAM_SIZE);
        if (stor_err != Storage_Error_None)
            return false;
    }
    else
    {
        /* section found */
        stor_err = Storage.get(Para_User, ControllerMonitor.Att_SSO.item, (uint8_t *)&pid_param, ATT_CASECADE_PID_PARAM_SIZE);
        if (stor_err != Storage_Error_None)
        {
            /* set parameter as default */
            pid_param = Att_CasecadePID_Controller.default_param();
            return false;
        }
    }

    /* test */
    /* set parameter */
    /* test */

    return Att_CasecadePID_Controller.init(pid_param);
}

static bool Controller_Alt_Init(ControlMode_List mode)
{
    return false;
}

