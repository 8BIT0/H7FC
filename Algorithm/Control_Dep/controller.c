#include "controller.h"
#include "../../System/storage/Storage.h"
#include "Att_Casecade_PID.h"

/* test code */
#include "shell_port.h"
/* test code */

#define ATTITUDE_PID_PARAM_SEC_NAME "pid_att"
#define ALTITUDE_PID_PARAM_SEC_NAME "pid_alt"

typedef struct
{
    Storage_ItemSearchOut_TypeDef Att_SSO;  /* attitude control parameter section search out */
    Storage_ItemSearchOut_TypeDef Alt_SSO;  /* altitude control parameter section search out */

    ControlMode_List att_ctl_mode;
} ControllerMonitor_TypeDef;

/* internanl vriable */
ControllerMonitor_TypeDef ControllerMonitor;

/* internal function */
/* PID controller section */
static bool Controller_PID_AttControl_ParamLoad(void);
static bool Controller_PID_AttParam_Set(uint8_t *p_param, uint16_t size);

/* external function */
/* attitude section */
static bool Controller_Att_Init(ControlMode_List mode);
static bool Controller_AttControl(ControlMode_List mode, uint32_t sys_ms, bool angular_only, AttControl_In_TypeDef exp, AttControl_In_TypeDef mea, AngControl_Out_TypeDef *out);
static bool Controller_Set_Param(bool ARM, ControlTarget_List target, ControlMode_List mode, uint8_t *p_param, uint16_t size);

/* altitude section */
static bool Controller_Alt_Init(ControlMode_List mode);

Control_TypeDef Controller = {
    .att_ctl_init = Controller_Att_Init,
    .alt_ctl_init = Controller_Alt_Init,
    
    .att_param_set = Controller_Set_Param,

    .att_ctl = Controller_AttControl,
};

static bool Controller_Get_AttParam(ControlMode_List *cur_mode, uint8_t *p_data, uint16_t *para_len)
{
    if ((cur_mode == NULL) || \
        (p_data == NULL) || \
        (para_len == NULL))
        return false;

    *cur_mode = ControllerMonitor.att_ctl_mode;

    return false;
}

static bool Controller_Set_AttParam(ControlMode_List mode, uint8_t *p_data, uint16_t len)
{
    if ((p_data == NULL) || \
        (len == 0))
        return false;

    switch (mode)
    {
        case CtlM_PID: /* attitude casecade pid tunning */ return true;
        default: return false;
    }

    return false;
}

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
    ControllerMonitor.att_ctl_mode = mode;
    return false;
}

static bool Controller_Set_Param(bool ARM, ControlTarget_List target, ControlMode_List mode, uint8_t *p_param, uint16_t size)
{
    /* noticed: only when drone is under ARM state, parameter is setable */
    /* check drone state */
    if (!ARM || \
        (target > CtlT_Altitude) || \
        (mode >= CtlM_All) || \
        (p_param == NULL) || \
        (size == 0))
        return false;

    if (target == CtlT_Attitude)
    {
        switch (mode)
        {
            case CtlM_PID: return Controller_PID_AttParam_Set(p_param, size);
            default: return false;
        }
    }
    else if (target == CtlT_Altitude)
    {
        /* still in developping */
        return false;
    }

    return false;
}

/****************************************************************** pid controller section *****************************************************************************/
static bool Controller_PID_AttParam_Set(uint8_t *p_param, uint16_t size)
{
    if (size != ATT_CASECADE_PID_PARAM_SIZE)
        return false;
    
    if (!Att_CasecadePID_Controller.set(*TO_ATT_CASECADE_PID_PARA_PTR(p_param)))
        return false;
            
    /* storage parameter */
    if (Storage.update(Para_User, ControllerMonitor.Att_SSO.item_addr, p_param, size) != Storage_Error_None)
        return false;

    return true;
}

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

    /* test code */
    /* set parameter */
    pid_param.GyroX_Para.gP = 2.4;
    pid_param.GyroX_Para.gI = 0.0;
    pid_param.GyroX_Para.gI_Max = 100;
    pid_param.GyroX_Para.gI_Min = -100;
    pid_param.GyroX_Para.gD = 0.0;//2.8;

    pid_param.GyroY_Para.gP = 2.4;
    pid_param.GyroY_Para.gI = 0.0;
    pid_param.GyroY_Para.gI_Max = 100;
    pid_param.GyroY_Para.gI_Min = -100;
    pid_param.GyroY_Para.gD = 0.0;//2.8;
    /* test code */

    return Att_CasecadePID_Controller.init(pid_param);
}

static bool Controller_Alt_Init(ControlMode_List mode)
{
    return false;
}

