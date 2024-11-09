#include "controller.h"
#include "../../System/storage/Storage.h"
#include "Att_Casecade_PID.h"

/* test code */
#include "Srv_DataHub.h"
#include "shell_port.h"
/* test code */

#define ATTITUDE_PID_PARAM_SEC_NAME "pid_att"
#define ALTITUDE_PID_PARAM_SEC_NAME "pid_alt"

typedef enum
{
    TP_Pitch = 0,
    TP_Roll,
    TP_GyroX,
    TP_GyroY,
    TP_GyroZ,
} ControllerTunningPart_List;

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

    return Att_CasecadePID_Controller.set(pid_param);
}

static bool Controller_Alt_Init(ControlMode_List mode)
{
    return false;
}

/****************************************************************** CLI section *****************************************************************************/
static void Controller_Show_PID_Param(Shell *obj, PID_Param_TypeDef param)
{
    if (obj == NULL)
        return;

    shellPrint(obj, "\tP: %f\r\n", param.gP);
    shellPrint(obj, "\tI: %f\r\n", param.gI);
    shellPrint(obj, "\tD: %f\r\n", param.gD);
}

static void Controller_Show_AttPID_Param(Shell *obj, AttCaseCadePID_Param_TypeDef param)
{
    if (obj == NULL)
        return;

    shellPrint(obj, "[ ---- Pitch Parameter ---- ]\r\n");
    Controller_Show_PID_Param(obj, param.Pitch_Para);

    shellPrint(obj, "[ ---- Roll Parameter ---- ]\r\n");
    Controller_Show_PID_Param(obj, param.Roll_Para);

    shellPrint(obj, "[ ---- GyroX Parameter ---- ]\r\n");
    Controller_Show_PID_Param(obj, param.GyroX_Para);

    shellPrint(obj, "[ ---- GyroY Parameter ---- ]\r\n");
    Controller_Show_PID_Param(obj, param.GyroY_Para);

    shellPrint(obj, "[ ---- GyroZ Parameter ---- ]\r\n");
    Controller_Show_PID_Param(obj, param.GyroZ_Para);
}

static void Controller_AttPID_Tune_CLI(uint8_t part, float P, float I, float D)
{
    Shell *shell_obj = Shell_GetInstence();
    AttCaseCadePID_Param_TypeDef inuse;
    AttCaseCadePID_Param_TypeDef inuse_lst;
    PID_Param_TypeDef *selected = NULL;

    if (shell_obj == NULL)
        return;

    memset(&inuse, 0, sizeof(AttCaseCadePID_Param_TypeDef));
    memset(&inuse_lst, 0, sizeof(AttCaseCadePID_Param_TypeDef));
 
    inuse = Att_CasecadePID_Controller.cur_param();
    inuse_lst = inuse;

    shellPrint(shell_obj, "[ ---- Attitude Casecade PID Tunning ---- ]\r\n");
    shellPrint(shell_obj, "[ 1st parameter 1ndicate to selecetd part ]\r\n");
    shellPrint(shell_obj, "  ---- Part 0 ----- Pitch Param\r\n");
    shellPrint(shell_obj, "  ---- Part 1 ----- Roll  Param\r\n");
    shellPrint(shell_obj, "  ---- Part 2 ----- GyroX Param\r\n");
    shellPrint(shell_obj, "  ---- Part 3 ----- GyroY Param\r\n");
    shellPrint(shell_obj, "  ---- Part 4 ----- GyroZ Param\r\n");

    switch (part)
    {
        case TP_Pitch:
            shellPrint(shell_obj, " ---- Pitch is selected\r\n");
            selected = &inuse.Pitch_Para;
            break;

        case TP_Roll:
            shellPrint(shell_obj, " ---- Roll is selected\r\n");
            selected = &inuse.Roll_Para;
            break;

        case TP_GyroX:
            shellPrint(shell_obj, " ---- GyroX is selected\r\n");
            selected = &inuse.GyroX_Para;
            break;

        case TP_GyroY:
            shellPrint(shell_obj, " ---- GyroY is selected\r\n");
            selected = &inuse.GyroY_Para;
            break;

        case TP_GyroZ:
            shellPrint(shell_obj, " ----- GyroZ is selected\r\n");
            selected = &inuse.GyroZ_Para;
            break;

        default: shellPrint(shell_obj, "[ ---- Invalid Part ---- ]\r\n"); return;
    }

    /* show input parameter */
    shellPrint(shell_obj, " ---- Input P: %f\r\n", P);
    shellPrint(shell_obj, " ---- Input I: %f\r\n", I);
    shellPrint(shell_obj, " ---- Input D: %f\r\n", D);

    selected->gP = P;
    selected->gI = I;
    selected->gD = D;

    if (!Att_CasecadePID_Controller.set(inuse))
    {
        shellPrint(shell_obj, "[ ---- parameter set failed ---- ]\r\n");
        Att_CasecadePID_Controller.set(inuse_lst);
        return;
    }

    /* storage parameter */
    if (!Controller_PID_AttParam_Set((uint8_t *)&inuse, sizeof(AttCaseCadePID_Param_TypeDef)))
    {
        shellPrint(shell_obj, "[ ---- parameter save failed ---- ]\r\n");
        return;
    }

    /* display inuse parameter */
    memset(&inuse, 0, sizeof(AttCaseCadePID_Param_TypeDef));
    shellPrint(shell_obj, "[ ---- get inuse parameter ---- ]\r\n");
    inuse = Att_CasecadePID_Controller.cur_param();
    Controller_Show_AttPID_Param(shell_obj, inuse);

    /* display storaged paramter */
    memset(&inuse, 0, sizeof(AttCaseCadePID_Param_TypeDef));
    shellPrint(shell_obj, "[ ---- get storaged parameter ---- ]\r\n");
    if (Storage.get(Para_User, ControllerMonitor.Att_SSO.item, (uint8_t *)&inuse, sizeof(AttCaseCadePID_Param_TypeDef)) != Storage_Error_None)
    {
        shellPrint(shell_obj, "[ ---- read storage error ---- ]\r\n");
        return;
    }
    Controller_Show_AttPID_Param(shell_obj, inuse);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, tune_att_pid, Controller_AttPID_Tune_CLI, tune attitude control parameter);
