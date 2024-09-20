#include "controller.h"
#include "storage.h"
#include "shell_port.h"
#include "Att_Casecade_PID.h"

__attribute__((weak)) void* Controller_Malloc(uint32_t size){return NULL;};
__attribute__((weak)) void Controller_Free(void *ptr){};

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
static bool Check_Controller_Mode(ControlMode_List mode);
static bool Controller_PID_AttControl_ParamLoad(Control_DataObj_TypeDef *obj);

static bool Check_Controller_Mode(ControlMode_List mode)
{
    /* in developping currently only support PID */
    // if (mode >= CtlM_All)
    if (mode >= CtlM_LADRC)
        return false;

    return true;
}

static bool Controller_Att_Init(Control_DataObj_TypeDef *obj)
{
    memset(&ControllerMonitor.Att_SSO, 0, sizeof(Storage_ItemSearchOut_TypeDef));

    if ((obj == NULL) || \
        !Check_Controller_Mode(obj->CtlMode))
        return false;

    if (obj->p_para_stream == NULL)
    {
        obj->p_para_stream = Controller_Malloc(sizeof(Control_DataObj_TypeDef));
        if (obj->p_para_stream == NULL)
            return false;
    
        memset(obj->p_para_stream, 0, sizeof(Control_DataObj_TypeDef));
    }

    switch ((uint8_t) obj->CtlMode)
    {
        case CtlM_PID:
            if (!Controller_PID_AttControl_ParamLoad(obj))
                return false;
            return true;

        default: return false;
    }

    return true;
}

static bool Controller_PID_AttControl_ParamLoad(Control_DataObj_TypeDef *obj)
{
    if ((obj == NULL) || (obj->p_para_stream == NULL))
        return false;

    obj->p_para_stream->size = ATT_CASECADE_PID_PARAM_SIZE;
    obj->p_para_stream->p_para = Controller_Malloc(ATT_CASECADE_PID_PARAM_SIZE);
    if (obj->p_para_stream->p_para == NULL)
    {
        obj->p_para_stream->size = 0;
            Controller_Free(obj->p_para_stream);
        return false;
    }
    memset(obj->p_para_stream->p_para, 0, ATT_CASECADE_PID_PARAM_SIZE);

    /* load parameter */
    // Storage.search();

    return true;
}
