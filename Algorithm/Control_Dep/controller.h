#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

typedef enum
{
    CtlM_PID = 0, /* PID      control modle */
    CtlM_LADRC,   /* LADRC    control modle reserved */
    CtlM_MUDE,    /* MUDE     control modle reserved */
    CtlM_All,
} ControlMode_List;

typedef struct
{
    void *p_para;
    uint16_t size;
} ControlParam_TypeDef;

/* attitude & angular speed control */
typedef struct
{
    ControlMode_List CtlMode;
    ControlParam_TypeDef *p_para_stream;
} Control_DataObj_TypeDef;

typedef struct
{
    bool (*att_clt_init)(Control_DataObj_TypeDef *p_obj);
    bool (*alt_clt_init)(Control_DataObj_TypeDef *p_obj);
    
    void (*att_ctl)();
    void (*alt_ctl)();
} Control_TypeDef;

extern Control_TypeDef Controller;

#ifdef __cplusplus
}
#endif

#endif
