#ifndef __SRV_ACTUATOR_H
#define __SRV_ACTUATOR_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "bsp_gpio.h"

#define MAX_PWM_OUT_SIG_CHANNEL_CNT 12

#define QUAD_CONTROL_COMPONENT \
            (SrvActuator_ModelComponentNum_TypeDef){4, 4, 0}
#define HEX_CONTROL_COMPONENT \
            (SrvActuator_ModelComponentNum_TypeDef){6, 6, 0}
#define OCT_CONTROL_COMPONENT \
            (SrvActuator_ModelComponentNum_TypeDef){8, 8, 0}
#define X8_CONTROL_COMPONENT \
            (SrvActuator_ModelComponentNum_TypeDef){8, 8, 0}
#define Y6_CONTROL_CONPONENT \
            (SrvActuator_ModelComponentNum_TypeDef){6, 6, 0}
#define TRI_CONTROL_COMPONENT \
            (SrvActuator_ModelComponentNum_TypeDef){4, 3, 1}
#define TDRONE_CONTROL_COMPONENT \
            (SrvActuator_ModelComponentNum_TypeDef){5, 2, 3}

typedef enum
{
    Model_Quad = 0,
    Model_Hex,
    Model_Oct,
    Model_X8,
    Model_Y6,
    Model_Tri,
    Model_TDrone,
} SrvActuator_Model_List;

typedef enum
{
    Actuator_PWM_Sig1 = 0,
    Actuator_PWM_Sig2,
    Actuator_PWM_Sig3,
    Actuator_PWM_Sig4,
    Actuator_PWM_Sig5,
    Actuator_PWM_Sig6,
    Actuator_PWM_Sig7,
    Actuator_PWM_Sig8,
    Actuator_PWM_Sig9,
    Actuator_PWM_Sig10,
    Actuator_PWM_Sig11,
    Actuator_PWM_Sig12,
} SrvActuator_MotoTag_List;

typedef enum
{
    SrvActuator_Tag_Moto = 0,
    SrvActuator_Tag_Servo,
} SrvActuator_ComponentTag_TypeDef;

#pragma pack(1)
typedef enum
{
    ESC_Type_PWM = 0,
    ESC_Type_Dshot_150,
    ESC_Type_Dshot_300,
    ESC_Type_Dshot_600,
    ESC_Type_Dshot_1200,
    ESC_Type_OneShot,
} SrvActuator_ESC_Type_List;

typedef struct
{
    uint8_t sig_id;
    uint8_t tag;
    uint8_t drv_type;

    uint16_t ctl_val;
    uint16_t max_val;
    uint16_t min_val;
    uint16_t idle_val;
    uint16_t lock_val;

    uint32_t tim_base;
    uint32_t tim_channel;
    uint32_t dma;
    uint32_t dma_channel;

    void *drv_obj;
    BspGPIO_Obj_TypeDef pin;
} SrvActuator_PWMOutObj_TypeDef;

typedef struct
{
    uint8_t total_cnt;
    uint8_t moto_cnt;
    uint8_t servo_cnt;
} SrvActuator_ModelComponentNum_TypeDef;

typedef struct
{
    SrvActuator_ModelComponentNum_TypeDef num;
    SrvActuator_PWMOutObj_TypeDef *obj_list;
} SrcActuatorCTL_Obj_TypeDef;

typedef struct
{
    SrvActuator_Model_List model;
    SrcActuatorCTL_Obj_TypeDef drive_module;
} SrvActuatorObj_TypeDef;
#pragma pak()


#endif
