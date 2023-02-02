#ifndef __TASK_CONTROL_H
#define __TASK_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define QUAD_CONTROL_COMPONENT \
            (Control_Component_TypeDef){4, 0, NULL, NULL, NULL, NULL}
#define HEX_CONTROL_COMPONENT \
            (Control_Component_TypeDef){6, 0, NULL, NULL, NULL, NULL}
#define OCT_CONTROL_COMPONENT \
            (Control_Component_TypeDef){8, 0, NULL, NULL, NULL, NULL}
#define X8_CONTROL_COMPONENT \
            (Control_Component_TypeDef){8, 0, NULL, NULL, NULL, NULL}
#define Y6_CONTROL_CONPONENT \
            (Control_Component_TypeDef){6, 0, NULL, NULL, NULL, NULL}
#define TRI_CONTROL_COMPONENT \
            (Control_Component_TypeDef){3, 1, NULL, NULL, NULL, NULL}
#define TDRONE_CONTROL_COMPONENT \
            (Control_Component_TypeDef){2, 3, NULL, NULL, NULL, NULL}

#define DSHOT_IDLE 100
#define DSHOT_MAX 1900

typedef enum
{
    Model_Quad = 0,
    Model_Hex,
    Model_Oct,
    Model_X8,
    Model_Y6,
    Model_Tri,
    Model_TDrone,
}Control_Model_List;

#pragma pack(1)
typedef enum
{
    ESC_Type_PWM = 0,
    ESC_Type_Dshot_150,
    ESC_Type_Dshot_300,
    ESC_Type_Dshot_600,
    ESC_Type_Dshot_1200,
    WSC_Type_OneShot,
}Control_ESC_Type_List;
#pragma pak()

typedef struct
{
    uint16_t min;
    uint16_t max;
}ComponentCTL_Range_TypeDef;

typedef struct
{
    uint8_t moto_num;
    uint8_t servo_num;

    Control_ESC_Type_List *moto_type;
    ComponentCTL_Range_TypeDef *moto_range;
    ComponentCTL_Range_TypeDef *servo_range;
    void *rotor_obj;
}Control_Component_TypeDef;

typedef struct
{
    Control_Model_List model;
    Control_Component_TypeDef drive_component;
}ControlParam_TypDef;

#endif
