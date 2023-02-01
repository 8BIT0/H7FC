#ifndef __TASK_CONTROL_H
#define __TASK_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define QUAD_CONTROL_COMPONENT {4, 0}
#define HEX_CONTROL_COMPONENT {6, 0}
#define OCT_CONTROL_COMPONENT {8, 0}
#define X8_CONTROL_COMPONENT {8, 0}
#define Y6_CONTROL_CONPONENT {6, 0}
#define TRI_CONTROL_COMPONENT {3, 1}
#define TDRONE_CONTROL_COMPONENT {2, 3}

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

typedef struct
{
    uint16_t min;
    uint16_t max;
}ComponentCTL_Range_TypeDef;

typedef struct
{
    uint8_t moto_num;
    uint8_t servo_num;

    ComponentCTL_Range_TypeDef *moto_range;
    ComponentCTL_Range_TypeDef *servo_range;
}Control_Component_TypeDef;

typedef struct
{
    Control_Model_List model;
    Control_Component_TypeDef drive_component;
}ControlParam_TypDef;

#endif
