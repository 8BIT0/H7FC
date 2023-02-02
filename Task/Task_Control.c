/*
*  coder : 8_B!T0
*  this file use for moto control
*/
#include "Task_Control.h"
#include "DataPipe.h"
#include "Dev_Dshot.h"
#include "mmu.h"

#define DEFAULT_CONTROL_MODLE Model_Quad
#define DEFAULT_CONTROL_ESC_TYPE ESC_Type_Dshot_600

static ControlParam_TypDef Control_Param; 

/*
 * For MATEK H743 Flight Controller Hardware
 * S1  PB0  TIM3  CH3
 * S2  PB1  TIM3  CH4
 * S3  PA0  TIM5  CH1
 * S4  PA1  TIM5  CH2
 * S5  PA2  TIM5  CH3
 * S6  PA3  TIM5  CH4
 * S7  PD12 TIM4  CH1
 * S8  PD13 TIM4  CH2
 * S9  PD14 TIM4  CH3
 * S10 PD15 TIM4  CH4 NO DMA
 * S11 PE5  TIM15 CH1
 * S12 PE6  TIM15 CH2
 */
bool TaskControl_Init(void)
{
    memset(&Control_Param, 0, sizeof(Control_Param));

    /* read in storage */
    Control_Param.model = DEFAULT_CONTROL_MODLE;

    switch(Control_Param.model)
    {
        case Model_Quad:
            Control_Param.drive_component = QUAD_CONTROL_COMPONENT;
            break;

        case Model_Hex:
            Control_Param.drive_component = HEX_CONTROL_COMPONENT;
            break;

        case Model_Oct:
            Control_Param.drive_component = OCT_CONTROL_COMPONENT;
            break;

        case Model_X8:
            Control_Param.drive_component = X8_CONTROL_COMPONENT;
            break;

        case Model_Y6:
            Control_Param.drive_component = Y6_CONTROL_CONPONENT;
            break;

        case Model_Tri:
            Control_Param.drive_component = TRI_CONTROL_COMPONENT;
            break;

        case Model_TDrone:
            Control_Param.drive_component = TDRONE_CONTROL_COMPONENT;
            break;

        default:
            return false;
    }

    return true;
}