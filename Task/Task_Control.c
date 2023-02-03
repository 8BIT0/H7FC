/*
 *  coder : 8_B!T0
 *  this file use for moto control
 *
 * For MATEK H743 Flight Controller Hardware
 * S1   PB0     TIM3    CH3 DMA1 Stream0
 * S2   PB1     TIM3    CH4 DMA1 Stream1
 * S3   PA0     TIM5    CH1 DMA1 Stream2
 * S4   PA1     TIM5    CH2 DMA1 Stream3
 * S5   PA2     TIM5    CH3
 * S6   PA3     TIM5    CH4
 * S7   PD12    TIM4    CH1
 * S8   PD13    TIM4    CH2
 * S9   PD14    TIM4    CH3
 * S10  PD15    TIM4    CH4 NO DMA
 * S11  PE5     TIM15   CH1
 * S12  PE6     TIM15   CH2
 */
#include "Task_Control.h"
#include "DataPipe.h"
#include "Dev_Dshot.h"
#include "mmu.h"

#define DEFAULT_CONTROL_MODLE Model_Quad
#define DEFAULT_CONTROL_ESC_TYPE ESC_Type_Dshot_600

static ControlParam_TypDef Control_Param; 
static Control_Stream_TypeDef Control_Stream;

bool TaskControl_Init(void)
{
    uint8_t i = 0;
    memset(&Control_Param, 0, sizeof(Control_Param));

    /* read in storage */
    /* current use default */
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

    if(Control_Param.drive_component.moto_num)
    {
        Control_Param.drive_component.moto_type = DEFAULT_CONTROL_ESC_TYPE;
        Control_Param.drive_component.moto_range = (ComponentCTL_Range_TypeDef *)MMU_Malloc(sizeof(ComponentCTL_Range_TypeDef) * Control_Param.drive_component.moto_num);

        if( (Control_Param.drive_component.moto_type == ESC_Type_Dshot_150) ||
            (Control_Param.drive_component.moto_type == ESC_Type_Dshot_300) ||
            (Control_Param.drive_component.moto_type == ESC_Type_Dshot_600) ||
            (Control_Param.drive_component.moto_type == ESC_Type_Dshot_1200))
        {
            Control_Param.drive_component.moto_obj = (DevDshotObj_TypeDef *)MMU_Malloc(sizeof(DevDshotObj_TypeDef) * Control_Param.drive_component.moto_num);

            if(Control_Param.drive_component.moto_obj == NULL)
            {
                MMU_Free(Control_Param.drive_component.moto_obj);
                return false;
            }
                
            Control_Stream.moto_num = Control_Param.drive_component.moto_num;
            Control_Stream.moto_val = (uint16_t *)MMU_Malloc(Control_Stream.moto_num * sizeof(uint16_t));
            if(Control_Stream.moto_val == NULL)
            {
                MMU_Free(Control_Param.drive_component.moto_obj);
                MMU_Free(Control_Stream.moto_val);
                return false;
            }

            for(i = 0; i < Control_Param.drive_component.moto_num; i++)
            {
                /* set moto range */
                Control_Param.drive_component.moto_range[i]->min = DSHOT_LOCK;
                Control_Param.drive_component.moto_range[i]->idle = DSHOT_IDLE;
                Control_Param.drive_component.moto_range[i]->max = DSHOT_MAX;
            
                /* init dshot device object */
                DevDshot.init(&Control_Param.drive_component.moto_obj[i], TIM3, TIM_CHANNEL_3, );

                /* lock dshot device output */
                Control_Stream.moto_val[i] = Control_Param.drive_component.moto_range[i]->min;
            }
        }

        /* create servo object */
        if(Control_Param.drive_component.servo_num)
        {
            Control_Stream.servo_num = Control_Param.drive_component.servo_num;
            Control_Stream.servo_val = (uint16_t *)MMU_Malloc(Control_Stream.servo_num * sizeof(uint16_t));
            if(Control_Stream.servo_val == NULL)
            {
                if(Control_Param.drive_component.moto_num)
                {
                    MMU_Free(Control_Param.drive_component.moto_obj);
                    MMU_Free(Control_Stream.moto_val);
                }

                MMU_Free(Control_Stream.servo_val);
                return false;
            }

            if(Control_Param.drive_component.moto_range == NULL)
            {
                if(Control_Param.drive_component.moto_num)
                {
                    MMU_Free(Control_Param.drive_component.moto_obj);
                    MMU_Free(Control_Stream.moto_val);
                }

                MMU_Free(Control_Stream.servo_val);
                MMU_Free(Control_Param.drive_component.moto_range);
                return false;
            }
        }
    }

    return true;
}
