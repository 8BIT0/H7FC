/*
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
#include "Srv_Actuator.h"
#include "Dev_Dshot.h"
#include "datapipe.h"
#include "mmu.h"

SrvActuatorObj_TypeDef SrvActuator_Obj;
SrcActuatorCTL_Obj_TypeDef SrvActuator_ControlStream;

static bool SrvActuator_Init(SrvActuator_Model_List model, uint8_t esc_type)
{
    uint8_t i = 0;
    memset(&SrvActuator_Obj, 0, sizeof(SrvActuator_Obj));
    memset(&SrvActuator_ControlStream, 0, sizeof(SrvActuator_ControlStream));

    switch(model)
    {
        case Model_Quad:
            SrvActuator_Obj.drive_module.num = QUAD_CONTROL_COMPONENT;
            break;

        case Model_Hex:
            SrvActuator_Obj.drive_module.num = HEX_CONTROL_COMPONENT;
            break;

        case Model_Oct:
            SrvActuator_Obj.drive_module.num = OCT_CONTROL_COMPONENT;
            break;

        case Model_X8:
            SrvActuator_Obj.drive_module.num = X8_CONTROL_COMPONENT;
            break;

        case Model_Y6:
            SrvActuator_Obj.drive_module.num = Y6_CONTROL_CONPONENT;
            break;

        case Model_Tri:
            SrvActuator_Obj.drive_module.num = TRI_CONTROL_COMPONENT;
            break;

        case Model_TDrone:
            SrvActuator_Obj.drive_module.num = TDRONE_CONTROL_COMPONENT;
            break;

        default:
            return false;
    }

    /* read in storage */
    /* current use default */
    SrvActuator_Obj.model = model;

    if(SrvActuator_Obj.drive_module.num.moto_cnt)
    {
        /* malloc dshot esc driver obj for using */
        SrvActuator_Obj.drive_module.obj_list = (SrvActuator_PWMOutObj_TypeDef *)MMU_Malloc(sizeof(SrvActuator_PWMOutObj_TypeDef) * SrvActuator_Obj.drive_module.num.moto_cnt);

        if(SrvActuator_Obj.drive_module.obj_list)
        {
            for(uint8_t i = 0; i < SrvActuator_Obj.drive_module.num.moto_cnt; i++)
            {
                switch(esc_type)
                {
                    case DevDshot_150:
                    case DevDshot_300:
                    case DevDshot_600:
                        SrvActuator_Obj.drive_module.obj_list[i].drv_type = esc_type;
                        break;

                    default:
                        MMU_Free(SrvActuator_Obj.drive_module.obj_list);
                        return false;
                }

                SrvActuator_Obj.drive_module.obj_list[i].drv_obj = (DevDshotObj_TypeDef *)MMU_Malloc(sizeof(DevDshotObj_TypeDef));

                if(SrvActuator_Obj.drive_module.obj_list[i].drv_obj)
                {

                }
                else
                {
                    for(uint8_t j = 0; j < i; j++)
                    {
                        MMU_Free(SrvActuator_Obj.drive_module.obj_list[j].drv_obj);
                    }
                                
                    MMU_Free(SrvActuator_Obj.drive_module.obj_list);
                    return false;
                }
            }
        }
        else
        {
            MMU_Free(SrvActuator_Obj.drive_module.obj_list);
            return false;
        }

        // if( (SrvActuator_Obj.drive_component.moto_type == ESC_Type_Dshot_150) ||
        //     (SrvActuator_Obj.drive_component.moto_type == ESC_Type_Dshot_300) ||
        //     (SrvActuator_Obj.drive_component.moto_type == ESC_Type_Dshot_600) ||
        //     (SrvActuator_Obj.drive_component.moto_type == ESC_Type_Dshot_1200))
        // {
        //     SrvActuator_ControlStream.moto_num = SrvActuator_Obj.drive_component.moto_num;
        //     SrvActuator_ControlStream.moto_val = (uint16_t *)MMU_Malloc(SrvActuator_ControlStream.moto_num * sizeof(uint16_t));
        //     if(SrvActuator_ControlStream.moto_val == NULL)
        //     {
        //         MMU_Free(SrvActuator_Obj.drive_component.moto_obj);
        //         MMU_Free(SrvActuator_ControlStream.moto_val);
        //         return false;
        //     }

        //     for(i = 0; i < SrvActuator_Obj.drive_component.moto_num; i++)
        //     {
        //         /* set moto range */
        //         SrvActuator_Obj.drive_component.moto_range[i].min = DSHOT_LOCK_THROTTLE;
        //         SrvActuator_Obj.drive_component.moto_range[i].idle = DSHOT_IDLE_THROTTLE;
        //         SrvActuator_Obj.drive_component.moto_range[i].max = DSHOT_MAX_THROTTLE;
            
        //         /* init dshot device object */
        //         DevDshot.init(&SrvActuator_Obj.drive_component.moto_obj[i], TIM3, TIM_CHANNEL_3, );

        //         /* lock dshot device output */
        //         SrvActuator_ControlStream.moto_val[i] = SrvActuator_Obj.drive_component.moto_range[i].min;
        //     }
        // }

        // /* create servo object */
        // if(SrvActuator_Obj.drive_component.servo_num)
        // {
        //     SrvActuator_ControlStream.servo_num = SrvActuator_Obj.drive_component.servo_num;
        //     SrvActuator_ControlStream.servo_val = (uint16_t *)MMU_Malloc(SrvActuator_ControlStream.servo_num * sizeof(uint16_t));
        //     if(SrvActuator_ControlStream.servo_val == NULL)
        //     {
        //         if(SrvActuator_Obj.drive_component.moto_num)
        //         {
        //             MMU_Free(SrvActuator_Obj.drive_component.moto_obj);
        //             MMU_Free(SrvActuator_ControlStream.moto_val);
        //         }

        //         MMU_Free(SrvActuator_ControlStream.servo_val);
        //         return false;
        //     }

        //     if(SrvActuator_Obj.drive_component.moto_range == NULL)
        //     {
        //         if(SrvActuator_Obj.drive_component.moto_num)
        //         {
        //             MMU_Free(SrvActuator_Obj.drive_component.moto_obj);
        //             MMU_Free(SrvActuator_ControlStream.moto_val);
        //         }

        //         MMU_Free(SrvActuator_ControlStream.servo_val);
        //         MMU_Free(SrvActuator_Obj.drive_component.moto_range);
        //         return false;
        //     }
        // }
    }

    return true;
}



