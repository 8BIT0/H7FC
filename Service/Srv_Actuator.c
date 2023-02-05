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

const SrvActuator_PeriphSet_TypeDef SrvActuator_Periph_List[Actuator_PWM_SigSUM] = {
    SRVACTUATOR_PB0_SIG_1,
    SRVACTUATOR_PB1_SIG_2,
    SRVACTUATOR_PA0_SIG_3,
    SRVACTUATOR_PA1_SIG_4,
    SRVACTUATOR_PA2_SIG_5,
    SRVACTUATOR_PA3_SIG_6,
    SRVACTUATOR_PD12_SIG_7,
    SRVACTUATOR_PD13_SIG_8,
    SRVACTUATOR_PD14_SIG_9,
    SRVACTUATOR_PD15_SIG_10,
    SRVACTUATOR_PE5_SIG_11,
    SRVACTUATOR_PE6_SIG_12
};

const uint8_t default_sig_serial[Actuator_PWM_SigSUM] = {
    Actuator_PWM_Sig1,
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
};

/* internal variable */
SrvActuatorObj_TypeDef SrvActuator_Obj;
SrcActuatorCTL_Obj_TypeDef SrvActuator_ControlStream;

/* internal function */
static bool SrcActuator_Get_ChannelRemap(void);

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

    /* malloc dshot esc driver obj for using */
    SrvActuator_Obj.drive_module.obj_list = (SrvActuator_PWMOutObj_TypeDef *)MMU_Malloc(sizeof(SrvActuator_PWMOutObj_TypeDef) * SrvActuator_Obj.drive_module.num.total_cnt);

    if(SrvActuator_Obj.drive_module.obj_list == NULL)
    {
        MMU_Free(SrvActuator_Obj.drive_module.obj_list);
        return false;
    }

    if(SrvActuator_Obj.drive_module.num.moto_cnt)
    {
        /* default init */
        for(uint8_t i = 0; i < SrvActuator_Obj.drive_module.num.moto_cnt; i++)
        {
            switch(esc_type)
            {
                case DevDshot_150:
                case DevDshot_300:
                case DevDshot_600:
                    SrvActuator_Obj.drive_module.obj_list[i].drv_type = esc_type;
                    SrvActuator_Obj.drive_module.obj_list[i].tag = SrvActuator_Tag_Moto;

                    SrvActuator_Obj.drive_module.obj_list[i].ctl_val = DSHOT_LOCK_THROTTLE;
                    SrvActuator_Obj.drive_module.obj_list[i].min_val = DSHOT_MIN_THROTTLE;
                    SrvActuator_Obj.drive_module.obj_list[i].max_val = DSHOT_MAX_THROTTLE;
                    SrvActuator_Obj.drive_module.obj_list[i].idle_val = DSHOT_IDLE_THROTTLE;
                    SrvActuator_Obj.drive_module.obj_list[i].lock_val = DSHOT_LOCK_THROTTLE;
                    break;

                default:
                    MMU_Free(SrvActuator_Obj.drive_module.obj_list);
                    return false;
            }

            SrvActuator_Obj.drive_module.obj_list[i].drv_obj = (DevDshotObj_TypeDef *)MMU_Malloc(sizeof(DevDshotObj_TypeDef));

            if(SrvActuator_Obj.drive_module.obj_list[i].drv_obj == NULL)
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

    /* create servo object */
    if(SrvActuator_Obj.drive_module.num.servo_cnt)
    {
    }

    /* check value remap relationship */
    /* we can read this info from storage module */
    SrcActuator_Get_ChannelRemap();

    return true;
}

static bool SrcActuator_Get_ChannelRemap(void)
{
    uint8_t storage_serial[SrvActuator_Obj.drive_module.num.moto_cnt + SrvActuator_Obj.drive_module.num.servo_cnt];
    SrvActuator_PeriphSet_TypeDef *periph_ptr = NULL;

    /* get remap relationship */
    memcpy(storage_serial, default_sig_serial, sizeof(storage_serial));

    /* moto section */
    if(SrvActuator_Obj.drive_module.num.moto_cnt)
    {
        for(uint8_t i = 0; i < SrvActuator_Obj.drive_module.num.moto_cnt; i++)
        {
            SrvActuator_Obj.drive_module.obj_list[i].sig_id = storage_serial[storage_serial[i]];
            SrvActuator_Obj.drive_module.obj_list[i].periph_ptr = &SrvActuator_Periph_List[storage_serial[i]];
            periph_ptr = SrvActuator_Obj.drive_module.obj_list[i].periph_ptr;

            DevDshot.init(SrvActuator_Obj.drive_module.obj_list[i].drv_obj, 
                          periph_ptr->tim_base, periph_ptr->tim_channel, periph_ptr->pin
                          , periph_ptr->dma, periph_ptr->dma_channel);
        }
    }

    /* servo section */
    if(SrvActuator_Obj.drive_module.num.servo_cnt)
    {
        for(uint8_t i = SrvActuator_Obj.drive_module.num.moto_cnt; i < SrvActuator_Obj.drive_module.num.total_cnt; i++)
        {
        }
    }
}

static void SrvActuator_Control(uint16_t *p_val, uint8_t len)
{
    if((p_val == NULL) || (len != SrvActuator_Obj.drive_module.num.total_cnt))
        return;

    switch(SrvActuator_Obj.model)
    {
        case Model_Quad:
            break;

        case Model_Hex:
            break;

        case Model_Oct:
            break;

        case Model_X8:
            break;

        case Model_Y6:
            break;

        case Model_Tri:
            break;

        case Model_TDrone:
            break;

        default:
            return;
    }
}




