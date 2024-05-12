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
 * 
 * For AT32 AIO Flight Controller Hardware
 * S1   PB0     TIM3    CH3 Channel_63
 * S2   PB1     TIM3    CH4 Channel_64
 * S3   PA3     TIM2    CH4 Channel_59
 * S4   PA2     TIM2    CH3 Channel_58
 * S5   PC8     TIM8    CH3 Channel_51
 * S6   PA8     TIM1    CH1 Channel_42
 * 
 */
#include "Srv_Actuator.h"
#include "Srv_OsCommon.h"
#include "datapipe.h"

const SrvActuator_PeriphSet_TypeDef SrvActuator_Periph_List[Actuator_PWM_SigSUM] = {
    SRVACTUATOR_PB0_SIG_1,
    SRVACTUATOR_PB1_SIG_2,
    SRVACTUATOR_PA0_SIG_3,
    SRVACTUATOR_PA1_SIG_4,
    SRVACTUATOR_PA2_SIG_5,
    SRVACTUATOR_PA3_SIG_6,

#if defined MATEKH743_V1_5
    SRVACTUATOR_PD12_SIG_7,
    SRVACTUATOR_PD13_SIG_8,
    SRVACTUATOR_PD14_SIG_9,
    SRVACTUATOR_PD15_SIG_10,
    SRVACTUATOR_PE5_SIG_11,
    SRVACTUATOR_PE6_SIG_12
#endif
};

const uint8_t default_sig_serial[Actuator_PWM_SigSUM] = {
    Actuator_PWM_Sig1,
    Actuator_PWM_Sig2,
    Actuator_PWM_Sig3,
    Actuator_PWM_Sig4,
    Actuator_PWM_Sig5,
    Actuator_PWM_Sig6,

#if defined MATEKH743_V1_5
    Actuator_PWM_Sig7,
    Actuator_PWM_Sig8,
    Actuator_PWM_Sig9,
    Actuator_PWM_Sig10,
    Actuator_PWM_Sig11,
    Actuator_PWM_Sig12,
#endif
};

/* internal variable */
SrvActuatorObj_TypeDef SrvActuator_Obj;
SrcActuatorCTL_Obj_TypeDef SrvActuator_ControlStream;

/* internal function */
static void SrcActuator_Get_ChannelRemap(SrvActuator_Setting_TypeDef cfg);
static bool SrvActuator_Config_MotoSpinDir(void);
static bool SrvActuator_QuadDrone_MotoMixControl(uint16_t *rc_ctl);

/* external function */
static bool SrvActuator_DeInit(void);
static bool SrvActuator_Init(SrvActuator_Setting_TypeDef cfg);
static bool SrvActuator_DeInit(void);
static void SrvActuator_MotoControl(uint16_t *p_val);
static bool SrvActuator_InvertMotoSpinDir(uint8_t component_index);
static bool SrvActuator_Lock(void);
static SrvActuator_ModelComponentNum_TypeDef SrvActuator_Get_NumData(void);
static SrvActuator_Model_List SrvActuator_GetModel(void);
static bool SrvActuator_Get_MotoControlRange(uint8_t moto_index, int16_t *min, int16_t *idle, int16_t *max);
static bool SrvActuator_Get_ServoControlRange(uint8_t servo_index, int16_t *min, int16_t *idle, int16_t *max);
static bool SrvActuator_Moto_DirectDrive(uint8_t index, uint16_t value);
static bool SrvActuator_Servo_DirectDrive(uint8_t index, uint16_t value);
static SrvActuator_Setting_TypeDef SrvActuator_Default_Setting(void);
 
/* external variable */
SrvActuator_TypeDef SrvActuator = {
    .init = SrvActuator_Init,
    .de_init = SrvActuator_DeInit,
    .lock = SrvActuator_Lock,
    .moto_control = SrvActuator_MotoControl,
    .invert_spin = SrvActuator_InvertMotoSpinDir,
    .get_cnt = SrvActuator_Get_NumData,
    .get_model = SrvActuator_GetModel,
    .get_moto_control_range = SrvActuator_Get_MotoControlRange,
    .get_servo_control_range = SrvActuator_Get_ServoControlRange,
    .moto_direct_drive = SrvActuator_Moto_DirectDrive,
    .servo_direct_drive = SrvActuator_Servo_DirectDrive,
    .default_param = SrvActuator_Default_Setting,
};

static bool SrvActuator_DeInit(void)
{
    SrvActuator_ModelComponentNum_TypeDef actuator_num;
    SrvActuator_PWMOutObj_TypeDef *PWM_List = NULL;
    static uint8_t m_i = 0;
    static uint8_t s_i = 0;

    memset(&actuator_num, 0, sizeof(SrvActuator_ModelComponentNum_TypeDef));

    if (SrvActuator_Obj.init)
    {
        actuator_num = SrvActuator_Obj.drive_module.num;
        PWM_List = SrvActuator_Obj.drive_module.obj_list;

        if (PWM_List)
        {
            /* deinit moto timer */
            for (; m_i < actuator_num.moto_cnt; m_i ++)
            {
                switch(PWM_List[m_i].drv_type)
                {
                    case DevDshot_150:
                    case DevDshot_300:
                    case DevDshot_600:
                        if (!DevDshot.de_init(PWM_List[m_i].drv_obj))
                            return false;
                        break;

                    default: break;
                }
            }

            /* deinit servo timer */
            /* still in developping */
            for (; s_i < actuator_num.servo_cnt; s_i ++)
            {

            }
        }
    }

    return true;
}

static bool SrvActuator_Init(SrvActuator_Setting_TypeDef cfg)
{
    memset(&SrvActuator_Obj, 0, sizeof(SrvActuator_Obj));
    memset(&SrvActuator_ControlStream, 0, sizeof(SrvActuator_ControlStream));

    switch (cfg.model)
    {
    case Model_Quad:
        SrvActuator_Obj.drive_module.num = QUAD_CONTROL_COMPONENT;
        break;
    
    case Model_Hex:
        SrvActuator_Obj.drive_module.num = HEX_CONTROL_COMPONENT;
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
#if defined MATEKH743_V1_5

    case Model_Oct:
        SrvActuator_Obj.drive_module.num = OCT_CONTROL_COMPONENT;
        break;

    case Model_X8:
        SrvActuator_Obj.drive_module.num = X8_CONTROL_COMPONENT;
        break;
#endif

    default:
        return false;
    }

    /* read in storage */
    /* current use default */
    SrvActuator_Obj.model = cfg.model;

    /* malloc dshot esc driver obj for using */
    SrvActuator_Obj.drive_module.obj_list = (SrvActuator_PWMOutObj_TypeDef *)SrvOsCommon.malloc(sizeof(SrvActuator_PWMOutObj_TypeDef) * SrvActuator_Obj.drive_module.num.total_cnt);

    if (SrvActuator_Obj.drive_module.obj_list == NULL)
    {
        SrvOsCommon.free(SrvActuator_Obj.drive_module.obj_list);
        return false;
    }

    if (SrvActuator_Obj.drive_module.num.moto_cnt)
    {
        /* default init */
        for (uint8_t i = 0; i < SrvActuator_Obj.drive_module.num.moto_cnt; i++)
        {
            switch (cfg.esc_type)
            {
            case Actuator_DevType_DShot150:
            case Actuator_DevType_DShot300:
            case Actuator_DevType_DShot600:
                SrvActuator_Obj.drive_module.obj_list[i].drv_type = cfg.esc_type;

                SrvActuator_Obj.drive_module.obj_list[i].ctl_val = DSHOT_LOCK_THROTTLE;
                SrvActuator_Obj.drive_module.obj_list[i].min_val = DSHOT_MIN_THROTTLE;
                SrvActuator_Obj.drive_module.obj_list[i].max_val = DSHOT_MAX_THROTTLE;
                SrvActuator_Obj.drive_module.obj_list[i].idle_val = DSHOT_IDLE_THROTTLE;
                SrvActuator_Obj.drive_module.obj_list[i].lock_val = DSHOT_LOCK_THROTTLE;

                SrvActuator_Obj.drive_module.obj_list[i].drv_obj = (DevDshotObj_TypeDef *)SrvOsCommon.malloc(sizeof(DevDshotObj_TypeDef));
                break;

            case Actuator_DevType_ServoPWM:
                break;

            default:
                SrvOsCommon.free(SrvActuator_Obj.drive_module.obj_list);
                return false;
            }

            if (SrvActuator_Obj.drive_module.obj_list[i].drv_obj == NULL)
            {
                for (uint8_t j = 0; j < i; j++)
                {
                    SrvOsCommon.free(SrvActuator_Obj.drive_module.obj_list[j].drv_obj);
                }

                SrvOsCommon.free(SrvActuator_Obj.drive_module.obj_list);
                return false;
            }
        }
    }

    /* create servo object */
    if (SrvActuator_Obj.drive_module.num.servo_cnt)
    {
    }

    /* check value remap relationship */
    /* we can read this info from storage module */
    SrcActuator_Get_ChannelRemap(cfg);
    // SrvActuator_Config_MotoSpinDir();

    SrvActuator_Lock();

    SrvActuator_Obj.init = true;
    return true;
}

static SrvActuator_Setting_TypeDef SrvActuator_Default_Setting(void)
{
    SrvActuator_Setting_TypeDef default_setting;
    memset(&default_setting, 0, sizeof(SrvActuator_Setting_TypeDef));

    default_setting.model = Model_Quad;
    default_setting.esc_type = DevDshot_300;

    default_setting.moto_num = 4;
    default_setting.servo_num = 0;

    memcpy(default_setting.pwm_ch_map, default_sig_serial, 4);

    return default_setting;
}

static void SrcActuator_Get_ChannelRemap(SrvActuator_Setting_TypeDef cfg)
{
    SrvActuator_PeriphSet_TypeDef *periph_ptr = NULL;

    /* get remap relationship */
    /* moto section */
    if (SrvActuator_Obj.drive_module.num.moto_cnt)
    {
        for (uint8_t i = 0; i < SrvActuator_Obj.drive_module.num.moto_cnt; i++)
        {
            SrvActuator_Obj.drive_module.obj_list[i].sig_id = cfg.pwm_ch_map[i];
            SrvActuator_Obj.drive_module.obj_list[i].periph_ptr = (SrvActuator_PeriphSet_TypeDef *)&SrvActuator_Periph_List[cfg.pwm_ch_map[i]];
            periph_ptr = SrvActuator_Obj.drive_module.obj_list[i].periph_ptr;

            DevDshot.init(SrvActuator_Obj.drive_module.obj_list[i].drv_obj,
                          periph_ptr->tim_base, periph_ptr->tim_channel, (void *)&(periph_ptr->pin),
                          periph_ptr->dma, periph_ptr->dma_channel);
        }
    }

    /* servo section */
    /* stilling in developping */
    if (SrvActuator_Obj.drive_module.num.servo_cnt)
    {
        for (uint8_t i = SrvActuator_Obj.drive_module.num.moto_cnt; i < SrvActuator_Obj.drive_module.num.total_cnt; i++)
        {
        }
    }
}

static bool SrvActuator_Lock(void)
{
    uint8_t i = 0;

    if (!SrvActuator_Obj.init)
        return false;

    for (i = 0; i < SrvActuator_Obj.drive_module.num.total_cnt; i++)
    {
        switch (SrvActuator_Obj.drive_module.obj_list[i].drv_type)
        {
        case Actuator_DevType_DShot150:
        case Actuator_DevType_DShot300:
        case Actuator_DevType_DShot600:
            DevDshot.control(SrvActuator_Obj.drive_module.obj_list[i].drv_obj, SrvActuator_Obj.drive_module.obj_list[i].lock_val);
            break;

        /* servo part still in developping */
        case Actuator_DevType_ServoPWM:
        default:
            return false;
        }
    }
    
    return true;
}

static void SrvActuator_MotoControl(uint16_t *p_val)
{
    if ((p_val == NULL) || !SrvActuator_Obj.init)
        return;

    switch (SrvActuator_Obj.model)
    {
    case Model_Quad:
        SrvActuator_QuadDrone_MotoMixControl(p_val);
        break;

    default:
        return;
    }
}

static void SrvActuator_ServoControl(uint8_t index, uint16_t val)
{
}

/* mast set spin direction when moto under halt statement */
static bool SrvActuator_SetMotoSpinDir(uint8_t component_index, SrvActuator_SpinDir_List dir)
{
    uint32_t dir_cmd = 0;

    if (!SrvActuator_Obj.init ||
        (component_index >= SrvActuator_Obj.drive_module.num.moto_cnt) ||
        (SrvActuator_Obj.drive_module.obj_list[component_index].ctl_val !=
         SrvActuator_Obj.drive_module.obj_list[component_index].lock_val) ||
        (dir == Actuator_SS_CW) ||
        (dir == Actuator_SS_ACW))
        return false;

    switch (SrvActuator_Obj.drive_module.obj_list[component_index].drv_type)
    {
    case Actuator_DevType_DShot150:
    case Actuator_DevType_DShot300:
    case Actuator_DevType_DShot600:
        if (dir == Actuator_MS_CW)
        {
            dir_cmd = DSHOT_CMD_SET_SPIN_CLOCKWISE;
        }
        else
            dir_cmd = DSHOT_CMD_SET_SPIN_ANTICLOCKWISE;

        DevDshot.command(SrvActuator_Obj.drive_module.obj_list[component_index].drv_obj, dir_cmd);
        SrvOsCommon.delay_ms(10);
        DevDshot.command(SrvActuator_Obj.drive_module.obj_list[component_index].drv_obj, DSHOT_CMD_SAVE_SETTING);
        break;

    case Actuator_DevType_ServoPWM:
        break;

    default:
        return false;
    }

    return true;
}

static bool SrvActuator_Config_MotoSpinDir(void)
{
    /* we should read moto spin direction info from storage module */
    /* if read failed use default setting down below */
    switch (SrvActuator_Obj.model)
    {
    case Model_Quad:
        for (uint8_t i = 0; i < SrvActuator_Obj.drive_module.num.moto_cnt; i++)
        {
            if ((i == 0) || (i == 3))
            {
                SrvActuator_Obj.drive_module.obj_list[i].spin_dir = Actuator_MS_ACW;
            }
            else if ((i == 1) || (i == 2))
            {
                SrvActuator_Obj.drive_module.obj_list[i].spin_dir = Actuator_MS_CW;
            }

            SrvActuator_SetMotoSpinDir(i, SrvActuator_Obj.drive_module.obj_list[i].spin_dir);
        }
        break;

    default:
        break;
    }

    return true;
}

static bool SrvActuator_InvertMotoSpinDir(uint8_t component_index)
{
    if (!SrvActuator_Obj.init ||
        (component_index >= SrvActuator_Obj.drive_module.num.moto_cnt))
        return false;

    switch (SrvActuator_Obj.drive_module.obj_list[component_index].spin_dir)
    {
    case Actuator_MS_CW:
        if (SrvActuator_SetMotoSpinDir(component_index, Actuator_MS_ACW))
        {
            SrvActuator_Obj.drive_module.obj_list[component_index].spin_dir = Actuator_MS_ACW;
            return true;
        }
        return false;

    case Actuator_MS_ACW:
        if (SrvActuator_SetMotoSpinDir(component_index, Actuator_MS_CW))
        {
            SrvActuator_Obj.drive_module.obj_list[component_index].spin_dir = Actuator_MS_CW;
            return true;
        }
        return false;

    default:
        return false;
    }
}

static SrvActuator_ModelComponentNum_TypeDef SrvActuator_Get_NumData(void)
{
    return SrvActuator_Obj.drive_module.num;
}

static SrvActuator_Model_List SrvActuator_GetModel(void)
{
    return SrvActuator_Obj.model;
}

/*
 * X axis -> Roll
 * Y axis -> Pitch
 * Z axis -> Yaw
 *
 * M1    M2
 *   \  /
 *    \/
 *    /\
 *   /  \
 * M3    M4
 *
 */
static bool SrvActuator_QuadDrone_MotoMixControl(uint16_t *pid_ctl)
{
    float throttle_base_percent = 0.0f;
    uint16_t ctl_val[4] = {0};

    if ((!SrvActuator_Obj.init) ||
        (pid_ctl == NULL))
        return false;

    /* limit throttle max output at 80% */
    if (pid_ctl[Actuator_Ctl_Throttle] >= SRV_ACTUATOR_MAX_THROTTLE_PERCENT)
        pid_ctl[Actuator_Ctl_Throttle] = SRV_ACTUATOR_MAX_THROTTLE_PERCENT;

    throttle_base_percent = pid_ctl[Actuator_Ctl_Throttle] / 100.0f;

    for (uint8_t i = 0; i < 4; i++)
    {
        SrvActuator_Obj.drive_module.obj_list[i].ctl_val = (SrvActuator_Obj.drive_module.obj_list[i].max_val -
                                                            SrvActuator_Obj.drive_module.obj_list[i].min_val) *
                                                            throttle_base_percent +
                                                            SrvActuator_Obj.drive_module.obj_list[i].min_val;
    }

    ctl_val[0] += pid_ctl[Actuator_Ctl_GyrX] - pid_ctl[Actuator_Ctl_GyrY] - pid_ctl[Actuator_Ctl_GyrZ];
    ctl_val[1] += pid_ctl[Actuator_Ctl_GyrX] + pid_ctl[Actuator_Ctl_GyrY] + pid_ctl[Actuator_Ctl_GyrZ];
    ctl_val[2] -= pid_ctl[Actuator_Ctl_GyrX] + pid_ctl[Actuator_Ctl_GyrY] - pid_ctl[Actuator_Ctl_GyrZ];
    ctl_val[3] -= pid_ctl[Actuator_Ctl_GyrX] - pid_ctl[Actuator_Ctl_GyrY] + pid_ctl[Actuator_Ctl_GyrZ];

    for (uint8_t i = 0; i < SrvActuator_Obj.drive_module.num.moto_cnt; i++)
    {
        if (!SrvActuator_Moto_DirectDrive(i, ctl_val[i]))
            return false;
    }

    return true;
}

static bool SrvActuator_Get_MotoControlRange(uint8_t moto_index, int16_t *min, int16_t *idle, int16_t *max)
{
    (*min) = 0;
    (*idle) = 0;
    (*max) = 0;
 
    if(SrvActuator_Obj.init && \
       SrvActuator_Obj.drive_module.num.moto_cnt && \
       min && max && idle && \
       moto_index < SrvActuator_Obj.drive_module.num.moto_cnt)
    {
        (*min) = SrvActuator_Obj.drive_module.obj_list[moto_index].min_val;
        (*max) = SrvActuator_Obj.drive_module.obj_list[moto_index].max_val;
        (*idle) = SrvActuator_Obj.drive_module.obj_list[moto_index].idle_val;

        return true;
    }

    return false;
} 

static bool SrvActuator_Get_ServoControlRange(uint8_t servo_index, int16_t *min, int16_t *idle, int16_t *max)
{
    (*min) = 0;
    (*idle) = 0;
    (*max) = 0;

    if(SrvActuator_Obj.init && \
       SrvActuator_Obj.drive_module.num.servo_cnt && \
       min && max && idle && \
       servo_index < SrvActuator_Obj.drive_module.num.servo_cnt)
    {
        /* still in developping */
        return true;
    }

    return false;
}

static bool SrvActuator_Moto_DirectDrive(uint8_t index, uint16_t value)
{
    if (index < SrvActuator_Obj.drive_module.num.moto_cnt)
    {
        if (value < SrvActuator_Obj.drive_module.obj_list[index].min_val)
        {
            SrvActuator_Obj.drive_module.obj_list[index].ctl_val = SrvActuator_Obj.drive_module.obj_list[index].min_val;
        }
        else if (value > SrvActuator_Obj.drive_module.obj_list[index].max_val)
        {
            SrvActuator_Obj.drive_module.obj_list[index].ctl_val = SrvActuator_Obj.drive_module.obj_list[index].max_val;
        }
        else
            SrvActuator_Obj.drive_module.obj_list[index].ctl_val = value;

        switch (SrvActuator_Obj.drive_module.obj_list[index].drv_type)
        {
            case  Actuator_DevType_DShot600:
            case  Actuator_DevType_DShot300:
            case  Actuator_DevType_DShot150:
                DevDshot.control(SrvActuator_Obj.drive_module.obj_list[index].drv_obj, SrvActuator_Obj.drive_module.obj_list[index].ctl_val);
                return true;

            default:
                break;
        }
    }

    return false;
}

static bool SrvActuator_Servo_DirectDrive(uint8_t index, uint16_t value)
{
    return false;
}

/****************************************************** ESC Weak Function Implimentation *******************************************************/
void *DShot_Malloc(uint32_t size)
{
    return SrvOsCommon.malloc(size);
}

void DShot_Free(void *ptr)
{
    SrvOsCommon.free(ptr);
}

bool DShot_Port_Init(void *obj, uint32_t prescaler, void *time_ins, uint32_t time_ch, void *pin, uint8_t dma, uint8_t stream)
{
    if (obj && time_ins && pin)
    {
#if defined AT32F435RGT7
        To_DShot_Obj(obj)->pwm_obj.dma_callback_obj = SrvOsCommon.malloc(sizeof(BspDMA_IrqCall_Obj_TypeDef));
#endif

        if (!BspTimer_PWM.init(&(To_DShot_Obj(obj)->pwm_obj), time_ins, time_ch, *(BspGPIO_Obj_TypeDef *)pin, dma, stream, (uint32_t)(To_DShot_Obj(obj)->ctl_buf), DSHOT_DMA_BUFFER_SIZE))
            return false;

        BspTimer_PWM.set_prescaler(&(To_DShot_Obj(obj))->pwm_obj, prescaler);
        BspTimer_PWM.set_autoreload(&(To_DShot_Obj(obj))->pwm_obj, (MOTOR_BITLENGTH - 1));

        BspTimer_PWM.start_pwm(&(To_DShot_Obj(obj))->pwm_obj);
        return true;
    }

    return false;
}

void DShot_Port_Trans(void *obj)
{
    if (obj)
        BspTimer_PWM.dma_trans(&(To_DShot_Obj(obj)->pwm_obj));
}

uint32_t DShot_Get_Timer_CLKFreq(void *obj)
{
    return BspTimer_PWM.get_clock_freq(&(To_DShot_Obj(obj)->pwm_obj));
}




