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
#include "../System/DataPipe/DataPipe.h"

#define Actuator_Malloc(size) SrvOsCommon.malloc(size)
#define Actuator_Free(ptr) SrvOsCommon.free(ptr)

const SrvActuator_PeriphSet_TypeDef SrvActuator_Periph_List[Actuator_PWM_SigSUM] = {
    SRVACTUATOR_SIG_1,
    SRVACTUATOR_SIG_2,
    SRVACTUATOR_SIG_3,
    SRVACTUATOR_SIG_4,

#if defined BATEAT32F435_AIO 
    SRVACTUATOR_SIG_5,
    SRVACTUATOR_SIG_6,
#endif

#if defined MATEKH743_V1_5
    SRVACTUATOR_SIG_7,
    SRVACTUATOR_SIG_8,
    SRVACTUATOR_SIG_9,
    SRVACTUATOR_SIG_10,
    SRVACTUATOR_SIG_11,
    SRVACTUATOR_SIG_12
#endif
};

const uint8_t default_sig_serial[Actuator_PWM_SigSUM] = {
    Actuator_PWM_Sig1,
    Actuator_PWM_Sig2,
    Actuator_PWM_Sig3,
    Actuator_PWM_Sig4,

#if defined BATEAT32F435_AIO 
    Actuator_PWM_Sig5,
    Actuator_PWM_Sig6,
#endif

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
DataPipe_CreateDataObj(SrvActuatorPipeData_TypeDef, Actuator);
SrvActuatorObj_TypeDef SrvActuator_Obj;
SrcActuatorCTL_Obj_TypeDef SrvActuator_ControlStream;
osSemaphoreId SrvActuator_Sem = NULL;

/* internal function */
static void SrcActuator_Get_ChannelRemap(SrvActuator_Setting_TypeDef cfg);
static bool SrvActuator_QuadDrone_MotoMixControl(int16_t *rc_ctl);
static void SrvActuator_PipeData(void);

/* external function */
static bool SrvActuator_DeInit(void);
static bool SrvActuator_Init(SrvActuator_Setting_TypeDef cfg);
static void SrvActuator_MotoControl(int16_t *p_val);
static bool SrvActuator_TmpReversedMotoSpinDir(uint8_t component_index);
static bool SrvActuator_Lock(void);
static SrvActuator_ModelComponentNum_TypeDef SrvActuator_Get_NumData(void);
static SrvActuator_Model_List SrvActuator_GetModel(void);
static bool SrvActuator_Get_MotoControlRange(uint8_t moto_index, int16_t *min, int16_t *idle, int16_t *max);
static bool SrvActuator_Get_ServoControlRange(uint8_t servo_index, int16_t *min, int16_t *idle, int16_t *max);
static bool SrvActuator_Moto_DirectDrive(uint8_t index, uint16_t value);
static bool SrvActuator_Servo_DirectDrive(uint8_t index, uint16_t value);
static SrvActuator_Setting_TypeDef SrvActuator_Default_Setting(void);
static bool SrvActuator_SetSpin_Dir(uint8_t component_index, uint8_t dir);
static void SrvActuator_Saving(uint8_t component_index);

/* external variable */
SrvActuator_TypeDef SrvActuator = {
    .init = SrvActuator_Init,
    .de_init = SrvActuator_DeInit,
    .lock = SrvActuator_Lock,
    .moto_control = SrvActuator_MotoControl,
    .tmp_reverse_spin = SrvActuator_TmpReversedMotoSpinDir,
    .set_spin_dir = SrvActuator_SetSpin_Dir,
    .get_cnt = SrvActuator_Get_NumData,
    .get_model = SrvActuator_GetModel,
    .get_moto_control_range = SrvActuator_Get_MotoControlRange,
    .get_servo_control_range = SrvActuator_Get_ServoControlRange,
    .moto_direct_drive = SrvActuator_Moto_DirectDrive,
    .servo_direct_drive = SrvActuator_Servo_DirectDrive,
    .default_param = SrvActuator_Default_Setting,
    .save = SrvActuator_Saving,
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

    case Model_Tri:
        SrvActuator_Obj.drive_module.num = TRI_CONTROL_COMPONENT;
        break;
        
    case Model_TDrone:
        SrvActuator_Obj.drive_module.num = TDRONE_CONTROL_COMPONENT;
        break;
        
#if defined BATEAT32F435_AIO
    case Model_Hex:
        SrvActuator_Obj.drive_module.num = HEX_CONTROL_COMPONENT;
        break;

    case Model_Y6:
        SrvActuator_Obj.drive_module.num = Y6_CONTROL_CONPONENT;
        break;
#endif
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

    SrvActuator_Obj.drive_module.num.moto_cnt = cfg.moto_num;
    SrvActuator_Obj.drive_module.num.servo_cnt = cfg.servo_num;
    SrvActuator_Obj.drive_module.num.total_cnt = cfg.moto_num + cfg.servo_num;

    /* malloc dshot esc driver obj for using */
    SrvActuator_Obj.drive_module.obj_list = (SrvActuator_PWMOutObj_TypeDef *)Actuator_Malloc(sizeof(SrvActuator_PWMOutObj_TypeDef) * SrvActuator_Obj.drive_module.num.total_cnt);

    if (SrvActuator_Obj.drive_module.obj_list == NULL)
    {
        Actuator_Free(SrvActuator_Obj.drive_module.obj_list);
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

                SrvActuator_Obj.drive_module.obj_list[i].drv_obj = (DevDshotObj_TypeDef *)Actuator_Malloc(sizeof(DevDshotObj_TypeDef));
                break;

            case Actuator_DevType_Brush:
                SrvActuator_Obj.drive_module.obj_list[i].drv_type = cfg.esc_type;
                break;

            default:
                Actuator_Free(SrvActuator_Obj.drive_module.obj_list);
                return false;
            }

            if (SrvActuator_Obj.drive_module.obj_list[i].drv_obj == NULL)
            {
                for (uint8_t j = 0; j < i; j++)
                {
                    Actuator_Free(SrvActuator_Obj.drive_module.obj_list[j].drv_obj);
                }

                Actuator_Free(SrvActuator_Obj.drive_module.obj_list);
                return false;
            }
        }
    }

    /* create servo object */
    if (SrvActuator_Obj.drive_module.num.servo_cnt)
    {
        /* reserved */
    }

    /* data pipe init */
    Actuator_Smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Actuator);
    Actuator_Smp_DataPipe.data_size = DataPipe_DataSize(Actuator);
    DataPipe_Enable(&Actuator_Smp_DataPipe);

    /* check value remap relationship */
    /* we can read this info from storage module */
    SrcActuator_Get_ChannelRemap(cfg);
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

    memcpy(default_setting.moto_map, default_sig_serial, 4);

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
            SrvActuator_Obj.drive_module.obj_list[i].sig_id = cfg.moto_map[i];
            SrvActuator_Obj.drive_module.obj_list[i].periph_ptr = (SrvActuator_PeriphSet_TypeDef *)&SrvActuator_Periph_List[cfg.moto_map[i]];
            periph_ptr = SrvActuator_Obj.drive_module.obj_list[i].periph_ptr;

            DevDshot.init(SrvActuator_Obj.drive_module.obj_list[i].drv_obj,
                          periph_ptr->tim_base, periph_ptr->tim_channel, (void *)&(periph_ptr->pin),
                          periph_ptr->dma, periph_ptr->dma_channel);
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
                SrvActuator_Obj.drive_module.obj_list[i].ctl_val = SrvActuator_Obj.drive_module.obj_list[i].lock_val;
                break;

            /* still in developping */
            case Actuator_DevType_Brush:
                break;
                
            default: return false;
        }
    }

    SrvActuator_PipeData();
    return true;
}

static void SrvActuator_PipeData(void)
{
    uint8_t m = 0;

    /* pipe actuator control data to data hub */
    DataPipe_DataObj(Actuator).time_stamp = SrvOsCommon.get_os_ms();
    DataPipe_DataObj(Actuator).moto_cnt = SrvActuator_Obj.drive_module.num.moto_cnt;
    DataPipe_DataObj(Actuator).servo_cnt = SrvActuator_Obj.drive_module.num.servo_cnt;
    /* reserved */
    memset(DataPipe_DataObj(Actuator).servo, 0, sizeof(DataPipe_DataObj(Actuator).servo));
    m = DataPipe_DataObj(Actuator).moto_cnt;
    if (DataPipe_DataObj(Actuator).moto_cnt > (sizeof(DataPipe_DataObj(Actuator).moto) / sizeof(DataPipe_DataObj(Actuator).moto[0])))
        m = sizeof(DataPipe_DataObj(Actuator).moto) / sizeof(DataPipe_DataObj(Actuator).moto[0]);

    for (uint8_t i = 0; i < m; i ++)
        DataPipe_DataObj(Actuator).moto[i] = SrvActuator_Obj.drive_module.obj_list[i].ctl_val;

    DataPipe_SendTo(&Actuator_Smp_DataPipe, &Actuator_hub_DataPipe);
}

static void SrvActuator_MotoControl(int16_t *p_val)
{
    uint8_t i = 0;
    uint8_t offset;

    if ((p_val == NULL) || !SrvActuator_Obj.init)
        return;

    switch (SrvActuator_Obj.model)
    {
        case Model_Quad:
            SrvActuator_QuadDrone_MotoMixControl(p_val);
            break;

        default:
            for (i = 0; i < SrvActuator_Obj.drive_module.num.moto_cnt; i ++)
                SrvActuator_Obj.drive_module.obj_list[i].ctl_val = SrvActuator_Obj.drive_module.obj_list[i].lock_val;
            
            offset = SrvActuator_Obj.drive_module.num.moto_cnt;
            for (i = 0; i < SrvActuator_Obj.drive_module.num.servo_cnt; i ++)
                SrvActuator_Obj.drive_module.obj_list[i + offset].ctl_val = 0;
            break;
    }

    SrvActuator_PipeData();
}

static void SrvActuator_SendCommand(DevDshotObj_TypeDef *p_moto, uint8_t cmd)
{
    if (p_moto == NULL)
        return;

    SrvOsCommon.delay_ms(9);
    for (uint8_t i = 0; i < 10; i++)
    {
        SrvOsCommon.delay_ms(1);
        DevDshot.command(p_moto, cmd);
    }
    SrvOsCommon.delay_ms(1);
}

static bool SrvActuator_TmpReversedMotoSpinDir(uint8_t component_index)
{
    if (!SrvActuator_Obj.init || \
        (component_index >= SrvActuator_Obj.drive_module.num.moto_cnt))
        return false;

    if ((SrvActuator_Obj.drive_module.obj_list[component_index].drv_type == Actuator_DevType_DShot150) || \
        (SrvActuator_Obj.drive_module.obj_list[component_index].drv_type == Actuator_DevType_DShot300) || \
        (SrvActuator_Obj.drive_module.obj_list[component_index].drv_type == Actuator_DevType_DShot600))
    {
        SrvActuator_SendCommand(SrvActuator_Obj.drive_module.obj_list[component_index].drv_obj, DevDshot_SpinDir_Reversed);
        return true;
    }

    return false;
}

static void SrvActuator_Saving(uint8_t component_index)
{
    if ((SrvActuator_Obj.drive_module.obj_list[component_index].drv_type != Actuator_DevType_DShot150) && \
        (SrvActuator_Obj.drive_module.obj_list[component_index].drv_type != Actuator_DevType_DShot300) && \
        (SrvActuator_Obj.drive_module.obj_list[component_index].drv_type != Actuator_DevType_DShot600))
        return;

    SrvActuator_SendCommand(To_DShot_Obj(SrvActuator_Obj.drive_module.obj_list[component_index].drv_obj), DevDshot_Save_Setting);
    SrvOsCommon.delay_ms(50);
}

static bool SrvActuator_SetSpin_Dir(uint8_t component_index, uint8_t dir)
{
    uint8_t cmd = 0;

    if (!SrvActuator_Obj.init || \
        (component_index >= SrvActuator_Obj.drive_module.num.moto_cnt))
        return false;

    if (((SrvActuator_Obj.drive_module.obj_list[component_index].drv_type == Actuator_DevType_DShot150) || \
         (SrvActuator_Obj.drive_module.obj_list[component_index].drv_type == Actuator_DevType_DShot300) || \
         (SrvActuator_Obj.drive_module.obj_list[component_index].drv_type == Actuator_DevType_DShot600)) && \
        (dir <= (uint8_t)DevDshot_SpinDir_2))
    {
        cmd = DevDshot_RotateDir_ClockWise;
        if (dir == DevDshot_SpinDir_2)
            cmd = DevDshot_RotateDir_AntiClockWise;

        SrvActuator_SendCommand(To_DShot_Obj(SrvActuator_Obj.drive_module.obj_list[component_index].drv_obj), cmd);
        return true;
    }

    return false;
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
static bool SrvActuator_QuadDrone_MotoMixControl(int16_t *pid_ctl)
{
    float throttle_base_percent = 0.0f;

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
                                                            SrvActuator_Obj.drive_module.obj_list[i].idle_val) *
                                                            throttle_base_percent +
                                                            SrvActuator_Obj.drive_module.obj_list[i].idle_val;
    }

    SrvActuator_Obj.drive_module.obj_list[0].ctl_val += pid_ctl[Actuator_Ctl_GyrX] - pid_ctl[Actuator_Ctl_GyrY] - pid_ctl[Actuator_Ctl_GyrZ];
    SrvActuator_Obj.drive_module.obj_list[1].ctl_val += pid_ctl[Actuator_Ctl_GyrX] + pid_ctl[Actuator_Ctl_GyrY] + pid_ctl[Actuator_Ctl_GyrZ];
    SrvActuator_Obj.drive_module.obj_list[2].ctl_val -= pid_ctl[Actuator_Ctl_GyrX] + pid_ctl[Actuator_Ctl_GyrY] - pid_ctl[Actuator_Ctl_GyrZ];
    SrvActuator_Obj.drive_module.obj_list[3].ctl_val -= pid_ctl[Actuator_Ctl_GyrX] - pid_ctl[Actuator_Ctl_GyrY] + pid_ctl[Actuator_Ctl_GyrZ];

    for (uint8_t i = 0; i < SrvActuator_Obj.drive_module.num.moto_cnt; i++)
        SrvActuator_Moto_DirectDrive(i, SrvActuator_Obj.drive_module.obj_list[i].ctl_val);

    return true;
}

static bool SrvActuator_Get_MotoControlRange(uint8_t moto_index, int16_t *min, int16_t *idle, int16_t *max)
{ 
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
    if(SrvActuator_Obj.init && \
       SrvActuator_Obj.drive_module.num.servo_cnt && \
       min && max && idle && \
       servo_index < SrvActuator_Obj.drive_module.num.servo_cnt)
    {
        (*min) = 0;
        (*idle) = 0;
        (*max) = 0;
        /* still in developping */
        return true;
    }

    return false;
}

static bool SrvActuator_Moto_DirectDrive(uint8_t index, uint16_t value)
{
    if (index < SrvActuator_Obj.drive_module.num.moto_cnt)
    {
        SrvActuator_Obj.drive_module.obj_list[index].ctl_val = value;
        if (value < SrvActuator_Obj.drive_module.obj_list[index].min_val)
        {
            SrvActuator_Obj.drive_module.obj_list[index].ctl_val = SrvActuator_Obj.drive_module.obj_list[index].min_val;
        }
        else if (value > SrvActuator_Obj.drive_module.obj_list[index].max_val)
        {
            SrvActuator_Obj.drive_module.obj_list[index].ctl_val = SrvActuator_Obj.drive_module.obj_list[index].max_val;
        }

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
    return Actuator_Malloc(size);
}

void DShot_Free(void *ptr)
{
    Actuator_Free(ptr);
}

bool DShot_Port_DeInit(void *obj)
{
    if (obj && BspTimer_PWM.de_init(To_TimerPWMObj_Ptr(To_DShot_Obj(obj)->p_timr_obj)))
        return true;

    return false;
}

void DShot_Port_Trans(void *obj)
{
    if (obj && SrvActuator_Sem)
    {
        BspTimer_PWM.dma_trans(To_TimerPWMObj_Ptr(To_DShot_Obj(obj)->p_timr_obj));
        osSemaphoreWait(SrvActuator_Sem, 1);
    }
}

uint32_t DShot_Get_Timer_CLKFreq(void *obj)
{
    return BspTimer_PWM.get_clock_freq(To_TimerPWMObj_Ptr(To_DShot_Obj(obj)->p_timr_obj));
}

void DShot_Tran_Finish(void)
{
    if (SrvActuator_Sem)
        osSemaphoreRelease(SrvActuator_Sem);
}

bool DShot_Port_Init(void *obj, uint32_t prescaler, void *time_ins, uint32_t time_ch, void *pin, uint8_t dma, uint8_t stream)
{
    if (obj && time_ins && pin)
    {
        if (SrvActuator_Sem == NULL)
        {
            osSemaphoreDef(DShot_Sem);
            SrvActuator_Sem = osSemaphoreCreate(osSemaphore(DShot_Sem), 1);
        }

        /* malloc timer dma pwm object */
        To_DShot_Obj(obj)->p_timr_obj = Actuator_Malloc(sizeof(BspTimerPWMObj_TypeDef));
        if (To_DShot_Obj(obj)->p_timr_obj == NULL)
        {
            Actuator_Free(To_DShot_Obj(obj)->p_timr_obj);
            return false;
        }

#if defined STM32H743xx
        To_TimerPWMObj_Ptr(To_DShot_Obj(obj)->p_timr_obj)->tim_hdl = Actuator_Malloc(sizeof(TIM_HandleType_Size));
        if (To_TimerPWMObj_Ptr(To_DShot_Obj(obj)->p_timr_obj)->tim_hdl == NULL)
        {
            Actuator_Free(To_TimerPWMObj_Ptr(To_DShot_Obj(obj)->p_timr_obj)->tim_hdl);
            Actuator_Free(To_DShot_Obj(obj)->p_timr_obj);
            return false;
        }

        To_TimerPWMObj_Ptr(To_DShot_Obj(obj)->p_timr_obj)->dma_hdl = Actuator_Malloc(sizeof(TIM_DMA_HandleType_Size));
        if (To_TimerPWMObj_Ptr(To_DShot_Obj(obj)->p_timr_obj)->dma_hdl == NULL)
        {
            Actuator_Free(To_TimerPWMObj_Ptr(To_DShot_Obj(obj)->p_timr_obj)->tim_hdl);
            Actuator_Free(To_TimerPWMObj_Ptr(To_DShot_Obj(obj)->p_timr_obj)->dma_hdl);
            Actuator_Free(To_DShot_Obj(obj)->p_timr_obj);
            return false;
        }
#elif defined AT32F435_437
        To_TimerPWMObj_Ptr(To_DShot_Obj(obj)->p_timr_obj)->dma_callback_obj = Actuator_Malloc(sizeof(BspDMA_IrqCall_Obj_TypeDef));
        if (To_TimerPWMObj_Ptr(To_DShot_Obj(obj)->p_timr_obj)->dma_callback_obj == NULL)
            return false;
#endif

        if (!BspTimer_PWM.init(To_TimerPWMObj_Ptr(To_DShot_Obj(obj)->p_timr_obj), \
                               time_ins, time_ch, \
                               (MOTOR_BITLENGTH - 1), prescaler, \
                               *(BspGPIO_Obj_TypeDef *)pin, dma, stream, \
                               (uint32_t)(To_DShot_Obj(obj)->ctl_buf), DSHOT_DMA_BUFFER_SIZE))
            return false;

        To_TimerPWMObj_Ptr(To_DShot_Obj(obj)->p_timr_obj)->send_callback = DShot_Tran_Finish;
        BspTimer_PWM.set_dma_pwm((To_TimerPWMObj_Ptr(To_DShot_Obj(obj)->p_timr_obj)));
        return true;
    }

    return false;
}


