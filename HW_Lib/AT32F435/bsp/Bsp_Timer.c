#include "Bsp_Timer.h"
#include "Bsp_DMA.h"
#include "Bsp_GPIO.h"
#include "at32f435_437_crm.h"
#include "at32f435_437_tmr.h"

#define To_Timer_Instance(x) ((tmr_type *)x)

typedef struct
{
    tmr_type *list[32];
    uint8_t init_cnt;
    bool monitor_init;
} BspTIM_PWMInitMonitor_TypeDef;

BspTIM_PWMInitMonitor_TypeDef monitor = {
    .monitor_init = false,
};

/* internal function */
static void BspTimer_DMA_TransCplt_Callback(void *arg);

/* external function */
static bool BspTimer_PWM_Init(BspTimerPWMObj_TypeDef *obj,
                              void *instance,
                              uint32_t ch,
                              BspGPIO_Obj_TypeDef pin,
                              uint8_t dma,
                              uint8_t stream,
                              uint32_t buf_aadr,
                              uint32_t buf_size);
static void BspTimer_SetPreScale(BspTimerPWMObj_TypeDef *obj, uint32_t prescale);
static void BspTimer_SetAutoReload(BspTimerPWMObj_TypeDef *obj, uint32_t auto_reload);
static void BspTimer_PWM_Start(BspTimerPWMObj_TypeDef *obj);
static void BspTimer_DMA_Start(BspTimerPWMObj_TypeDef *obj);
static uint32_t BspTimer_Get_Clock_Freq(BspTimerPWMObj_TypeDef *obj);

BspTimerPWM_TypeDef BspTimer_PWM = {
    .init = BspTimer_PWM_Init,
    .set_prescaler = BspTimer_SetPreScale,
    .set_autoreload = BspTimer_SetAutoReload,
    .start_pwm = BspTimer_PWM_Start,
    .dma_trans = BspTimer_DMA_Start,
    .get_clock_freq = BspTimer_Get_Clock_Freq,
};

static bool BspTimer_Clock_Enable(void *instance)
{
    if (To_Timer_Instance(instance) == TMR2)
    {
        crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
        return true;
    }
    else if (To_Timer_Instance(instance) == TMR3)
    {
        crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
        return true;
    }
    else if (To_Timer_Instance(instance) == TMR8)
    {
        crm_periph_clock_enable(CRM_TMR8_PERIPH_CLOCK, TRUE);
        return true;
    }

    return false;
}

static dmamux_requst_id_sel_type BspTimer_Get_DMA_MuxSeq(BspTimerPWMObj_TypeDef *obj)
{
    if (obj)
    {
        switch((uint32_t)obj->instance)
        {
            case (uint32_t)TMR2:
                switch(obj->tim_channel)
                {
                    case TMR_SELECT_CHANNEL_1: return DMAMUX_DMAREQ_ID_TMR2_CH1;
                    case TMR_SELECT_CHANNEL_2: return DMAMUX_DMAREQ_ID_TMR2_CH2;
                    case TMR_SELECT_CHANNEL_3: return DMAMUX_DMAREQ_ID_TMR2_CH3;
                    case TMR_SELECT_CHANNEL_4: return DMAMUX_DMAREQ_ID_TMR2_CH4;
                    default: return 0;
                }
                
            case (uint32_t)TMR3:
                switch(obj->tim_channel)
                {
                    case TMR_SELECT_CHANNEL_1: return DMAMUX_DMAREQ_ID_TMR3_CH1;
                    case TMR_SELECT_CHANNEL_2: return DMAMUX_DMAREQ_ID_TMR3_CH2;
                    case TMR_SELECT_CHANNEL_3: return DMAMUX_DMAREQ_ID_TMR3_CH3;
                    case TMR_SELECT_CHANNEL_4: return DMAMUX_DMAREQ_ID_TMR3_CH4;
                    default: return 0;
                }
                
            case (uint32_t)TMR8:
                switch(obj->tim_channel)
                {
                    case TMR_SELECT_CHANNEL_1: return DMAMUX_DMAREQ_ID_TMR8_CH1;
                    case TMR_SELECT_CHANNEL_2: return DMAMUX_DMAREQ_ID_TMR8_CH2;
                    case TMR_SELECT_CHANNEL_3: return DMAMUX_DMAREQ_ID_TMR8_CH3;
                    case TMR_SELECT_CHANNEL_4: return DMAMUX_DMAREQ_ID_TMR8_CH4;
                    default: return 0;
                }
                
            default:
                return 0;
        }
    }

    return 0;   
}

static uint32_t BspTimer_Get_PrtiphAddr(BspTimerPWMObj_TypeDef *obj)
{
    if (obj && obj->instance && obj->tim_channel)
    {
        switch(obj->tim_channel)
        {
            case TMR_SELECT_CHANNEL_1:
                return (uint32_t)&(To_Timer_Instance(obj->instance)->c1dt);

            case TMR_SELECT_CHANNEL_2:
                return (uint32_t)&(To_Timer_Instance(obj->instance)->c2dt);

            case TMR_SELECT_CHANNEL_3:
                return (uint32_t)&(To_Timer_Instance(obj->instance)->c3dt);

            case TMR_SELECT_CHANNEL_4:
                return (uint32_t)&(To_Timer_Instance(obj->instance)->c4dt);
            
            default:
                break;
        }
    }

    return 0;
}

static bool BspTimer_DMA_Init(BspTimerPWMObj_TypeDef *obj)
{
    dma_init_type dma_init_struct;
    dmamux_requst_id_sel_type dma_req_id = 0;
    tmr_dma_request_type dma_req_type = 0;

    if ((obj == NULL) || \
        (obj->instance == NULL) || \
        (obj->buffer_addr == 0) || \
        (obj->dma_hdl == NULL) || \
        (obj->buffer_size == 0))
        return false;

    switch(obj->tim_channel)
    {
        case TMR_SELECT_CHANNEL_1: dma_req_type = TMR_C1_DMA_REQUEST; break;
        case TMR_SELECT_CHANNEL_2: dma_req_type = TMR_C2_DMA_REQUEST; break;
        case TMR_SELECT_CHANNEL_3: dma_req_type = TMR_C3_DMA_REQUEST; break;
        case TMR_SELECT_CHANNEL_4: dma_req_type = TMR_C4_DMA_REQUEST; break;
        default: return false;
    }

    tmr_dma_request_enable(obj->instance, dma_req_type, TRUE);
    dma_req_id = BspTimer_Get_DMA_MuxSeq(obj);

    if (dma_req_id)
    {
        /* enable timer output dma request */
        dmamux_init(obj->dma_hdl, dma_req_id);
        dma_reset(obj->dma_hdl);
        dma_init_struct.buffer_size = obj->buffer_size;
        dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
        dma_init_struct.memory_base_addr = (uint32_t)obj->buffer_addr;
        dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
        dma_init_struct.memory_inc_enable = TRUE;
        dma_init_struct.peripheral_base_addr = BspTimer_Get_PrtiphAddr(obj);
        dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
        dma_init_struct.peripheral_inc_enable = FALSE;
        dma_init_struct.priority = DMA_PRIORITY_HIGH;
        dma_init_struct.loop_mode_enable = FALSE;
        dma_init(obj->dma_hdl, &dma_init_struct);

        return true;
    }

    return false;
}

static bool BspTimer_PWM_Init(BspTimerPWMObj_TypeDef *obj,
                              void *instance,
                              uint32_t ch,
                              BspGPIO_Obj_TypeDef pin,
                              uint8_t dma,
                              uint8_t stream,
                              uint32_t buf_aadr,
                              uint32_t buf_size)
{
    tmr_output_config_type tmr_output_struct;
    // tmr_dma_address_type dma_addr = 0;

    memset(&tmr_output_struct, 0, sizeof(tmr_output_config_type));

    if (!monitor.monitor_init)
    {
        memset(monitor.list, 0, sizeof(monitor.list));
        monitor.init_cnt = 0;
        monitor.monitor_init = true;
    }

    if ((obj == NULL) || \
        (instance == NULL) || \
        (buf_aadr == 0) || \
        (buf_size == 0))
        return false;

    if (!BspTimer_Clock_Enable(instance))
        return false;

    obj->dma = dma;
    obj->stream = stream;
    obj->buffer_addr = buf_aadr;
    obj->buffer_size = buf_size;
    obj->instance = instance;
    obj->tim_channel = ch;
    obj->dma_hdl = BspDMA.get_instance(dma, stream);

    /* init pin */
    if (!BspGPIO.alt_init(pin, 0))
        return false;

    /* init pwm output */
    tmr_cnt_dir_set(obj->instance, TMR_COUNT_UP);
    tmr_output_default_para_init(&tmr_output_struct);
    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.oc_idle_state = FALSE;
    // tmr_output_struct.occ_output_state = FALSE;
    // tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    // tmr_output_struct.occ_idle_state = FALSE;
    tmr_output_channel_config(obj->instance, obj->tim_channel, &tmr_output_struct);

    // switch(obj->tim_channel)
    // {
    //     case TMR_SELECT_CHANNEL_1: dma_addr = TMR_C1DT_ADDRESS; break;
    //     case TMR_SELECT_CHANNEL_2: dma_addr = TMR_C2DT_ADDRESS; break;
    //     case TMR_SELECT_CHANNEL_3: dma_addr = TMR_C3DT_ADDRESS; break;
    //     case TMR_SELECT_CHANNEL_4: dma_addr = TMR_C4DT_ADDRESS; break;
    //     default: return false;
    // }

    // tmr_dma_control_config(obj->instance, TMR_DMA_TRANSFER_18BYTES, dma_addr);

    if (obj->dma_hdl && BspTimer_DMA_Init(obj))
    {
        if (obj->dma_callback_obj)
        {
            To_DMA_IrqCallbackObj_Ptr(obj->dma_callback_obj)->cus_data = (void *)obj;
            To_DMA_IrqCallbackObj_Ptr(obj->dma_callback_obj)->BspDMA_Irq_Callback_Func = BspTimer_DMA_TransCplt_Callback;
        }

        BspDMA.enable_irq(dma, stream, 5, 0, BspTimer_Get_DMA_MuxSeq(obj), obj->dma_callback_obj);

        monitor.list[monitor.init_cnt] = obj->instance;
        monitor.init_cnt ++;

        return true;
    }

    return false;
}

static uint32_t BspTimer_Get_Clock_Freq(BspTimerPWMObj_TypeDef *obj)
{
    crm_clocks_freq_type crm_clocks_freq_struct;
    UNUSED(obj);

    memset(&crm_clocks_freq_struct, 0, sizeof(crm_clocks_freq_type));
    crm_clocks_freq_get(&crm_clocks_freq_struct);
    return (crm_clocks_freq_struct.apb2_freq * 2);
}

static void BspTimer_SetPreScale(BspTimerPWMObj_TypeDef *obj, uint32_t prescale)
{
    if (obj && obj->instance)
    {
        obj->prescale = prescale;
        tmr_base_init(obj->instance, obj->auto_reload, prescale);
    }
}

static void BspTimer_SetAutoReload(BspTimerPWMObj_TypeDef *obj, uint32_t auto_reload)
{
    if (obj && obj->instance && obj->tim_channel)
    {
        obj->auto_reload = auto_reload;
        tmr_base_init(obj->instance, auto_reload, obj->prescale);
        // tmr_channel_value_set(obj->instance, obj->tim_channel, auto_reload);
    }
}

static void BspTimer_PWM_Start(BspTimerPWMObj_TypeDef *obj)
{
    if (obj && obj->instance)
    {
        /* nothing to do */
    }
}

/* test code */
// uint8_t send_test = 0;
// uint8_t cb_test = 0;
/* test code */

static void BspTimer_DMA_Start(BspTimerPWMObj_TypeDef *obj)
{
    if (obj && obj->dma_hdl)
    {
        // send_test ++;
        To_DMA_Handle_Ptr(obj->dma_hdl)->paddr = BspTimer_Get_PrtiphAddr(obj);
        To_DMA_Handle_Ptr(obj->dma_hdl)->maddr = obj->buffer_addr;
        To_DMA_Handle_Ptr(obj->dma_hdl)->dtcnt = obj->buffer_size * 2;
        dma_channel_enable(To_DMA_Handle_Ptr(obj->dma_hdl), TRUE);
        tmr_counter_enable(To_Timer_Instance(obj->instance), TRUE);

        /* test code */
        // while(cb_test < send_test);
    }
}

static void BspTimer_DMA_TransCplt_Callback(void *arg)
{
    BspTimerPWMObj_TypeDef *obj = NULL;

    if (arg && To_TimerPWMObj_Ptr(arg)->dma_hdl)
    {
        // cb_test ++;
        obj = To_TimerPWMObj_Ptr(arg);
        dma_channel_enable(To_DMA_Handle_Ptr(obj->dma_hdl), FALSE);
        tmr_counter_enable(To_Timer_Instance(obj->instance), FALSE);
    }
}

