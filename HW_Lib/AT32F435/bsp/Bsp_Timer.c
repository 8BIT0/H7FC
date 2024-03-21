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

BspTimerPWM_TypeDef BspTimer_PWM = {
    .init = BspTimer_PWM_Init,
    .set_prescaler = BspTimer_SetPreScale,
    .set_autoreload = BspTimer_SetAutoReload,
    .start_pwm = BspTimer_PWM_Start,
    .dma_trans = BspTimer_DMA_Start,
};

static bool BspTimer_Clock_Enable(void *instance)
{
    if (To_Timer_Instance(instance) == TMR1)
    {
        crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
        return true;
    }
    else if (To_Timer_Instance(instance) == TMR2)
    {
        crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
        return true;
    }
    else if (To_Timer_Instance(instance) == TMR3)
    {
        crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
        return true;
    }
    else if (To_Timer_Instance(instance) == TMR4)
    {
        crm_periph_clock_enable(CRM_TMR4_PERIPH_CLOCK, TRUE);
        return true;
    }
    else if (To_Timer_Instance(instance) == TMR5)
    {
        crm_periph_clock_enable(CRM_TMR5_PERIPH_CLOCK, TRUE);
        return true;
    }
    else if (To_Timer_Instance(instance) == TMR6)
    {
        crm_periph_clock_enable(CRM_TMR6_PERIPH_CLOCK, TRUE);
        return true;
    }
    else if (To_Timer_Instance(instance) == TMR7)
    {
        crm_periph_clock_enable(CRM_TMR7_PERIPH_CLOCK, TRUE);
        return true;
    }
    else if (To_Timer_Instance(instance) == TMR8)
    {
        crm_periph_clock_enable(CRM_TMR8_PERIPH_CLOCK, TRUE);
        return true;
    }
    else if (To_Timer_Instance(instance) == TMR9)
    {
        crm_periph_clock_enable(CRM_TMR9_PERIPH_CLOCK, TRUE);
        return true;
    }
    else if (To_Timer_Instance(instance) == TMR10)
    {
        crm_periph_clock_enable(CRM_TMR10_PERIPH_CLOCK, TRUE);
        return true;
    }
    else if (To_Timer_Instance(instance) == TMR11)
    {
        crm_periph_clock_enable(CRM_TMR11_PERIPH_CLOCK, TRUE);
        return true;
    }
    else if (To_Timer_Instance(instance) == TMR12)
    {
        crm_periph_clock_enable(CRM_TMR12_PERIPH_CLOCK, TRUE);
        return true;
    }
    else if (To_Timer_Instance(instance) == TMR13)
    {
        crm_periph_clock_enable(CRM_TMR13_PERIPH_CLOCK, TRUE);
        return true;
    }
    else if (To_Timer_Instance(instance) == TMR14)
    {
        crm_periph_clock_enable(CRM_TMR14_PERIPH_CLOCK, TRUE);
        return true;
    }
    else if (To_Timer_Instance(instance) == TMR20)
    {
        crm_periph_clock_enable(CRM_TMR20_PERIPH_CLOCK, TRUE);
        return true;
    }

    return false;
}

static bool BspTimer_DMA_Init(BspTimerPWMObj_TypeDef *obj)
{
    dma_init_type dma_init_struct;
    dma_channel_type *p_dma_channel = NULL;

    if ((obj == NULL) || \
        (obj->instance == NULL) || \
        (obj->dma_hdl == NULL) || \
        (obj->buffer_addr == 0) || \
        (obj->buffer_size == 0))
        return false;

    p_dma_channel = BspDMA.get_channel_index(obj->dma, obj->stream);
    
    if (p_dma_channel)
    {
        /* enable tmr1 overflow dma request */
        tmr_dma_request_enable(obj->instance, TMR_OVERFLOW_DMA_REQUEST, TRUE);
        tmr_dma_control_config(obj->instance, TMR_DMA_TRANSFER_18BYTES, TMR_PR_ADDRESS);

        dma_reset(p_dma_channel);
        dma_init_struct.buffer_size = obj->buffer_size;
        dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
        dma_init_struct.memory_base_addr = (uint32_t)obj->buffer_addr;
        dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
        dma_init_struct.memory_inc_enable = TRUE;
        dma_init_struct.peripheral_base_addr = (uint32_t)To_Timer_Instance(obj->instance)->dmadt;
        dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
        dma_init_struct.peripheral_inc_enable = FALSE;
        dma_init_struct.priority = DMA_PRIORITY_VERY_HIGH;
        dma_init_struct.loop_mode_enable = FALSE;
        dma_init(p_dma_channel, &dma_init_struct);
    }
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
    tmr_output_config_type  tmr_output_struct;

    if (!monitor.monitor_init)
    {
        memset(monitor.list, 0, sizeof(monitor.list) / sizeof(monitor.list[0]));
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
    obj->buffer_addr = buf_aadr;
    obj->buffer_size = buf_size;
    obj->instance = instance;
    obj->tim_channel = ch;
    obj->dma_hdl = BspDMA.get_instance(dma, stream);

    /* init pin */
    if (!BspGPIO.alt_init(pin, 0))
        return false;

    /* init pwm output */
    // tmr_base_init(obj->instance, , );
    tmr_cnt_dir_set(obj->instance, TMR_COUNT_UP);
    tmr_output_default_para_init(&tmr_output_struct);

    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
    tmr_output_struct.oc_idle_state = TRUE;
    tmr_output_struct.occ_output_state = FALSE;
    tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_LOW;
    tmr_output_struct.occ_idle_state = FALSE;

    tmr_output_channel_config(obj->instance, obj->tim_channel, &tmr_output_struct);
    // tmr_channel_value_set(obj->instance, obj->tim_channel, );

    if (obj->dma_hdl)
    {
        /* dma init */
    }

    return true;
}

static void BspTimer_SetPreScale(BspTimerPWMObj_TypeDef *obj, uint32_t prescale)
{

}

static void BspTimer_SetAutoReload(BspTimerPWMObj_TypeDef *obj, uint32_t auto_reload)
{

}

static void BspTimer_PWM_Start(BspTimerPWMObj_TypeDef *obj)
{
    
}

static void BspTimer_DMA_Start(BspTimerPWMObj_TypeDef *obj)
{
    
}

