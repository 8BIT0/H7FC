#include "Bsp_Dshot_BitBang.h"
#include "Bsp_GPIO.h"
#include "Bsp_DMA.h"
#include "Bsp_Timer.h"

typedef struct
{
    bool monitor_init;
    BspTiemrClk_EnReg_TypeDef clk_en;
} BspDshotBitBang_InitMonitor_TypeDef;

static BspDshotBitBang_InitMonitor_TypeDef Monitor = {
    .monitor_init = false,
};

static bool BspDshotBitBang_TMRClockEn_Ctl(tmr_type *tmr, confirm_state state)
{
    if (tmr == NULL)
        return false;

    if ((!Monitor.clk_en.bit.tim1) && (tmr == TMR1))
    {
        crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, state);
        Monitor.clk_en.bit.tim1 = true;
        return true;
    }

    if ((!Monitor.clk_en.bit.tim2) && (tmr == TMR2))
    {
        crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, state);
        Monitor.clk_en.bit.tim2 = true;
        return true;
    }

    if ((!Monitor.clk_en.bit.tim3) && (tmr == TMR3))
    {
        crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, state);
        Monitor.clk_en.bit.tim3 = true;
        return true;
    }

    if ((!Monitor.clk_en.bit.tim4) && (tmr == TMR4))
    {
        crm_periph_clock_enable(CRM_TMR4_PERIPH_CLOCK, state);
        Monitor.clk_en.bit.tim4 = true;
        return true;
    }

    if ((!Monitor.clk_en.bit.tim5) && (tmr == TMR5))
    {
        crm_periph_clock_enable(CRM_TMR5_PERIPH_CLOCK, state);
        Monitor.clk_en.bit.tim5 = true;
        return true;
    }

    if ((!Monitor.clk_en.bit.tim6) && (tmr == TMR6))
    {
        crm_periph_clock_enable(CRM_TMR6_PERIPH_CLOCK, state);
        Monitor.clk_en.bit.tim6 = true;
        return true;
    }

    if ((!Monitor.clk_en.bit.tim7) && (tmr == TMR7))
    {
        crm_periph_clock_enable(CRM_TMR7_PERIPH_CLOCK, state);
        Monitor.clk_en.bit.tim7 = true;
        return true;
    }


    if ((!Monitor.clk_en.bit.tim8) && (tmr == TMR8))
    {
        crm_periph_clock_enable(CRM_TMR8_PERIPH_CLOCK, state);
        Monitor.clk_en.bit.tim8 = true;
        return true;
    }

    return false;
}

static bool BspDshotBitBang_TimerInit(BspTimerPWMObj_TypeDef *obj, \
                                      uint32_t period, \
                                      void *instance, \
                                      uint32_t ch, \
                                      BspGPIO_Obj_TypeDef pin, \
                                      uint8_t dma, \
                                      uint8_t stream, \
                                      uint32_t buf_addr, \
                                      uint32_t buf_size)
{
    tmr_output_config_type  TIM_OCStruct;
	dma_init_type dmainit;

    if ((obj == NULL) || \
        (period == 0) || \
        (instance == NULL) || \
        (buf_addr == 0) || \
        (buf_size == 0))
        return false;

    if (!Monitor.monitor_init)
    {
        memset(&Monitor, 0, sizeof(BspDshotBitBang_InitMonitor_TypeDef));
        Monitor.monitor_init = true;
    }

    obj->instance = instance;
    obj->buffer_addr = buf_addr;
    obj->buffer_size = buf_size;
    obj->dma = dma;
    obj->stream = stream;
    obj->dma_hdl = BspDMA.get_instance(obj->dma, obj->stream);

    if (obj->dma_hdl == NULL)
        return false;

    /* pin init */
    if (!BspGPIO.out_init(pin))
        return false;

    if (!BspDshotBitBang_TMRClockEn_Ctl(To_Timer_Instance(instance), TRUE))
        return false;
    
    /* timer init */
    tmr_base_init(To_Timer_Instance(instance), period, 0);
    tmr_clock_source_div_set(To_Timer_Instance(instance), TMR_CLOCK_DIV1);
    tmr_cnt_dir_set(To_Timer_Instance(instance), TMR_COUNT_UP);
    tmr_period_buffer_enable(To_Timer_Instance(instance), TRUE);

    /* channel init */
	tmr_output_default_para_init(&TIM_OCStruct);
	TIM_OCStruct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    TIM_OCStruct.oc_idle_state = TRUE;
    TIM_OCStruct.oc_output_state = TRUE;
	TIM_OCStruct.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
    tmr_channel_value_set(To_Timer_Instance(instance), ch, 0);

    tmr_counter_enable(To_Timer_Instance(instance), FALSE);
    tmr_output_channel_config(To_Timer_Instance(instance), ch, &TIM_OCStruct);
    tmr_channel_enable(To_Timer_Instance(instance), ch, TRUE);
    tmr_output_channel_buffer_enable(To_Timer_Instance(instance), ch, TRUE);
    tmr_counter_enable(To_Timer_Instance(instance), TRUE);

    /* dma init */
    obj->dma_hdl = BspDMA.get_instance(obj->dma, obj->stream);

	dma_default_para_init(&dmainit);
	dmainit.loop_mode_enable = FALSE;
	dmainit.peripheral_inc_enable =FALSE;
	dmainit.memory_inc_enable = TRUE;

    dmainit.priority = DMA_PRIORITY_VERY_HIGH;
    dmainit.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dmainit.buffer_size = obj->buffer_size;
    dmainit.peripheral_base_addr = (uint32_t)&To_GPIO_Port(pin.port)->port->scr;
    dmainit.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dmainit.memory_base_addr = obj->buffer_addr;
    dmainit.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init(obj->dma_hdl, &dmainit);

    /* irq enable */
    BspDMA.enable_irq();

    return false;
}
