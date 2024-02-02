#include "at32f435_437.h"
#include "at32f435_437_clock.h"
#include <stdbool.h>

#define Kernel_DisableIRQ() __asm("cpsid i")
#define Kernel_EnableIRQ() __asm("cpsie i")

static bool Kernel_TickTimer_Init = false;

bool Kernel_Init(void)
{
    system_clock_config();

    crm_clocks_freq_type crm_clocks_freq_struct = {0};

    /* enable tmr1 clock */
    crm_periph_clock_enable(CRM_TMR20_PERIPH_CLOCK, TRUE);

    crm_clocks_freq_get(&crm_clocks_freq_struct);

    /* tmr1 configuration */
    /* time base configuration */
    tmr_base_init(TMR20, 15999, (crm_clocks_freq_struct.apb2_freq / 8000000) - 1);
    tmr_cnt_dir_set(TMR20, TMR_COUNT_UP);

    /* overflow interrupt enable */
    tmr_interrupt_enable(TMR20, TMR_OVF_INT, TRUE);

    /* tmr1 hall interrupt nvic init */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(TMR20_OVF_IRQn, 14, 0);

    /* enable tmr1 */
    tmr_counter_enable(TMR20, TRUE);

    Kernel_TickTimer_Init = true;

    return true;
}

/* need test */
void Kernel_reboot(void)
{
    __set_FAULTMASK(1);

    NVIC_SystemReset();
}

bool Kernel_EnableTimer_IRQ(void)
{
    if(Kernel_TickTimer_Init)
    {
        tmr_interrupt_enable(TMR20, TMR_OVF_INT, TRUE);
        return true;
    }

    return false;
}

bool Kernel_DisableTimer_IRQ(void)
{
    if(Kernel_TickTimer_Init)
    {
        tmr_interrupt_enable(TMR20, TMR_OVF_INT, FALSE);
        return true;
    }

    return false;
}

bool Kernel_Set_PeriodValue(uint32_t value)
{
    int32_t set_diff = value;
    uint32_t cur_systick_period = 0;

    if(Kernel_TickTimer_Init)
    {
        /* sys default tick Period count is 10000 */
        /* 10000tick unit represent 1Ms, 1tick unit represent 100Ns */
        cur_systick_period = tmr_period_value_get(TMR20);
        set_diff -= cur_systick_period;

        /* single tune range 10Us to -10Us */
        if((set_diff <= 100) || (set_diff >= -100))
        {
            tmr_period_value_set(TMR20, value);
        }
        else if(set_diff > 100)
        {
            tmr_period_value_set(TMR20, cur_systick_period + 100);
        }
        else if(set_diff < -100)
        {
            tmr_period_value_set(TMR20, cur_systick_period - 100);
        }

        return true;
    }

    return false;
}

bool Kernel_Set_SysTimer_TickUnit(uint32_t unit)
{
    uint32_t addin = unit;

    if(Kernel_TickTimer_Init)
    {
        if (addin > tmr_period_value_get(TMR20))
            addin = tmr_period_value_get(TMR20);
    
        tmr_counter_value_set(TMR20, addin);
        return true;
    }

    return false;
}

uint32_t Kernel_Get_SysTimer_TickUnit(void)
{
    if (Kernel_TickTimer_Init)
        return tmr_counter_value_get(TMR20);

    return 0;
}

uint32_t Kernel_Get_PeriodValue(void)
{
    if (Kernel_TickTimer_Init)
        return tmr_period_value_get(TMR20);

    return 0;
}

uint32_t Kernel_TickVal_To_Us(void)
{
    if (Kernel_TickTimer_Init)
        return tmr_period_value_get(TMR20) / 1000;

    return 0;
}
