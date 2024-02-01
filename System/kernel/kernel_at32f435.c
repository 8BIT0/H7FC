#include "at32f435_437.h"
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
    tmr_base_init(TMR20, 1999, (crm_clocks_freq_struct.apb2_freq / 1000000) - 1);
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

}

bool Kernel_DisableTimer_IRQ(void)
{

}

bool Kernel_Set_PeriodValue(uint32_t value)
{

}

bool Kernel_Set_SysTimer_TickUnit(uint32_t unit)
{
    if(Kernel_TickTimer_Init)
    {

    }

    return false;
}

uint32_t Kernel_Get_SysTimer_TickUnit(void)
{
    if(Kernel_TickTimer_Init)
    {

    }

    return 0;
}

uint32_t Kernel_Get_PeriodValue(void)
{

}

uint32_t Kernel_TickVal_To_Us(void)
{

}
