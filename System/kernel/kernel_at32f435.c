#include "at32f435_437.h"
#include <stdbool.h>

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

    return true;
}

void Kernel_reboot(void)
{

}

