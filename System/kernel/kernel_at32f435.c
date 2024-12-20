#include "at32f435_437.h"
#include "at32f435_437_clock.h"
#include "kernel.h"
#include <stdbool.h>

#define Kernel_DisableIRQ() __asm("cpsid i")
#define Kernel_EnableIRQ() __asm("cpsie i")

#define Kernel_UID_BitSize 96

static bool Kernel_TickTimer_Init = false;
static Kernel_UID_TypeDef UID;

extern uint32_t __rom_s;
extern uint32_t __boot_e;

void Kernel_Read_UID(void);

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
    
    // nvic_vector_table_set(NVIC_VECTTAB_FLASH, ((uint32_t)&__boot_e - (uint32_t)&__rom_s));
    __enable_irq();

    /* noticed */
    /* when use Betaflight Configurator tool IAP flight controller with all disk erase selection on */
    /* this project will fall into hardfault somehow when procedure on libc_init_array */
    /* this because ram address been change when doing full disk erase */
    /* all we need to do is temporary modify .ld file as */                        
    /* RAM (xrw)       : ORIGIN = 0x20010000, LENGTH = 128K */
    /* re-compile the project and download to ur flight controller */
    /* when system boot up successed */
    /* we can modify RAM Origin address back to default 0x20000000 and length as 384k */
    /* RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 384K */

    /* volatile uint8_t *SRAM_Setting = 0x1FFFC010; */
    /* set as sram 384k */
    /* volatile uint8_t Data = 0b10; */
    
    /* data in SRAM_Setting address */
    /* 0b000：on chip SRAM 512K Byte + ZW 128K Byte */
    /* 0b001：on chip SRAM 448K Byte + ZW 192K Byte */
    /* 0b010：on chip SRAM 384K Byte + ZW 256K Byte */
    /* 0b011：on chip SRAM 320K Byte + ZW 320K Byte */
    /* 0b100：on chip SRAM 256K Byte + ZW 384K Byte */
    /* 0b101：on chip SRAM 192K Byte + ZW 448K Byte */

    flash_unlock();
    flash_user_system_data_erase();
    flash_user_system_data_program(0x1FFFC010, 0b10);
    flash_lock();
    /* noticed */

    /* enable tmr1 */
    tmr_counter_enable(TMR20, TRUE);

    /* get UID */
    Kernel_Read_UID();

    Kernel_TickTimer_Init = true;

    return true;
}

void Kernel_Read_UID(void)
{
    volatile uint32_t *UID_Addr_tmp;

    memset(&UID, 0, sizeof(Kernel_UID_TypeDef));
    UID.BitSize = Kernel_UID_BitSize;
    UID.ByteSize = (UID.BitSize / 8) + (UID.BitSize % 8);

    /* UID Address 1 0x1FFFF7E8 */
    /* UID Address 2 0x1FFFF7EC */
    /* UID Address 3 0x1FFFF7F0 */
    UID_Addr_tmp = 0x1FFFF7E8;
    UID.UID[0] = *(uint32_t *)UID_Addr_tmp;

    UID_Addr_tmp = 0x1FFFF7EC;
    UID.UID[1] = *(uint32_t *)UID_Addr_tmp;

    UID_Addr_tmp = 0x1FFFF7F0;
    UID.UID[2] = *(uint32_t *)UID_Addr_tmp;
}

Kernel_UID_TypeDef Kernel_Get_UID(void)
{
    return UID;
}

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
