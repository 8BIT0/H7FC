#include "Bsp_Timer.h"

static bool BspTimer_Init()
{
    /* init DMA IRQ */
    HAL_NVIC_SetPriority(irqn, 5, 0);
    HAL_NVIC_EnableIRQ(irqn);
}

static void BspTimer_Set()
{
}
