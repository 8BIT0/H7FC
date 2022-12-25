#include "Bsp_Timer.h"
#include "Bsp_DMA.h"
#include "Bsp_GPIO.h"

typedef struct
{
    TIM_TypeDef *list[32];
    uint8_t init_cnt;
    bool monitor_init;
} BspTIM_PWMInitMonitor_TypeDef;

BspTIM_PWMInitMonitor_TypeDef monitor = {
    .monitor_init = false,
};

static bool BspTimer_PWM_InitMonit(TIM_TypeDef *tim)
{
    if (!monitor.monitor_init)
    {
        monitor.init_cnt = 0;
        memset(monitor.list, NULL, sizeof(monitor.list) / sizeof(monitor.list[0]));
        monitor.monitor_init = true;
    }

    if (monitor.init_cnt)
    {
        for (uint8_t i = 0; i < monitor.init_cnt; i++)
        {
            if (hdl.Instance == monitor.list[i]->Instance)
                return false;
        }
    }

    if (monitor.init_cnt < sizeof(monitor.list) / sizeof(monitor.list[0]))
    {
        if (tim == TIM1)
        {
            __HAL_RCC_TIM1_CLK_ENABLE();
        }
        else if (tim == TIM2)
        {
            __HAL_RCC_TIM2_CLK_ENABLE();
        }
        else if (tim == TIM3)
        {
            __HAL_RCC_TIM3_CLK_ENABLE();
        }
        else if (tim == TIM4)
        {
            __HAL_RCC_TIM4_CLK_ENABLE();
        }
        else if (tim == TIM5)
        {
            __HAL_RCC_TIM5_CLK_ENABLE();
        }
        else if (tim == TIM6)
        {
            __HAL_RCC_TIM6_CLK_ENABLE();
        }
        else if (hdl->Instance == TIM7)
        {
            __HAL_RCC_TIM7_CLK_ENABLE();
        }
        else if (hdl->Instance == TIM8)
        {
            __HAL_RCC_TIM8_CLK_ENABLE();
        }
        else if (hdl->Instance == TIM12)
        {
            __HAL_RCC_TIM12_CLK_ENABLE();
        }
        else if (hdl->Instance == TIM13)
        {
            __HAL_RCC_TIM13_CLK_ENABLE();
        }
        else if (hdl->Instance == TIM14)
        {
            __HAL_RCC_TIM14_CLK_ENABLE();
        }
        else if (hdl->Instance == TIM15)
        {
            __HAL_RCC_TIM15_CLK_ENABLE();
        }
        else if (hdl->Instance == TIM16)
        {
            __HAL_RCC_TIM16_CLK_ENABLE();
        }
        else if (hdl->Instance == TIM17)
        {
            __HAL_RCC_TIM17_CLK_ENABLE();
        }
        else
            return false;

        monitor.list[monitor.init_cnt] = tim;
        monitor.init_cnt++;
    }

    return true;
}

static bool BspTimer_PWM_Init(BspTimerPWMObj_TypeDef *obj, TIM_TypeDef *instance, uint32_t ch, uint8_t dma, uint8_t stream)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    if (BspTimer_PWM_InitMonit(instance))
    {
        obj->instance = instance;
        obj->tim_hdl.Instance = instance;
        obj->tim_hdl.Init.Prescaler = 0;
        obj->tim_hdl.Init.CounterMode = TIM_COUNTERMODE_UP;
        obj->tim_hdl.Init.Period = 0;
        obj->tim_hdl.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        obj->tim_hdl.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

        if (HAL_TIM_PWM_Init(&(obj->tim_hdl)) != HAL_OK)
            return false;

        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&(obj->tim_hdl), &sMasterConfig) != HAL_OK)
            return false;
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    switch (ch)
    {
    case TIM_CHANNEL_1:
        obj->tim_cc = TIM_DMA_ID_CC1;
    case TIM_CHANNEL_2:
        obj->tim_cc = TIM_DMA_ID_CC2;
    case TIM_CHANNEL_3:
        obj->tim_cc = TIM_DMA_ID_CC3;
    case TIM_CHANNEL_4:
        obj->tim_cc = TIM_DMA_ID_CC4;
        if (HAL_TIM_PWM_ConfigChannel(&(obj->tim_hdl), &sConfigOC, ch) != HAL_OK)
            return false;
        obj->tim_channel = ch;
        break;

    default:
        return false;
    }

    /* pin init */
    BspGPIO.alt_init();

    /* dma init */
    obj->dma_hdl.Instance = BspDMA.get_instance(dma, stream);
    obj->dma_hdl.Init.Request = DMA_REQUEST_MEM2MEM;
    obj->dma_hdl.Init.Direction = DMA_MEMORY_TO_MEMORY;
    obj->dma_hdl.Init.PeriphInc = DMA_PINC_ENABLE;
    obj->dma_hdl.Init.MemInc = DMA_MINC_ENABLE;
    obj->dma_hdl.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    obj->dma_hdl.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    obj->dma_hdl.Init.Mode = DMA_NORMAL;
    obj->dma_hdl.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    obj->dma_hdl.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    obj->dma_hdl.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    obj->dma_hdl.Init.MemBurst = DMA_MBURST_SINGLE;
    obj->dma_hdl.Init.PeriphBurst = DMA_PBURST_SINGLE;

    if (HAL_DMA_Init(&obj->dma_hdl) != HAL_OK)
        return false;

    __HAL_LINKDMA(&(obj->tim_hdl), hdma[TIM_DMA_ID_CC1], obj->dma_hdl);

    /* dma regist */
    BspDMA.regist(dma, stream, obj->dma_hdl);

    /* init DMA IRQ */
    HAL_NVIC_SetPriority(irqn, 5, 0);
    HAL_NVIC_EnableIRQ(irqn);
}

static void BspTimer_Set()
{
}
