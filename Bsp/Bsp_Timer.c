#include "Bsp_Timer.h"
#include "Bsp_DMA.h"

static bool BspTimer_PWM_Init(BspTimerPWMObj_TypeDef *obj, TIM_TypeDef *instance, uint32_t tim_channel, uint8_t dma, uint8_t stream, uint32_t dma_channel)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    obj->instance = instance;
    obj->hdl.Instance = instance;
    obj->hdl.Init.Prescaler = 0;
    obj->hdl.Init.CounterMode = TIM_COUNTERMODE_UP;
    obj->hdl.Init.Period = 0;
    obj->hdl.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    obj->hdl.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&(obj->hdl)) != HAL_OK)
        return false;

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&(obj->hdl), &sMasterConfig) != HAL_OK)
        return false;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    switch (channel)
    {
    case TIM_CHANNEL_1:
    case TIM_CHANNEL_2:
    case TIM_CHANNEL_3:
    case TIM_CHANNEL_4:
    case TIM_CHANNEL_5:
    case TIM_CHANNEL_6:
        if (HAL_TIM_PWM_ConfigChannel(&(obj->hdl), &sConfigOC, channel) != HAL_OK)
            return false;

    default:
        return false;
    }

    obj->channel = channel;

    /* pin init */
    BspDMA.regist();

    /* init DMA IRQ */
    HAL_NVIC_SetPriority(irqn, 5, 0);
    HAL_NVIC_EnableIRQ(irqn);
}

static void BspTimer_Set()
{
}
