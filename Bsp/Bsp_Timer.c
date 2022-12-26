#include "Bsp_Timer.h"

typedef struct
{
    TIM_TypeDef *list[32];
    uint8_t init_cnt;
    bool monitor_init;
} BspTIM_PWMInitMonitor_TypeDef;

BspTIM_PWMInitMonitor_TypeDef monitor = {
    .monitor_init = false,
};

/* external function */
static bool BspTimer_PWM_Init(BspTimerPWMObj_TypeDef *obj,
                              TIM_TypeDef *instance,
                              uint32_t ch,
                              BspGPIO_Obj_TypeDef pin,
                              uint8_t dma,
                              uint8_t stream,
                              uint32_t buf_aadr,
                              uint32_t buf_size);

BspTimerPWM_TypeDef BspTimer_PWM = {
    .init = BspTimer_PWM_Init,
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
            if (tim == monitor.list[i])
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
        else
            return false;

        monitor.list[monitor.init_cnt] = tim;
        monitor.init_cnt++;
    }

    return true;
}

static bool BspTimer_PWM_Init(BspTimerPWMObj_TypeDef *obj, TIM_TypeDef *instance, uint32_t ch, BspGPIO_Obj_TypeDef pin, uint8_t dma, uint8_t stream, uint32_t buf_aadr, uint32_t buf_size)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    if ((obj == NULL) || (instance == NULL) || (buf_aadr == 0) || (buf_size == 0))
        return false;

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
        obj->tim_dma_id_cc = TIM_DMA_ID_CC1;
        obj->tim_dma_cc = TIM_DMA_CC1;
    case TIM_CHANNEL_2:
        obj->tim_dma_id_cc = TIM_DMA_ID_CC2;
        obj->tim_dma_cc = TIM_DMA_CC2;
    case TIM_CHANNEL_3:
        obj->tim_dma_id_cc = TIM_DMA_ID_CC3;
        obj->tim_dma_cc = TIM_DMA_CC3;
    case TIM_CHANNEL_4:
        obj->tim_dma_id_cc = TIM_DMA_ID_CC4;
        obj->tim_dma_cc = TIM_DMA_CC4;
        if (HAL_TIM_PWM_ConfigChannel(&(obj->tim_hdl), &sConfigOC, ch) != HAL_OK)
            return false;
        obj->tim_channel = ch;
        break;

    default:
        return false;
    }

    /* pin init */
    BspGPIO.alt_init(pin);

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

    __HAL_LINKDMA(&(obj->tim_hdl), hdma[obj->tim_dma_id_cc], obj->dma_hdl);

    /* dma regist */
    if (BspDMA.regist(dma, stream, &obj->dma_hdl))
    {
        /* init DMA IRQ */
        BspDMA.enable_irq(dma, stream, 5, 0);
        return true;
    }

    return false;
}

static void BspTimer_PWM_DMA_Enable(BspTimerPWMObj_TypeDef *obj)
{
    __HAL_TIM_ENABLE_DMA(&obj->tim_hdl, obj->tim_dma_cc);
}

static void BspTimer_PWM_Start(BspTimerPWMObj_TypeDef *obj)
{
    HAL_TIM_PWM_Start(&obj->tim_hdl, obj->tim_channel);
}

static void BspTimer_DMA_Start(BspTimerPWMObj_TypeDef *obj)
{
    uint32_t src_addr = 0;

    switch (obj->tim_dma_id_cc)
    {
    case TIM_DMA_ID_CC1:
        src_addr = (uint32_t) & (obj->tim_hdl.Instance->CCR1);
        break;

    case TIM_DMA_ID_CC2:
        src_addr = (uint32_t) & (obj->tim_hdl.Instance->CCR2);
        break;

    case TIM_DMA_ID_CC3:
        src_addr = (uint32_t) & (obj->tim_hdl.Instance->CCR3);
        break;

    case TIM_DMA_ID_CC4:
        src_addr = (uint32_t) & (obj->tim_hdl.Instance->CCR4);
        break;

    default:
        return;
    }

    HAL_DMA_Start_IT(obj->tim_hdl.hdma[obj->tim_dma_id_cc], obj->buffer_addr, src_addr, obj->buffer_size);
}

static void BspTimer_SetPreScale(BspTimerPWMObj_TypeDef *obj, uint32_t prescale)
{
    __HAL_TIM_SET_PRESCALER(&obj->tim_hdl, prescale);
}

static void BspTimer_SetAutoReload(BspTimerPWMObj_TypeDef *obj, uint32_t auto_reload)
{
    __HAL_TIM_SET_AUTORELOAD(&obj->tim_hdl, auto_reload);
}

static void BspTimer_DMA_Callback(DMA_HandleTypeDef *hdma)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    if (hdma == htim->hdma[TIM_DMA_ID_CC1])
    {
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
    }
    else if (hdma == htim->hdma[TIM_DMA_ID_CC2])
    {
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
    }
    else if (hdma == htim->hdma[TIM_DMA_ID_CC3])
    {
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
    }
    else if (hdma == htim->hdma[TIM_DMA_ID_CC4])
    {
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
    }
}

static void BspTimer_Set_DMA_Callback(BspTimerPWMObj_TypeDef *obj)
{
    obj->tim_hdl.hdma[obj->tim_dma_id_cc]->XferCpltCallback = BspTimer_DMA_Callback;
}