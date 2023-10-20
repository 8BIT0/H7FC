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

static BspTimerTick_TypeDef *BspTimer_TickObj_List[BspTimer_TickObj_Sum] = {NULL};

/* internal function */
static void BspTimer_DMA_Callback(DMA_HandleTypeDef *hdma);
static bool BspTimer_Clk_Enable(TIM_TypeDef *tim);

/* external function */
static bool BspTimer_PWM_Init(BspTimerPWMObj_TypeDef *obj,
                              TIM_TypeDef *instance,
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

/***************************************************************** General Function ***********************************************************************/
TIM_HandleTypeDef* BspTimer_Get_Tick_HandlePtr(BspTimer_Instance_List index)
{
    switch(index)
    {
        case BspTimer_1:
            return BspTimer_TickObj_List[BspTimer_1];

        case BspTimer_2:
            return BspTimer_TickObj_List[BspTimer_2];
        
        case BspTimer_3:
            return BspTimer_TickObj_List[BspTimer_3];
        
        case BspTimer_4:
            return BspTimer_TickObj_List[BspTimer_4];
        
        case BspTimer_5:
            return BspTimer_TickObj_List[BspTimer_5];
        
        case BspTimer_6:
            return BspTimer_TickObj_List[BspTimer_6];

        case BspTimer_7:
            return BspTimer_TickObj_List[BspTimer_7];

        case BspTimer_8:
            return BspTimer_TickObj_List[BspTimer_8];
        
        default:
            return NULL;
    }
}

static void BspTimer_Fill_TickObj_ToList(BspTimerTickObj_TypeDef *obj)
{
    if(obj)
    {
        if (obj->instance == TIM1)
        {
            BspTimer_TickObj_List[BspTimer_1] = obj;
        }
        else if (obj->instance == TIM2)
        {
            BspTimer_TickObj_List[BspTimer_2] = obj;
        }
        else if (obj->instance == TIM3)
        {
            BspTimer_TickObj_List[BspTimer_3] = obj;
        }
        else if (obj->instance == TIM4)
        {
            BspTimer_TickObj_List[BspTimer_4] = obj;
        }
        else if (obj->instance == TIM5)
        {
            BspTimer_TickObj_List[BspTimer_5] = obj;
        }
        else if (obj->instance == TIM6)
        {
            BspTimer_TickObj_List[BspTimer_6] = obj;
        }
        else if (obj->instance == TIM7)
        {
            BspTimer_TickObj_List[BspTimer_7] = obj;
        }
        else if (obj->instance == TIM8)
        {
            BspTimer_TickObj_List[BspTimer_8] = obj;
        }
    }
}

static bool BspTimer_Clk_Enable(TIM_TypeDef *tim)
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
    else if (tim == TIM7)
    {
        __HAL_RCC_TIM7_CLK_ENABLE();
    }
    else if (tim == TIM8)
    {
        __HAL_RCC_TIM8_CLK_ENABLE();
    }
    else
        return false;

    return true;
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

/***************************************************************** DMA PWM Function ***********************************************************************/
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
        if(!BspTimer_Clk_Enable(tim))
            return false;

        monitor.list[monitor.init_cnt] = tim;
        monitor.init_cnt++;
    }

    return true;
}

static uint32_t BspTimer_Get_DMA_Request(TIM_TypeDef *instance, uint32_t ch)
{
    switch ((uint32_t)instance)
    {
    case (uint32_t)TIM5:
        switch (ch)
        {
        case TIM_CHANNEL_1:
            return DMA_REQUEST_TIM5_CH1;
        case TIM_CHANNEL_2:
            return DMA_REQUEST_TIM5_CH2;
        case TIM_CHANNEL_3:
            return DMA_REQUEST_TIM5_CH3;
        case TIM_CHANNEL_4:
            return DMA_REQUEST_TIM5_CH4;
        default:
            return 0;
        }
        break;

    case (uint32_t)TIM3:
        switch (ch)
        {
        case TIM_CHANNEL_1:
            return DMA_REQUEST_TIM3_CH1;
        case TIM_CHANNEL_2:
            return DMA_REQUEST_TIM3_CH2;
        case TIM_CHANNEL_3:
            return DMA_REQUEST_TIM3_CH3;
        case TIM_CHANNEL_4:
            return DMA_REQUEST_TIM3_CH4;
        default:
            return 0;
        }
        break;

    case (uint32_t)TIM4:
        switch (ch)
        {
        case TIM_CHANNEL_1:
            return DMA_REQUEST_TIM4_CH1;
        case TIM_CHANNEL_2:
            return DMA_REQUEST_TIM4_CH2;
        case TIM_CHANNEL_3:
            return DMA_REQUEST_TIM4_CH3;
        default:
            return 0;
        }
        break;

    case (uint32_t)TIM15:
        switch (ch)
        {
        case TIM_CHANNEL_1:
            return DMA_REQUEST_TIM15_CH1;
        case TIM_CHANNEL_2:
            return DMA_REQUEST_TIM15_UP;
        case TIM_CHANNEL_3:
            return DMA_REQUEST_TIM15_TRIG;
        case TIM_CHANNEL_4:
            return DMA_REQUEST_TIM15_COM;
        default:
            return 0;
        }
        break;

    default:
        return 0;
    }
}

static bool BspTimer_PWM_Init(BspTimerPWMObj_TypeDef *obj,
                              TIM_TypeDef *instance,
                              uint32_t ch,
                              BspGPIO_Obj_TypeDef pin,
                              uint8_t dma, uint8_t stream, uint32_t buf_aadr, uint32_t buf_size)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    if ((obj == NULL) || (instance == NULL) || (buf_aadr == 0) || (buf_size == 0))
        return false;

    obj->dma = dma;
    obj->stream = stream;

    switch (ch)
    {
    case TIM_CHANNEL_1:
        obj->tim_dma_id_cc = TIM_DMA_ID_CC1;
        obj->tim_dma_cc = TIM_DMA_CC1;
        break;

    case TIM_CHANNEL_2:
        obj->tim_dma_id_cc = TIM_DMA_ID_CC2;
        obj->tim_dma_cc = TIM_DMA_CC2;
        break;

    case TIM_CHANNEL_3:
        obj->tim_dma_id_cc = TIM_DMA_ID_CC3;
        obj->tim_dma_cc = TIM_DMA_CC3;
        break;

    case TIM_CHANNEL_4:
        obj->tim_dma_id_cc = TIM_DMA_ID_CC4;
        obj->tim_dma_cc = TIM_DMA_CC4;
        break;

    default:
        return false;
    }

    BspTimer_PWM_InitMonit(instance);
    obj->instance = instance;
    obj->tim_hdl.Instance = instance;
    obj->tim_hdl.Init.Prescaler = 0;
    obj->tim_hdl.Init.CounterMode = TIM_COUNTERMODE_UP;
    obj->tim_hdl.Init.Period = 0;
    obj->tim_hdl.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    obj->tim_hdl.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    /* dma init */
    obj->dma_hdl.Instance = BspDMA.get_instance(dma, stream);
    obj->dma_hdl.Init.Request = BspTimer_Get_DMA_Request(instance, ch);
    obj->dma_hdl.Init.Direction = DMA_MEMORY_TO_PERIPH;
    obj->dma_hdl.Init.PeriphInc = DMA_PINC_DISABLE;
    obj->dma_hdl.Init.MemInc = DMA_MINC_ENABLE;
    obj->dma_hdl.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    obj->dma_hdl.Init.MemDataAlignment = DMA_PDATAALIGN_WORD;
    obj->dma_hdl.Init.Mode = DMA_NORMAL;
    obj->dma_hdl.Init.Priority = DMA_PRIORITY_HIGH;
    obj->dma_hdl.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    obj->dma_hdl.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    obj->dma_hdl.Init.MemBurst = DMA_MBURST_SINGLE;
    obj->dma_hdl.Init.PeriphBurst = DMA_PBURST_SINGLE;

    if (HAL_DMA_Init(&obj->dma_hdl) != HAL_OK)
        return false;

    __HAL_LINKDMA(&(obj->tim_hdl), hdma[obj->tim_dma_id_cc], obj->dma_hdl);

    if (HAL_TIM_PWM_Init(&(obj->tim_hdl)) != HAL_OK)
        return false;

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&(obj->tim_hdl), &sMasterConfig) != HAL_OK)
        return false;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&(obj->tim_hdl), &sConfigOC, ch) != HAL_OK)
        return false;

    obj->tim_channel = ch;

    /* pin init */
    BspGPIO.alt_init(pin, GPIO_MODE_AF_PP);

    /* dma regist */
    if (BspDMA.regist(dma, stream, &obj->dma_hdl))
    {
        /* init DMA IRQ */
        BspDMA.enable_irq(dma, stream, 5, 0);
        return true;
    }

    return true;
}

static void BspTimer_PWM_Start(BspTimerPWMObj_TypeDef *obj)
{
    obj->tim_hdl.hdma[obj->tim_dma_id_cc]->XferCpltCallback = BspTimer_DMA_Callback;
    HAL_TIM_PWM_Start(&obj->tim_hdl, obj->tim_channel);
}

static void BspTimer_DMA_Start(BspTimerPWMObj_TypeDef *obj)
{
    uint32_t dst_addr = 0;

    if ((obj->buffer_addr == 0) || (obj->buffer_size == 0))
        return;

    switch (obj->tim_dma_id_cc)
    {
    case TIM_DMA_ID_CC1:
        dst_addr = (uint32_t)&obj->tim_hdl.Instance->CCR1;
        break;

    case TIM_DMA_ID_CC2:
        dst_addr = (uint32_t)&obj->tim_hdl.Instance->CCR2;
        break;

    case TIM_DMA_ID_CC3:
        dst_addr = (uint32_t)&obj->tim_hdl.Instance->CCR3;
        break;

    case TIM_DMA_ID_CC4:
        dst_addr = (uint32_t)&obj->tim_hdl.Instance->CCR4;
        break;

    default:
        return;
    }

    HAL_DMA_Start_IT(obj->tim_hdl.hdma[obj->tim_dma_id_cc], obj->buffer_addr, dst_addr, obj->buffer_size);
    __HAL_TIM_ENABLE_DMA(&obj->tim_hdl, obj->tim_dma_cc);
}

static void BspTimer_SetPreScale(BspTimerPWMObj_TypeDef *obj, uint32_t prescale)
{
    obj->prescale = prescale;
    __HAL_TIM_SET_PRESCALER(&obj->tim_hdl, prescale);
}

static void BspTimer_SetAutoReload(BspTimerPWMObj_TypeDef *obj, uint32_t auto_reload)
{
    obj->auto_reload = auto_reload;
    __HAL_TIM_SET_AUTORELOAD(&obj->tim_hdl, auto_reload);
}

/***************************************************************** Tick Function ***********************************************************************/
static bool BspTimer_Tick_Init(BspTimerTickObj_TypeDef *obj, uint32_t perscale, uint32_t period)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    if(obj && obj->instance)
    {
        // BspTimer_Clk_Enable();
        obj->tim_hdl.Instance = obj->instance; // TIM6
        obj->tim_hdl.Init.Prescaler = perscale;
        obj->tim_hdl.Init.CounterMode = TIM_COUNTERMODE_UP;
        obj->tim_hdl.Init.Period = period;
        obj->tim_hdl.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        obj->tim_hdl.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
        if (HAL_TIM_Base_Init(&obj->tim_hdl) != HAL_OK)
            return false;

        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        if (HAL_TIM_ConfigClockSource(&obj->tim_hdl, &sClockSourceConfig) != HAL_OK)
            return false;

        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&obj->tim_hdl, &sMasterConfig) != HAL_OK)
            return false;
    
        BspTimer_Clk_Enable(obj->instance);

        /* TIM interrupt Init */
        // HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
        // HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }

    return true;
}

/* Enable Timer IRQ */
static bool BspTimer_Tick_Start(BspTimerTickObj_TypeDef *obj)
{
    if(obj)
    {

    }

    return false;
}

/* DIsable Timer IRQ */
static bool BspTImer_Tick_Stop(BspTimerTickObj_TypeDef *obj)
{
    if(obj)
    {

    }

    return false;
}

