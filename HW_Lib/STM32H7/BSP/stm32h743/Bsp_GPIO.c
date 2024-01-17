#include "Bsp_GPIO.h"
#include "debug_util.h"

/* interbal function */
static EXTI_Callback EXTI_CallBack_List[GPIO_EXTI_SUM] = {NULL};

/* internal function */
static void BspGPIO_CLK_Enable(GPIO_TypeDef *port);
static uint8_t BspGPIO_GetEXTI_Index(uint16_t exti_id);
static IRQn_Type BspGPIO_GetExti_IRQnID(BspGPIO_Obj_TypeDef IO_Obj);

/* external function */
static bool BspGPIO_Output_Init(BspGPIO_Obj_TypeDef IO_Obj);
static bool BspGPIO_Input_Init(BspGPIO_Obj_TypeDef IO_Obj);
static bool BspGPIO_Alternate_Init(BspGPIO_Obj_TypeDef IO_Obj, uint32_t af_mode);
static bool BspGPIO_Read(BspGPIO_Obj_TypeDef IO_Obj);
static void BspGPIO_Write(BspGPIO_Obj_TypeDef IO_Obj, bool state);
static bool BspGPIO_ExtiInit(BspGPIO_Obj_TypeDef IO_Obj, EXTI_Callback callback);
static bool BspGPIO_ResetExtiCallback(BspGPIO_Obj_TypeDef IO_Obj, EXTI_Callback callback);
static bool BspGPIO_ExtiSetMode(BspGPIO_Obj_TypeDef IO_Obj, BspGPOP_ExtiMode_List mode);

BspGPIO_TypeDef BspGPIO = {
    .exti_init = BspGPIO_ExtiInit,
    .in_init = BspGPIO_Input_Init,
    .out_init = BspGPIO_Output_Init,
    .alt_init = BspGPIO_Alternate_Init,
    .read = BspGPIO_Read,
    .write = BspGPIO_Write,
    .set_exti_callback = BspGPIO_ResetExtiCallback,
    .set_exti_mode = BspGPIO_ExtiSetMode,
};

static uint8_t BspGPIO_GetEXTI_Index(uint16_t exti_id)
{
    float f_exti_id = (float)exti_id;
    uint32_t u32_exti_id = 0;
    uint8_t exp = 0;

    u32_exti_id = *((unsigned long *)(&f_exti_id));

    exp = (u32_exti_id >> 23) & 0xFF;
    exp -= 127;

    return exp;
}

static void BspGPIO_CLK_Enable(GPIO_TypeDef *port)
{
    if (port == GPIOA)
        __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB)
        __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC)
        __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD)
        __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (port == GPIOE)
        __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (port == GPIOF)
        __HAL_RCC_GPIOF_CLK_ENABLE();
    else if (port == GPIOG)
        __HAL_RCC_GPIOG_CLK_ENABLE();
    else if (port == GPIOH)
        __HAL_RCC_GPIOH_CLK_ENABLE();
#if defined(GPIOI)
    else if (port == GPIOI)
        __HAL_RCC_GPIOI_CLK_ENABLE();
#endif
    else if (port == GPIOJ)
        __HAL_RCC_GPIOJ_CLK_ENABLE();
    else if (port == GPIOK)
        __HAL_RCC_GPIOK_CLK_ENABLE();
}

static IRQn_Type BspGPIO_GetExti_IRQnID(BspGPIO_Obj_TypeDef IO_Obj)
{
    switch ((uint16_t)IO_Obj.pin)
    {
    case GPIO_PIN_0:
        return EXTI0_IRQn;

    case GPIO_PIN_1:
        return EXTI1_IRQn;

    case GPIO_PIN_2:
        return EXTI2_IRQn;

    case GPIO_PIN_3:
        return EXTI3_IRQn;

    case GPIO_PIN_4:
        return EXTI4_IRQn;

    case GPIO_PIN_5:
    case GPIO_PIN_6:
    case GPIO_PIN_7:
    case GPIO_PIN_8:
    case GPIO_PIN_9:
        return EXTI9_5_IRQn;

    case GPIO_PIN_10:
    case GPIO_PIN_11:
    case GPIO_PIN_12:
    case GPIO_PIN_13:
    case GPIO_PIN_14:
    case GPIO_PIN_15:
        return EXTI15_10_IRQn;

    default:
        assert(true);
        break;
    }
}

static bool BspGPIO_ExtiSetMode(BspGPIO_Obj_TypeDef IO_Obj, BspGPOP_ExtiMode_List mode)
{
    uint32_t mode_val;
    GPIO_InitTypeDef cfg_structure;

    if (IO_Obj.port == NULL)
        return false;

    HAL_NVIC_DisableIRQ(BspGPIO_GetExti_IRQnID(IO_Obj));

    switch ((uint8_t)mode)
    {
    case GPIO_Exti_Rasing:
        mode_val = GPIO_MODE_IT_RISING;
        break;

    case GPIO_Exti_Falling:
        mode_val = GPIO_MODE_IT_FALLING;
        break;

    case GPIO_Exti_TwoEdge:
        mode_val = GPIO_MODE_IT_RISING_FALLING;
        break;

    default:
        mode_val = GPIO_MODE_IT_FALLING;
        break;
    }

    BspGPIO_CLK_Enable(IO_Obj.port);

    cfg_structure.Pin = IO_Obj.pin;
    cfg_structure.Mode = mode_val;
    cfg_structure.Pull = GPIO_NOPULL;
    cfg_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    HAL_GPIO_WritePin(IO_Obj.port, IO_Obj.pin, IO_Obj.init_state);
    HAL_GPIO_Init(IO_Obj.port, &cfg_structure);

    HAL_NVIC_SetPriority(BspGPIO_GetExti_IRQnID(IO_Obj), 5, 0);
    HAL_NVIC_EnableIRQ(BspGPIO_GetExti_IRQnID(IO_Obj));
}

static bool BspGPIO_ResetExtiCallback(BspGPIO_Obj_TypeDef IO_Obj, EXTI_Callback callback)
{
    /* set exti callback */
    EXTI_CallBack_List[BspGPIO_GetEXTI_Index(IO_Obj.pin)] = callback;
}

static bool BspGPIO_Alternate_Init(BspGPIO_Obj_TypeDef IO_Obj, uint32_t af_mode)
{
    GPIO_InitTypeDef cfg_structure;

    BspGPIO_CLK_Enable(IO_Obj.port);

    cfg_structure.Pin = IO_Obj.pin;

    if((af_mode != GPIO_MODE_AF_PP) && (af_mode != GPIO_MODE_AF_OD))
        return false;

    // cfg_structure.Mode = GPIO_MODE_AF_PP;
    cfg_structure.Mode = af_mode;

    cfg_structure.Pull = GPIO_NOPULL;
    cfg_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    cfg_structure.Alternate = IO_Obj.alternate;

    HAL_GPIO_Init(IO_Obj.port, &cfg_structure);

    return true;
}

static bool BspGPIO_Input_Init(BspGPIO_Obj_TypeDef IO_Obj)
{
    GPIO_InitTypeDef cfg_structure;

    BspGPIO_CLK_Enable(IO_Obj.port);

    cfg_structure.Pin = IO_Obj.pin;
    cfg_structure.Mode = GPIO_MODE_INPUT;
    cfg_structure.Pull = GPIO_PULLDOWN;
    cfg_structure.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(IO_Obj.port, &cfg_structure);

    return true;
}

static bool BspGPIO_ExtiInit(BspGPIO_Obj_TypeDef IO_Obj, EXTI_Callback callback)
{
    GPIO_InitTypeDef cfg_structure;

    if (IO_Obj.port == NULL)
        return false;

    BspGPIO_CLK_Enable(IO_Obj.port);

    cfg_structure.Pin = IO_Obj.pin;
    cfg_structure.Mode = GPIO_MODE_IT_FALLING;
    cfg_structure.Pull = GPIO_NOPULL;
    cfg_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    HAL_GPIO_WritePin(IO_Obj.port, IO_Obj.pin, IO_Obj.init_state);
    HAL_GPIO_Init(IO_Obj.port, &cfg_structure);

    /* set exti callback */
    EXTI_CallBack_List[BspGPIO_GetEXTI_Index(IO_Obj.pin)] = callback;

    HAL_NVIC_SetPriority(BspGPIO_GetExti_IRQnID(IO_Obj), 5, 0);
    HAL_NVIC_EnableIRQ(BspGPIO_GetExti_IRQnID(IO_Obj));

    return true;
}

static bool BspGPIO_Output_Init(BspGPIO_Obj_TypeDef IO_Obj)
{
    GPIO_InitTypeDef cfg_structure;

    if (IO_Obj.port == NULL)
        return false;

    BspGPIO_CLK_Enable(IO_Obj.port);

    cfg_structure.Pin = IO_Obj.pin;
    cfg_structure.Mode = GPIO_MODE_OUTPUT_PP;
    cfg_structure.Pull = GPIO_PULLUP;
    cfg_structure.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_WritePin(IO_Obj.port, IO_Obj.pin, IO_Obj.init_state);
    HAL_GPIO_Init(IO_Obj.port, &cfg_structure);

    return true;
}

static bool BspGPIO_Read(BspGPIO_Obj_TypeDef IO_Obj)
{
    return HAL_GPIO_ReadPin(IO_Obj.port, IO_Obj.pin);
}

static void BspGPIO_Write(BspGPIO_Obj_TypeDef IO_Obj, bool state)
{
    HAL_GPIO_WritePin(IO_Obj.port, IO_Obj.pin, state);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if ((BspGPIO_GetEXTI_Index(GPIO_Pin) >= 0) && (EXTI_CallBack_List[BspGPIO_GetEXTI_Index(GPIO_Pin)] != NULL))
        EXTI_CallBack_List[BspGPIO_GetEXTI_Index(GPIO_Pin)]();
}
