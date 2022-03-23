#include "Bsp_GPIO.h"
#include "system_cfg.h"

static bool BspGPIO_Init(BspGPIO_Obj_TypeDef IO_Obj);
static bool BspGPIO_Read(uint32_t port, uint16_t pin);
static void BspGPIO_Write(uint32_t port, uint16_t pin, bool state);

BspGPIO_TypeDef BspGPIO = {
    .init = BspGPIO_Init,
    .read = BspGPIO_Read,
    .write = BspGPIO_Write,
};

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

static bool BspGPIO_ExtiInit(BspGPIO_Obj_TypeDef IO_Obj)
{
}

static bool BspGPIO_Init(BspGPIO_Obj_TypeDef IO_Obj)
{
    if (IO_Obj.port == NULL)
        return false;

    BspGPIO_CLK_Enable(IO_Obj.port);

    HAL_GPIO_WritePin(IO_Obj.port, IO_Obj.pin, IO_Obj.init_state);
    HAL_GPIO_Init(IO_Obj.port, &(IO_Obj.cfg_structure));

    return true;
}

static bool BspGPIO_Read(uint32_t port, uint16_t pin)
{
    return HAL_GPIO_ReadPin(port, pin);
}

static void BspGPIO_Write(uint32_t port, uint16_t pin, bool state)
{
    HAL_GPIO_WritePin(port, pin, state);
}
