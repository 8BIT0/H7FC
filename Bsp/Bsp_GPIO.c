#include "Bsp_GPIO.h"

static bool BspGPIO_Init(BspGPIO_Obj_TypeDef IO_Obj);
static bool BspGPIO_Read(uint32_t port, uint16_t pin);
static void BspGPIO_Write(uint32_t port, uint16_t pin, bool state);

BspGPIO_TypeDef BspGPIO = {
    .init = BspGPIO_Init,
    .read = BspGPIO_Read,
    .write = BspGPIO_Write,
};

static bool BspGPIO_Init(BspGPIO_Obj_TypeDef IO_Obj)
{
    if (IO_Obj.port == NULL)
        return false;

    if (IO_Obj.port == GPIOA)
        __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (IO_Obj.port == GPIOB)
        __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (IO_Obj.port == GPIOC)
        __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (IO_Obj.port == GPIOD)
        __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (IO_Obj.port == GPIOE)
        __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (IO_Obj.port == GPIOF)
        __HAL_RCC_GPIOF_CLK_ENABLE();
    else if (IO_Obj.port == GPIOG)
        __HAL_RCC_GPIOG_CLK_ENABLE();
    else if (IO_Obj.port == GPIOH)
        __HAL_RCC_GPIOH_CLK_ENABLE();
#if defined(GPIOI)
    else if (IO_Obj.port == GPIOI)
        __HAL_RCC_GPIOI_CLK_ENABLE();
#endif
    else if (IO_Obj.port == GPIOJ)
        __HAL_RCC_GPIOJ_CLK_ENABLE();
    else if (IO_Obj.port == GPIOK)
        __HAL_RCC_GPIOK_CLK_ENABLE();

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
