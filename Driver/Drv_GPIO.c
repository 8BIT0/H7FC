#include "Drv_GPIO.h"

static bool DrvGPIO_ExtiMode_Init(DrvGPIOObj_TypeDef Obj)
{
}

static bool DrvGPIO_InMOde_Init(DrvGPIOObj_TypeDef Obj)
{
}

static bool DrvGPIO_GenMode_Init(DrvGPIOObj_TypeDef Obj)
{
    DrvGPIOObj_TypeDef Target_Pin;

    memset(&Target_Pin, NULL, sizeof(DrvGPIOObj_TypeDef));

    Target_Pin.pin = Obj.pin;
    Target_Pin.port = Obj.port;
    Target_Pin.init_state = Obj.level;

    Target_Pin.cfg_structure.Pin = Obj.pin;
    Target_Pin.cfg_structure.Mode = GPIO_MODE_OUTPUT_PP;
    Target_Pin.cfg_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    Target_Pin.cfg_structure.Pull = GPIO_PULLUP;

    return BspGPIO.init(Target_Pin);
}

static void DrvGPIO_Set(DrvGPIOObj_TypeDef Obj, bool state)
{
    if (BspGPIO.write == NULL)
        return false;

    BspGPIO.write(Obj.port, Obj.pin, state);
    return true;
}

static bool DrvGPIO_Get(DrvGPIOObj_TypeDef Obj)
{
    if (BspGPIO.read == NULL)
        return false;

    return BspGPIO.read(bj.port, Obj.pin);
}

static bool DrvGPIO_CTL(DrvGPIO_CMD_List cmd, DrvGPIOObj_TypeDef Obj, void *arg)
{
    switch ((uint8_t)cmd)
    {
    case DrvGPIO_OutMode_Open:
        if (size != sizeof(DrvGPIOObj_TypeDef))
            return flase;

        UNUSED(arg);
        return DrvGPIO_GenMode_Init(*Obj);

    default:
        return false;
    }

    return true;
}
