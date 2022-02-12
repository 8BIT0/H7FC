#include "Dev_Led.h"
#include "Bsp_GPIO.h"

DevLed_TypeDef DevLED;

static bool DevLed_Init(DevLedObj_TypeDef LedObj);
static bool DevLed_Ctl(DevLedObj_TypeDef LedObj, bool state);

DevLed_TypeDef DevLED = {
    .init = DevLed_Init,
    .ctl = DevLed_Ctl,
};

static bool DevLed_Init(DevLedObj_TypeDef LedObj)
{
    BspGPIO_Obj_TypeDef LedIO_init_Cfg;

    if (BspGPIO.init == NULL)
        return false;

    LedIO_init_Cfg.pin = LedObj.pin;
    LedIO_init_Cfg.port = LedObj.port;
    LedIO_init_Cfg.init_state = LedObj.default_state;
    LedIO_init_Cfg.cfg_structure.Pin = LedObj.pin;
    LedIO_init_Cfg.cfg_structure.Mode = GPIO_MODE_OUTPUT_PP;
    LedIO_init_Cfg.cfg_structure.Pull = GPIO_NOPULL;
    LedIO_init_Cfg.cfg_structure.Speed = GPIO_SPEED_FREQ_HIGH;

    BspGPIO.init(LedIO_init_Cfg);

    return true;
}

static bool DevLed_Ctl(DevLedObj_TypeDef LedObj, bool state)
{
    if (BspGPIO.write == NULL)
        return false;

    BspGPIO.write(LedObj.port, LedObj.pin, state);
    return true;
}
