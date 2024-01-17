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
    if (BspGPIO.out_init == NULL)
        return false;

    BspGPIO.out_init(LedObj);

    return true;
}

static bool DevLed_Ctl(DevLedObj_TypeDef LedObj, bool state)
{
    if (BspGPIO.write == NULL)
        return false;

    BspGPIO.write(LedObj, state);
    return true;
}
