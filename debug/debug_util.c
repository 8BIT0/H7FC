#include "debug_util.h"

static bool DebugPin_Init(DebugPinObj_TypeDef pin);
static bool DebugPin_Ctl(DebugPinObj_TypeDef pin, bool state);

DebugPin_TypeDef DebugPin = {
    .init = DebugPin_Init,
    .ctl = DebugPin_Ctl,
};

static bool DebugPin_Init(DebugPinObj_TypeDef debug_pin)
{
    if (BspGPIO.out_init == NULL)
        return false;

    return BspGPIO.out_init(debug_pin);
}

static bool DebugPin_Ctl(DebugPinObj_TypeDef debug_pin, bool state)
{
    if (BspGPIO.write == NULL)
        return false;

    BspGPIO.write(debug_pin, state);
    return true;
}

void assert(bool state)
{
    if (state)
        while (true)
            ;
}
