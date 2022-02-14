#include "debug_util.h"
#include "system_cfg.h"

static bool DebugPin_Init(DebugPinObj_TypeDef pin);
static bool DebugPin_Ctl(DebugPinObj_TypeDef pin, bool state);

DebugPin_TypeDef DebugPin = {
    .init = DebugPin_Init,
    .ctl = DebugPin_Ctl,
};

static bool DebugPin_Init(DebugPinObj_TypeDef debug_pin)
{
    BspGPIO_Obj_TypeDef debug_pin_cfg;

    if (BspGPIO.init == NULL)
        return false;

    debug_pin_cfg.pin = debug_pin.pin;
    debug_pin_cfg.port = debug_pin.port;
    debug_pin_cfg.init_state = debug_pin.default_state;
    debug_pin_cfg.cfg_structure.Pin = debug_pin.pin;
    debug_pin_cfg.cfg_structure.Mode = GPIO_MODE_OUTPUT_PP;
    debug_pin_cfg.cfg_structure.Pull = GPIO_NOPULL;
    debug_pin_cfg.cfg_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    BspGPIO.init(debug_pin_cfg);
}

static bool DebugPin_Ctl(DebugPinObj_TypeDef debug_pin, bool state)
{
    if (BspGPIO.write == NULL)
        return false;

    BspGPIO.write(debug_pin.port, debug_pin.pin, state);
    return true;
}

void assert(bool state)
{
    if (state)
        while (true)
            ;
}
