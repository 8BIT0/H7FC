#include "IO_Definition.h"
#include "Dev_Led.h"

DevLedObj_TypeDef Led1 = {
    .port = LED1_PORT,
    .pin = LED1_PIN,
    .init_state = true,
};

DevLedObj_TypeDef Led2 = {
    .port = LED2_PORT,
    .pin = LED2_PIN,
    .init_state = true,
};

DevLedObj_TypeDef Led3 = {
    .port = LED3_PORT,
    .pin = LED3_PIN,
    .init_state = true,
};
