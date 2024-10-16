#include "debug_util.h"

#define MAX_PRINT_SIZE ((uint16_t)512)

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
    while (state);
}

static void Debug_PrintOut(DebugPrintObj_TypeDef *Obj, uint8_t *p_data, uint16_t len)
{
    if (p_data && len)
        BspUart.send(To_BspUart_Obj(Obj->port_obj), p_data, len);
}

void Debug_Port_Init(DebugPrintObj_TypeDef *Obj)
{
#if defined STM32H743xx
    To_BspUart_Obj(Obj->port_obj)->hdl = Obj->malloc(UART_HandleType_Size);
    if (To_BspUart_Obj(Obj->port_obj)->hdl == NULL)
        return;
#endif

    if (!Obj->port_obj || !BspUart.init(To_BspUart_Obj(Obj->port_obj)))
    {
        Obj->port_obj = NULL;
        return;
    }

    Obj->init = true;
    To_BspUart_Obj(Obj->port_obj)->cust_data_addr = ((uint32_t)Obj);
}

void Debug_Print(DebugPrintObj_TypeDef *Obj, const char *tag, const char* fmt, ...)
{
	va_list ap;
    uint16_t length = 0;
    char new_fmt[128];

    if (!Obj->init)
        return;

    if (Obj->free && Obj->malloc && (Obj->p_buf == NULL))
    {
        Obj->p_buf = Obj->malloc(MAX_PRINT_SIZE);
        if (Obj->p_buf == NULL)
            return;
    }

    memset(Obj->p_buf, 0, MAX_PRINT_SIZE);
    memset(new_fmt, 0, sizeof(new_fmt));
    strcpy(new_fmt, tag);
    strcat(new_fmt, fmt);

    if (Obj->p_buf && Obj->port_obj)
    {
	    va_start(ap, new_fmt);
	    length = vsnprintf(Obj->p_buf, MAX_PRINT_SIZE, new_fmt, ap);
	    va_end(ap);

        if (length > MAX_PRINT_SIZE)
            length = MAX_PRINT_SIZE;

        Debug_PrintOut(Obj, Obj->p_buf, length);
        memset(Obj->p_buf, 0, length);
    }
}
