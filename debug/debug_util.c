#include "debug_util.h"
#define MAX_PRINT_SIZE 512

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

static void Debug_SendCallback(uint32_t cust_data_addr, uint8_t *buff, uint16_t size)
{
    DebugPrintObj_TypeDef *DebugPortObj = NULL;

    if (cust_data_addr)
    {
        DebugPortObj = ((DebugPrintObj_TypeDef *)cust_data_addr);
        DebugPortObj->tx_fin_cnt ++;
    }
}

static void Debug_PrintOut(DebugPrintObj_TypeDef *Obj, uint8_t *p_data, uint16_t len)
{
    if (p_data && len)
    {
        Obj->tx_cnt ++;
        BspUart.send(To_BspUart_Obj(Obj->port_obj), p_data, len);
        while (Obj->tx_cnt != Obj->tx_fin_cnt);
    }
}

void Debug_Port_Init(DebugPrintObj_TypeDef *Obj)
{
    if (!Obj->port_obj || !BspUart.init(To_BspUart_Obj(Obj->port_obj)))
    {
        Obj->port_obj = NULL;
        return;
    }

    Obj->tx_cnt = 0;
    Obj->tx_fin_cnt = 0;
    Obj->init = true;
    To_BspUart_Obj(Obj->port_obj)->cust_data_addr = ((uint32_t)Obj);
    BspUart.set_tx_callback(To_BspUart_Obj(Obj->port_obj), Debug_SendCallback);
}

void Debug_Print(DebugPrintObj_TypeDef *Obj, const char* fmt, ...)
{
	va_list ap;
    uint16_t length = 0;

    if (!Obj->init)
        return;

    if (Obj->free && Obj->malloc && (Obj->p_buf == NULL))
    {
        Obj->p_buf = Obj->malloc(MAX_PRINT_SIZE);
        if (Obj->p_buf == NULL)
            return;
    }

    memset(Obj->p_buf, 0, MAX_PRINT_SIZE);

    if (Obj->p_buf && Obj->port_obj)
    {
	    va_start(ap, fmt);
	    length = vsnprintf(Obj->p_buf, MAX_PRINT_SIZE, fmt, ap);
	    va_end(ap);

        if (length > MAX_PRINT_SIZE)
            length = MAX_PRINT_SIZE;

        Debug_PrintOut(Obj, Obj->p_buf, length);
        memset(Obj->p_buf, 0, length);
    }
}
