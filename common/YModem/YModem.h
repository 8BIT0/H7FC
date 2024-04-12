#ifndef __YMODEM_H
#define __YMODEM_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef enum
{
    YModem_State_Idle = 0,
} YModem_State_List;

typedef struct
{
    YModem_State_List state;
} YModemObj_TypeDef;

typedef struct
{
    bool (*Recv)(YModemObj_TypeDef *obj, uint8_t *p_data, uint16_t len);
} YModem_TypeDef;

extern YModem_TypeDef YModem;

#endif
