#ifndef __YMODEM_H
#define __YMODEM_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define YMODEM_MAX_PROTO_SIZE 1024

typedef enum
{
    YModem_State_Idle = 0,
    YModem_State_Tx,
    YModem_State_Rx,
    YModem_State_TimeOut,
} YModem_State_List;

typedef uint32_t (*YModem_Get_SysTick)(void);

typedef struct
{
    uint32_t total_size;
    uint32_t cur_size;
    uint8_t *p_buf;
} YModem_Stream_TypeDef;

typedef struct
{
    YModem_State_List state;
    uint32_t timeout_threshold;
    YModem_Get_SysTick sys_tick;
    YModem_Stream_TypeDef tx_stream;
    YModem_Stream_TypeDef rx_stream;
} YModemObj_TypeDef;

typedef struct
{
    void (*recv)(YModemObj_TypeDef *obj, uint8_t *p_buf, uint16_t len);
} YModem_TypeDef;

extern YModem_TypeDef YModem;

#endif
