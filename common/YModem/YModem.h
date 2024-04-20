#ifndef __YMODEM_H
#define __YMODEM_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define YMODEM_MAX_PROTO_SIZE 1024

#define To_YModem_Obj(x) ((YModemObj_TypeDef *)x)
#define To_YModem_Api(x) ((YModem_TypeDef *)x)
#define YModemObj_size sizeof(YModemObj_TypeDef)

typedef uint32_t (*YModem_Get_SysTick)(void);
typedef void (*YModem_Start_Callback)(void *YModem_Obj);
typedef void (*YModem_Finish_Callback)(void *YModem_Obj);
typedef void (*YModem_Error_Callback)(void *YModem_Obj, uint8_t error_code);
typedef void (*YModem_Send_Callback)(uint8_t *p_buf, uint16_t len);

typedef enum
{
    YModem_State_Idle = 0,
    YModem_State_Tx,
    YModem_State_Rx,
    YModem_State_Rx_Waiting,
    YModem_State_Rx_PackDone,
    YModem_State_Rx_Failed,
    YModem_State_Tx_Failed,
    YModem_NotFull_Pack,
    YModem_State_TimeOut,
} YModem_State_List;

typedef struct
{
    uint32_t total_size;
    uint32_t cur_size;
    uint8_t *p_buf;
} YModem_Stream_TypeDef;

typedef union
{
    uint16_t val;

    struct
    {
        uint8_t recv : 1;   /* is receiving */
        uint8_t pack : 1;   /* is packing */
        uint8_t poll : 1;   /* in polling */
        uint8_t send : 1;   /* is sending */
        uint8_t res  : 4;   /* reserve */
    } bit;
} YModem_StateReg_TypeDef;

typedef enum
{
    YModem_None_Frame = 0,
    YModem_HandShake,
    YModem_Start_Frame,
    YModem_ACK,
    YModem_Data_Frame,
    YModem_NAK,
    YModem_EOT,
    YModem_End_Frame,
} YModem_Stage_List;

typedef struct
{
    YModem_State_List state;
    YModem_Stage_List stage;
    uint32_t timeout_ms;

    YModem_Get_SysTick sys_tick;
    YModem_Start_Callback start_callback;
    YModem_Finish_Callback finish_callback;
    YModem_Error_Callback error_callback;
    YModem_Send_Callback send_callback;

    YModem_Stream_TypeDef tx_stream;
    YModem_Stream_TypeDef rx_stream;

    YModem_StateReg_TypeDef state_reg;
    uint8_t cur_pack_id;
    uint8_t next_pack_id;
    uint16_t received_pack_num;
    uint32_t received_byte_num;
    uint16_t remain_byte;
} YModemObj_TypeDef;

typedef struct
{
    void (*recv)(YModemObj_TypeDef *obj, uint8_t *p_buf, uint16_t len);
    void (*pack)(YModemObj_TypeDef *obj, uint8_t *p_buf, uint16_t len);
    void (*polling)(YModemObj_TypeDef *obj);
} YModem_TypeDef;

extern YModem_TypeDef YModem;

#endif
