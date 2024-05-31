#ifndef __YMODEM_H
#define __YMODEM_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define YMODEM_MAX_PROTO_SIZE 1024

#define To_YModem_Obj(x) ((YModemObj_TypeDef *)x)
#define To_YModem_Api(x) ((YModem_TypeDef *)x)
#define YModemObj_size sizeof(YModemObj_TypeDef)
#define To_YModem_Stream(x) ((YModem_Stream_TypeDef *)x)
#define YModem_Stream_Size sizeof(YModem_Stream_TypeDef)

typedef void (*YModem_Send_Callback)(uint8_t *p_buf, uint16_t len);

typedef enum
{
    YModem_State_Idle = 0,
    YModem_State_Tx,
    YModem_State_Rx,
    YModem_State_Finish,
    YModem_State_Error,
    YModem_State_Unknow,
} YModem_State_List;

typedef enum
{
    YModem_Callback_Type_None = 0,
    YModem_Callback_Type_Start,
    YModem_Callback_Type_Finish,
    YModem_Callback_Type_Send,
    YModem_Callback_Type_Abort,
} YModem_CallbackType_List;

typedef enum
{
    YModem_Pack_Invalid = 0,
    YModem_Pack_InCompelete,
    YModem_Pack_Compelete,
    YModem_Pack_Default,
} YModem_PackState_List;

typedef struct
{
    uint32_t size;
    uint8_t *p_buf;
    YModem_PackState_List valid;
    bool file_data;
} YModem_Stream_TypeDef;

typedef enum
{
    YModem_Req = 0, /* request file */
    YModem_Cfm,     /* comfirm file */
    YModem_ACK,
    YModem_NAK,
    YModem_Lst,     /* req last pack */
    YModem_EOT,
} YModem_TxStage_List;

typedef struct
{
    YModem_State_List state;
    YModem_TxStage_List tx_stage;
    uint32_t timeout_ms;
    uint32_t re_send_time;
    bool data_income;

    YModem_Send_Callback send_callback;

    uint8_t EOT_Cnt;
    bool wait_last_pack;

    uint8_t cur_pack_id;
    uint8_t next_pack_id;
    uint16_t received_pack_num;
    uint32_t received_byte_num;
    uint16_t remain_byte;
} YModemObj_TypeDef;

typedef struct
{
    uint8_t (*polling)(uint32_t sys_time, YModemObj_TypeDef *obj, uint8_t *p_bug, uint16_t size, YModem_Stream_TypeDef *p_stream);
    void (*abort)(YModemObj_TypeDef *obj);
} YModem_TypeDef;

extern YModem_TypeDef YModem;

#endif
