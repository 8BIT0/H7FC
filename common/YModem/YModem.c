#include "YModem.h"
#include "util.h"

#define YMODEM_MIN_SIZE 128
#define YMODEM_MAX_SIZE 1024

typedef struct
{
    uint8_t header;
    uint8_t pack_id;
    uint8_t reverse_id;
    uint8_t *p_data;
    uint16_t crc;
} YModem_Frame_TypeDef;

typedef enum
{
    SOH    = 0x01,  /* 128  data size pack */
    STX    = 0x02,  /* 1024 data size pack */
    EOT    = 0x04,  /* end of translate */
    ACK    = 0x06,  /* send acknowledge */
    NAK    = 0x15,  /* send none acknowledge */
    CAN    = 0x18,  /* cancel translation */
    C      = 0x43,  /* request data pack */
    ABORT1 = 0x41,
    ABORT2 = 0x61,
} YModem_CMD_List;

/* external function */
static void YModem_Set_Callback(YModemObj_TypeDef *obj, uint8_t type, void *callback);
static void YModem_State_Polling(YModemObj_TypeDef *obj, uint8_t *p_bug, uint16_t *size);

YModem_TypeDef YModem = {
    .set_callback = YModem_Set_Callback,
    .polling = YModem_State_Polling,
};

static void YModem_Set_Callback(YModemObj_TypeDef *obj, uint8_t type, void *callback)
{
    if (obj == NULL)
        return;
    
    switch (type)
    {
        case YModem_Callback_Type_Start:  obj->start_callback  = callback; break;
        case YModem_Callback_Type_Finish: obj->finish_callback = callback; break;
        case YModem_Callback_Type_Send:   obj->send_callback   = callback; break;
        case YModem_Callback_Type_Abort:  obj->abort_callback  = callback; break;
        default: break;
    }
}

static YModem_Stream_TypeDef YModem_Decode(YModemObj_TypeDef *obj, uint8_t *p_buf, uint16_t size)
{
    YModem_Stream_TypeDef stream_out;
    uint16_t pack_size = 0;
    uint8_t *frame = NULL;

    memset(&stream_out, 0, sizeof(YModem_Stream_TypeDef));
    if (obj && p_buf && size)
    {
        for (uint16_t i = 0; i < size; i ++)
        {
            if (p_buf[i] == SOH)
            {
                pack_size = 128;
            }
            else if (p_buf[i] == STX)
            {
                pack_size == 1024;
            }
            else
            {
                pack_size = 0;
                i ++;
            }

            if (pack_size)
            {
                frame = &p_buf[i + 1];
                if (size >= pack_size)
                {

                }
            }
        }
    }

    return stream_out;
}

static void YModem_State_Polling(YModemObj_TypeDef *obj, uint8_t *p_buf, uint16_t *size)
{
    uint8_t tx_data = 0;
    bool check_rec = true;

    /* polling currently processing ymodem object */
    if (obj && p_buf && size)
    {
        switch ((uint8_t)(obj->state))
        {
            case YModem_State_Idle:
                obj->state = YModem_State_Tx;
                break;

            case YModem_State_Rx:
                /* update tx stage */
                switch ((uint8_t) obj->tx_stage)
                {
                    case YModem_Req:
                        obj->tx_stage = YModem_ACK;
                        check_rec = false;
                        break;

                    case YModem_ACK:
                        break;

                    default:
                        break;
                }

                if (check_rec && *size)
                {
                    
                }
                break;

            case YModem_State_Tx:
                switch ((uint8_t) obj->tx_stage)
                {
                    case YModem_Req:
                        /* send 'C' */
                        tx_data = C;
                        /* after req data send accomplished check received data */
                        obj->state = YModem_State_Rx;
                        break;

                    case YModem_ACK:
                        tx_data = ACK;
                        obj->state = YModem_State_Rx;
                        break;

                    case YModem_NAK:
                        tx_data = NAK;
                        break;

                    default:
                        tx_data = 0;
                        break;
                }

                if (obj->send_callback && tx_data)
                    obj->send_callback(&tx_data, 1);
                break;

            case YModem_State_Rx_PackDone:
                break;

            case YModem_NotFull_Pack:
                break;

            case YModem_State_TimeOut:
                break;

            default:
                break;
        }
    }
}
