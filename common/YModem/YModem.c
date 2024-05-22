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

/* internal function */
static YModem_Stream_TypeDef YModem_Decode(YModemObj_TypeDef *obj, uint8_t *p_buf, uint16_t size);

/* external function */
static void YModem_Set_Callback(YModemObj_TypeDef *obj, uint8_t type, void *callback);
static void YModem_State_Polling(uint32_t sys_time, YModemObj_TypeDef *obj, uint8_t *p_buf, uint16_t size, YModem_Stream_TypeDef *p_stream);

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
    bool is_EOT = false;

    memset(&stream_out, 0, sizeof(YModem_Stream_TypeDef));
    if (obj && p_buf && size)
    {
        for (uint16_t i = 0; i < size; i ++)
        {
            switch (p_buf[i])
            {
                case SOH:
                    if ((size - i) >= 133)
                        pack_size = 128;
                    break;
            
                case STX:
                    if ((size - i) >= 1029)
                        pack_size = 1024;
                    break;

                case EOT:
                    if (size == 1)
                    {
                        pack_size = 0;
                        is_EOT = true;
                        if (obj->EOT_Cnt < 2)
                            obj->EOT_Cnt ++;
                        
                        if (obj->EOT_Cnt == 2)
                            /* last pack remain */
                            obj->wait_last_pack = true;
                    }
                    break;

                default:
                    pack_size = 0;
                    i ++;
                    break;
            }

            if (pack_size)
            {
                stream_out.p_buf = &p_buf[i + 3];
                stream_out.size = pack_size;

                stream_out.valid = YModem_Pack_Compelete;
                /* check crc */
                
                if (obj->data_income)
                {
                    if (obj->received_pack_num && (p_buf[i + 2] != obj->next_pack_id))
                    {
                        /* error pack id */
                        stream_out.valid = YModem_Pack_Invalid;
                        obj->data_income = false;
                    }

                    obj->cur_pack_id = p_buf[i + 1];
                    obj->next_pack_id = obj->cur_pack_id + 1;
                    obj->received_pack_num ++;
                }
            }
            else if ((pack_size == 0) && is_EOT)
            {
                stream_out.valid = YModem_Pack_Compelete;
                stream_out.size = 1;
                stream_out.p_buf = NULL;
            }
        }
    }

    return stream_out;
}

static void YModem_State_Polling(uint32_t sys_time, YModemObj_TypeDef *obj, uint8_t *p_buf, uint16_t size, YModem_Stream_TypeDef *p_stream)
{
    uint8_t tx_data[2] = {0};
    uint8_t tx_size = 0;

    /* polling currently processing ymodem object */
    if (obj)
    {
        switch ((uint8_t)(obj->state))
        {
            case YModem_State_Idle:
                obj->state = YModem_State_Tx;
                break;

            case YModem_State_Rx:
                /* check for time out */
                if ((size == 0) || (p_buf == NULL) || (p_stream == NULL))
                {
                    if (((obj->tx_stage == YModem_Req) && (sys_time >= obj->re_send_time)) || \
                        ((obj->tx_stage == YModem_ACK) && (sys_time >= obj->timeout_ms)))
                    {
                        obj->state = YModem_State_Tx;
                        obj->tx_stage = YModem_Req;
                    }
                    return;
                }

                /* update tx stage after receive data */
                switch ((uint8_t) obj->tx_stage)
                {
                    /* receive data on req stage */
                    case YModem_Req:
                        /* if receive pack data set send stage as ack */
                        *p_stream = YModem_Decode(obj, p_buf, size);
                        switch ((uint8_t)p_stream->valid)
                        {
                            case YModem_Pack_Compelete:
                                obj->state = YModem_State_Tx;
                                obj->tx_stage = YModem_ACK;
                                obj->tx_stage = YModem_Cfm;
                                break;

                            default:
                                /* unknow state */
                                break;
                        }
                        break;

                    /* receive data on ACK stage */
                    case YModem_ACK:
                        *p_stream = YModem_Decode(obj, p_buf, size);
                        switch ((uint8_t)p_stream->valid)
                        {
                            case YModem_Pack_Compelete:
                                obj->state = YModem_State_Tx;
                                obj->tx_stage = YModem_ACK;
                                if (p_stream->size == 1)
                                {
                                    if (!obj->wait_last_pack)
                                        obj->tx_stage = YModem_NAK;
                                }
                                break;

                            default:
                                /* unknow state */
                                break;
                        }
                        break;

                    default:
                        break;
                }
                break;

            case YModem_State_Tx:
                switch ((uint8_t) obj->tx_stage)
                {
                    case YModem_Req:
                        /* send 'C' */
                        tx_data[0] = C;
                        tx_size = 1;
                        /* after req data send accomplished check received data */
                        obj->state = YModem_State_Rx;
                        obj->re_send_time = sys_time + 100;
                        obj->received_pack_num = 0;
                        break;

                    case YModem_Cfm:
                        /* send ACk and C */
                        tx_data[0] = ACK;
                        tx_data[1] = C;
                        tx_size = 2;
                        /* wait data input */
                        obj->state = YModem_State_Rx;
                        obj->received_pack_num = 0;
                        break;

                    case YModem_ACK:
                        tx_data[0] = ACK;
                        tx_size = 1;
                        obj->state = YModem_State_Rx;
                        obj->timeout_ms = sys_time + 200;
                        break;

                    case YModem_NAK:
                        tx_data[0] = NAK;
                        tx_size = 1;
                        break;

                    default:
                        tx_size = 0;
                        break;
                }

                if (obj->send_callback && tx_size)
                    obj->send_callback(tx_data, tx_size);
                break;

            default:
                break;
        }
    }
}
