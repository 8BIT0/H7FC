#include "YModem.h"
#include "../../FCHW_Config.h"
#include "util.h"

#define YMODEM_MIN_SIZE 133
#define YMODEM_MAX_SIZE 1029
#define YMODEM_EOT_SIZE 1
#define YMODEM_EOT_CNT  2
#define YMODEM_PAYLOAD_OFFSET 3
#define YMODEM_ID_P_OFFSET 1
#define YMODEM_ID_N_OFFSET 2

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
static uint8_t YModem_State_Polling(uint32_t sys_time, YModemObj_TypeDef *obj, uint8_t *p_buf, uint16_t size, YModem_Stream_TypeDef *p_stream);
static void YModem_Abort(YModemObj_TypeDef *obj);

YModem_TypeDef YModem = {
    .polling = YModem_State_Polling,
    .abort = YModem_Abort,
};

static YModem_Stream_TypeDef YModem_Decode(YModemObj_TypeDef *obj, uint8_t *p_buf, uint16_t size)
{
    YModem_Stream_TypeDef stream_out;
    uint16_t pack_size = 0;
    bool is_EOT = false;
    volatile uint16_t crc_get = 0;
    volatile uint16_t crc = 0;

    memset(&stream_out, 0, sizeof(YModem_Stream_TypeDef));
    if (obj && p_buf && size)
    {
        stream_out.size = size;

        for (uint16_t i = 0; i < size; i ++)
        {
            switch (p_buf[i])
            {
                case SOH:
                    if ((size - i) >= YMODEM_MIN_SIZE)
                    {
                        crc_get = p_buf[YMODEM_MIN_SIZE - 1];
                        crc_get |= p_buf[YMODEM_MIN_SIZE - 2] << 8;
                        pack_size = YMODEM_MIN_SIZE - 5;
                    }
                    else
                        stream_out.valid = YModem_Pack_InCompelete;
                    break;
            
                case STX:
                    if ((size - i) >= YMODEM_MAX_SIZE)
                    {
                        crc_get = p_buf[YMODEM_MAX_SIZE - 1];
                        crc_get |= p_buf[YMODEM_MAX_SIZE - 2] << 8;
                        pack_size = YMODEM_MAX_SIZE - 5;
                    }
                    else
                        stream_out.valid = YModem_Pack_InCompelete;
                    break;

                case EOT:
                    if (size == YMODEM_EOT_SIZE)
                    {
                        pack_size = 0;
                        is_EOT = true;
                        obj->data_income = false;
                        if (obj->EOT_Cnt < YMODEM_EOT_CNT)
                            obj->EOT_Cnt ++;
                        
                        if (obj->EOT_Cnt == YMODEM_EOT_CNT)
                            /* last pack remain */
                            obj->wait_last_pack = true;
                    }
                    break;

                default:
                    pack_size = 0;
                    break;
            }

            if (pack_size && ((p_buf[i + YMODEM_ID_N_OFFSET] + p_buf[i + YMODEM_ID_P_OFFSET]) == 0xFF))
            {
                stream_out.p_buf = &p_buf[i + YMODEM_PAYLOAD_OFFSET];
                stream_out.size = pack_size;

                stream_out.valid = YModem_Pack_Invalid;
                crc = Common_CRC16(&p_buf[i + YMODEM_PAYLOAD_OFFSET], pack_size);
                /* check crc */
                if (crc_get == crc) 
                {
                    stream_out.valid = YModem_Pack_Compelete;

                    if (!obj->data_income || obj->wait_last_pack)
                    {
                        /* is not file data */
                        stream_out.file_data = false;
                        break;
                    }

                    stream_out.file_data = true;
                    if (obj->received_pack_num && (p_buf[i + YMODEM_ID_P_OFFSET] != obj->next_pack_id))
                    {
                        /* error pack id */
                        stream_out.valid = YModem_Pack_Invalid;
                        obj->data_income = false;
                    }

                    obj->cur_pack_id = p_buf[i + YMODEM_ID_P_OFFSET];
                    obj->next_pack_id = obj->cur_pack_id + 1;
                    obj->received_pack_num ++;
                }

                break;
            }
            else if ((pack_size == 0) && is_EOT)
            {
                stream_out.valid = YModem_Pack_Compelete;
                stream_out.size = 1;
                stream_out.p_buf = NULL;

                break;
            }
            else if (stream_out.valid == YModem_Pack_InCompelete)
                break;
        }
    }

    return stream_out;
}

static void YModem_Rx_State_Polling(uint32_t sys_time, YModemObj_TypeDef *obj, uint8_t *p_buf, uint16_t size, YModem_Stream_TypeDef *p_stream)
{
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

    *p_stream = YModem_Decode(obj, p_buf, size);
    if (p_stream->valid == YModem_Pack_Invalid)
    {
        /* send NAK to host */
        obj->state = YModem_State_Tx;
        obj->tx_stage = YModem_NAK;

        /* check for re-send */

        /* check for time out */

        return;
    }
    else if (p_stream->valid == YModem_Pack_InCompelete)
        return;

    /* update tx stage after receive data */
    switch ((uint8_t) obj->tx_stage)
    {
        /* receive data after req */
        case YModem_Req:
            /* file name received */
            obj->state = YModem_State_Tx;
            if (p_stream->valid != YModem_Pack_Compelete)
                break;

            obj->tx_stage = YModem_Cfm;
            obj->data_income = true;
            break;

        /* receive data after confirm */
        case YModem_Cfm:
            /* is first pack received */
            obj->state = YModem_State_Tx;
            if (p_stream->valid != YModem_Pack_Compelete)
                break;

            obj->tx_stage = YModem_ACK;
            break;

        /* receive data after ACK */
        case YModem_ACK:
            obj->state = YModem_State_Tx;
            if (p_stream->valid != YModem_Pack_Compelete)
                break;

            obj->tx_stage = YModem_ACK;
            if (p_stream->size == 1)
            {
                if (!obj->wait_last_pack)
                {
                    obj->tx_stage = YModem_NAK;
                }
                else
                    obj->tx_stage = YModem_Lst;
            }
            break;

        case YModem_Lst:
            /* end trans */
            obj->state = YModem_State_Tx;
            if (p_stream->valid != YModem_Pack_Compelete)
                break;

            obj->tx_stage = YModem_EOT;
            break;

        default:
            break;
    }
}

static void YModem_Tx_State_Polling(uint32_t sys_time, YModemObj_TypeDef *obj)
{
    uint8_t tx_data[2] = {0};
    uint8_t tx_size = 0;

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
            obj->state = YModem_State_Rx;
            obj->timeout_ms = sys_time + 200;
            obj->tx_stage = YModem_ACK;
            break;

        case YModem_Lst:
            /* send ACk and C */
            tx_data[0] = ACK;
            tx_data[1] = C;
            tx_size = 2;
            /* wait data input */
            obj->state = YModem_State_Rx;
            break;

        case YModem_EOT:
            /* send last ack to host */
            tx_data[0] = ACK;
            tx_size = 1;
            /* proto finish */
            obj->state = YModem_State_Finish;
            break;

        default:
            tx_size = 0;
            break;
    }

    if (obj->send_callback && tx_size)
        obj->send_callback(tx_data, tx_size);
}

static void YModem_Abort(YModemObj_TypeDef *obj)
{
    uint8_t abort_cmd = CAN;

    if (obj && obj->send_callback)
    {
        obj->send_callback(&abort_cmd, 1);
        obj->send_callback(&abort_cmd, 1);
    }
}

static uint8_t YModem_State_Polling(uint32_t sys_time, YModemObj_TypeDef *obj, uint8_t *p_buf, uint16_t size, YModem_Stream_TypeDef *p_stream)
{
    /* polling currently processing ymodem object */
    if (obj)
    {
        switch ((uint8_t)(obj->state))
        {
            case YModem_State_Idle:
                obj->state = YModem_State_Tx;
                break;

            case YModem_State_Rx:
                YModem_Rx_State_Polling(sys_time, obj, p_buf, size, p_stream);
                break;

            case YModem_State_Tx:
                YModem_Tx_State_Polling(sys_time, obj);
                break;

            case YModem_State_Finish:
                break;

            default:
                break;
        }

        return (uint8_t)(obj->state);
    }

    return YModem_State_Error;
}
