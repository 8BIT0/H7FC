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

static void YModem_Recv(YModemObj_TypeDef *Obj, uint8_t *p_buf, uint16_t len)
{
    uint16_t recv_data_len = 0;
    YModem_Frame_TypeDef Frame;
    uint16_t index = 0;
    uint16_t data_size = 0;
    uint8_t *p_stream_buf = NULL;
    uint16_t *p_crc16 = NULL;
    uint16_t crc16 = 0;

    if (Obj && \
        Obj->rx_stream.p_buf && \
        Obj->rx_stream.total_size && \
        (Obj->rx_stream.total_size > Obj->rx_stream.cur_size) && \
        ((Obj->rx_stream.total_size - Obj->rx_stream.cur_size) >= len) && \
        p_buf && (len > 4))
    {
        while (true)
        {
            Obj->state_reg.bit.recv = true;
            Obj->state = YModem_State_Rx;
            
            if (p_buf[index] == SOH)
            {
                recv_data_len = YMODEM_MIN_SIZE;
            }
            else if (p_buf[index] == STX)
            {
                recv_data_len = YMODEM_MAX_SIZE;
            }

            if (Obj->remain_byte == 0)
            {
                Frame.header = p_buf[index];
                Frame.pack_id = p_buf[++index];
                Frame.reverse_id = p_buf[++index];
                Frame.p_data = &p_buf[++index];
                p_stream_buf = Frame.p_data; 

                Obj->next_pack_id = Frame.pack_id ++;

                data_size = len - 3;
                if (Obj->rx_stream.cur_size + data_size > Obj->rx_stream.total_size)
                {
                    /* no enough space for receive data */
                    /* error state */
                    /* after cache process finished request this frame again */
                    /* abort current pack receive */
                    Obj->next_pack_id --;
                    Obj->state = YModem_State_Rx_Failed;
                    return;
                }

                /* is not a full pack */
                if (recv_data_len > data_size)
                {
                    Obj->state = YModem_NotFull_Pack;
                    Obj->remain_byte = recv_data_len - data_size;
                    break;
                }
                else
                {
                    /* noticed endian */
                    *p_crc16 = (uint16_t *)(p_stream_buf + recv_data_len);
                    crc16 = Common_CRC16(p_stream_buf, recv_data_len);

                    /* check crc */
                    if (*p_crc16 == crc16)
                    {
                        memcpy((Obj->rx_stream.p_buf + Obj->rx_stream.cur_size), p_stream_buf, recv_data_len);
                        Obj->rx_stream.cur_size += recv_data_len;
                        Obj->state = YModem_State_Rx_PackDone;
                        break;
                    }
                    else
                    {
                        /* pack receive failed */
                        memset(Obj->rx_stream.p_buf, 0, Obj->rx_stream.total_size);
                        Obj->rx_stream.cur_size = 0;
                        Obj->state = YModem_State_Rx_Failed;
                        break;
                    }
                }
            }
            else
            {
                /* header matched */
                /* and prev pack is not full pack size in come */
                if (Obj->remain_byte > len)
                {
                    Obj->remain_byte -= len;
                }
                else
                {
                    len -= Obj->remain_byte;
                    Obj->remain_byte = 0;
                }
            }
            
            Obj->state_reg.bit.recv = false;
        }
    }
}

static void YModem_Pack(YModemObj_TypeDef *obj, uint8_t *p_buf, uint16_t len)
{
    if (obj && p_buf && len)
    {

    }
}

static void YModem_State_Polling(YModemObj_TypeDef *obj)
{
    /* polling currently processing ymodem object */
    if (obj)
    {
        /* if data is receiving polling process nothing */
        if (obj->state_reg.bit.recv)
            return;

        obj->state_reg.bit.poll = true;

        switch ((uint8_t)(obj->state))
        {
            case YModem_State_Idle:
                break;

            case YModem_State_Rx:
                break;

            case YModem_State_Tx:
                break;

            case YModem_State_Rx_PackDone:
                break;

            case YModem_State_Rx_Failed:
                break;

            case YModem_State_Tx_Failed:
                break;

            case YModem_NotFull_Pack:
                break;

            case YModem_State_TimeOut:
                break;

            default:
                break;
        }

        obj->state_reg.bit.poll = false;
    }
}
