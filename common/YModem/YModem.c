#include "YModem.h"

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

static YModemObj_TypeDef *Processing_YModem_Obj = NULL;

typedef enum
{
    SOH    = 0x01,
    STX    = 0x02,
    EOT    = 0x04,
    ACK    = 0x06,
    NAK    = 0x15,
    CAN    = 0x18,
    C      = 0x43,
    ABORT1 = 0x41,
    ABORT2 = 0x61,
} YModem_CMD_List;

static uint16_t YModem_Cal_CRC(uint8_t *p_buf, uint16_t len)
{
    if (p_buf && len)
    {

    }

    return 0;
}

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
        ((Obj == Processing_YModem_Obj) || \
        (Processing_YModem_Obj == NULL)) && \
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

                if (Processing_YModem_Obj == NULL)
                    Processing_YModem_Obj = Obj;

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
                    memcpy((Obj->rx_stream.p_buf + Obj->rx_stream.cur_size), p_stream_buf, recv_data_len);
                    p_crc16 = (uint16_t *)(p_stream_buf + recv_data_len);

                    /* check crc */
                    // crc16 = YModem_Cal_CRC(, );
                    if (*p_crc16 == crc16)
                    {

                    }
                    else
                    {
                        /* pack receive failed */
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

static void YModem_State_Polling(void)
{
    /* polling currently processing ymodem object */
    if (Processing_YModem_Obj)
    {
        /* if data is receiving polling process nothing */
        if (Processing_YModem_Obj->state_reg.bit.recv)
            return;

        Processing_YModem_Obj->state_reg.bit.poll = true;

        switch ((uint8_t)(Processing_YModem_Obj->state))
        {
            case YModem_State_Idle:
                break;

            case YModem_State_Rx:
                break;

            case YModem_State_Tx:
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

        Processing_YModem_Obj->state_reg.bit.poll = false;
    }
}
