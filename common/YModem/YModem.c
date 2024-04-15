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

    if (Obj && \
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
                Frame.pack_id = p_buf[index + 1];
                Frame.reverse_id = p_buf[index + 2];
                Frame.p_data = &p_buf[index + 3];

                Obj->next_pack_id = Frame.pack_id + 1;

                if (Processing_YModem_Obj == NULL)
                    Processing_YModem_Obj = Obj;

                data_size = len - 3;

                /* is not a full pack */
                if (recv_data_len > len)
                {
                    if (Obj->rx_stream.cur_size + data_size > Obj->rx_stream.total_size)
                    {
                        /* no enough space for receive data */
                    }
                    
                    Obj->remain_byte = recv_data_len - data_size;

                    break;
                }
                else
                {
                    // if ((Obj->state == YModem_State_Idle) && ())


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

        Processing_YModem_Obj->state_reg.bit.poll = false;
    }
}
