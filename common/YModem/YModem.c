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
static void YModem_State_Polling(YModemObj_TypeDef *obj, uint8_t *p_bug, uint16_t *size);

YModem_TypeDef YModem = {
    .polling = YModem_State_Polling,
};

static void YModem_State_Polling(YModemObj_TypeDef *obj, uint8_t *p_buf, uint16_t *size)
{
    /* polling currently processing ymodem object */
    if (obj && p_buf && size)
    {
        switch ((uint8_t)(obj->state))
        {
            case YModem_State_Idle:
                obj->state = YModem_State_Tx;
                break;

            case YModem_State_Rx:
                if (*size)
                {
                    /* update tx stage */
                    switch ((uint8_t) obj->tx_stage)
                    {
                        case YModem_Req:
                            obj->tx_stage = YModem_ACK;
                            break;

                        case YModem_ACK:
                            break;

                        default:
                            break;
                    }
                }
                // else
                //     return YModem_State_Rx_Waiting;
                break;

            case YModem_State_Tx:
                switch ((uint8_t) obj->tx_stage)
                {
                    case YModem_Req:
                        /* send 'C' */
                        if (obj->send_callback)
                            obj->send_callback(C, 1);
                        break;

                    default:
                        break;
                }

                /* after req data send accomplished check received data */
                obj->state = YModem_State_Rx;
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
