#ifndef __DEV_CRSF_H
#define __DEV_CRSF_H

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

// Basic setup
#define CRSF_MAX_CHANNEL 16
#define CRSF_FRAME_SIZE_MAX 64

// Device address & type
#define RADIO_ADDRESS 0xEA
// #define ADDR_MODULE             0xEE  //  Crossfire transmitter
#define TYPE_CHANNELS 0x16

#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811

#pragma pack(1)
typedef struct
{
    uint16_t Val[CRSF_MAX_CHANNEL];
    uint16_t channel_cnt;
} DevCRSF_Pack_TypeDef;
#pragma pack()

#endif
