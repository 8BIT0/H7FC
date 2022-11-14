#ifndef __DEV_CRSF_H
#define __DEV_CRSF_H

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

typedef void (*CRSF_Callback)(uint8_t *ptr, uint16_t size);

#define CRSF_BAUDRATE 420000

// Basic setup
#define CRSF_MAX_CHANNEL 16
#define CRSF_FRAME_SIZE_MAX 64

// Device address & type
#define RADIO_ADDRESS 0xEA
// #define ADDR_MODULE             0xEE  //  Crossfire transmitter
#define TYPE_CHANNELS 0x16

#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_1000 191
#define CRSF_DIGITAL_CHANNEL_MID 992
#define CRSF_DIGITAL_CHANNEL_2000 1792
#define CRSF_DIGITAL_CHANNEL_MAX 1811
#define CRSF_DIGITAL_CHANNEL_SPAN (CRSF_DIGITAL_CHANNEL_MAX - CRSF_DIGITAL_CHANNEL_MIN)

// Clashes with CRSF_ADDRESS_FLIGHT_CONTROLLER
#define CRSF_SYNC_BYTE 0XC8

#define CRSF_FRAME_LENGTH_ADDRESS 1         // length of ADDRESS field
#define CRSF_FRAME_LENGTH_FRAMELENGTH 1     // length of FRAMELENGTH field
#define CRSF_FRAME_LENGTH_TYPE 1            // length of TYPE field
#define CRSF_FRAME_LENGTH_CRC 1             // length of CRC field
#define CRSF_FRAME_LENGTH_TYPE_CRC 2        // length of TYPE and CRC fields combined
#define CRSF_FRAME_LENGTH_EXT_TYPE_CRC 4    // length of Extended Dest/Origin, TYPE and CRC fields combined
#define CRSF_FRAME_LENGTH_NON_PAYLOAD 4     // combined length of all fields except payload

#define CRSF_FRAME_GPS_PAYLOAD_SIZE 15
#define CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE 8
#define CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE 10
#define CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE 22  // 11 bits per channel * 16 channels = 22 bytes.
#define CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE 6

// Packet timeout where buffer is flushed if no data is received in this time
#define CRSF_PACKET_TIMEOUT_MS 100
#define CRSF_FAILSAFE_STAGE1_MS 300


typedef enum
{
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
    CRSF_FRAMETYPE_COMMAND = 0x32,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
} crsf_frame_type_list;

typedef enum
{
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
} crsf_addr_list;

typedef enum
{
    CRSF_State_LinkUp = 0,
    CRSF_State_LinkDown,
    CRSF_State_TimeOut,
}crsf_state_list;

#pragma pack(1)
typedef struct
{
    crsf_addr_list device_addr;
    uint8_t frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    crsf_frame_type_list type;
    uint8_t *data;
} crsf_header_t;

typedef struct
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} crsf_channels_t;

typedef struct
{
    unsigned voltage : 16;  // V * 10 big endian
    unsigned current : 16;  // A * 10 big endian
    unsigned capacity : 24; // mah
    unsigned remaining : 8; // %
} crsf_sensor_battery_t;
#pragma pack()

typedef struct
{
    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality;
    int8_t uplink_SNR;
    uint8_t active_antenna;
    uint8_t rf_Mode;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI;
    uint8_t downlink_Link_quality;
    int8_t downlink_SNR;
} crsfLinkStatistics_t;

typedef struct
{
    bool (*port_init)(uint32_t baudrate);
    uint8_t rx_buf[CRSF_FRAME_SIZE_MAX + 3];
    uint32_t update_rt;
    crsf_state_list state;
    bool failsafe;

    CRSF_Callback link_up_cb;
    CRSF_Callback link_down_cb;
}DevCRSFObj_TypeDef;

typedef struct
{
    bool (*init)(DevCRSFObj_TypeDef *obj);
    bool (*decode)(DevCRSFObj_TypeDef *obj, uint8_t *p_data, uint16_t len);
    bool (*core)(DevCRSFObj_TypeDef *obj);
} DevCRSF_TypeDef;

#endif
