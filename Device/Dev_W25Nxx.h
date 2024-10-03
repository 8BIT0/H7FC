#ifndef __DEV_W25NXX_H
#define __DEV_W25NXX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define To_DevW25Nxx_API(x) ((DevW25Nxx_TypeDef *)x)
#define To_DevW25Nxx_OBJ(x) ((DevW25NxxObj_TypeDef *)x)

#define W25NXX_RESET_CMD                        0xFF
#define W25NXX_JEDEC_ID                         0x9F
#define W25NXX_READ_STATUS_CMD                  0x05    //0x0F
#define W25NXX_WRITE_STATUS_CMD                 0x01    //0x1F
#define W25NXX_WRITE_ENABLE                     0x06
#define W25NXX_WRITE_DISABLE                    0x04
#define W25NXX_BB_MANAGEMENT                    0xA1
#define W25NXX_READ_BB_LUT                      0xA5
#define W25NXX_LAST_ECC_FAILURE                 0xA9
#define W25NXX_BLOCK_ERASE                      0xD8
#define W25NXX_PROGRAM_DATA_LOAD                0x02
#define W25NXX_RANDOM_PROGRAM_DATA_LOAD         0x84
#define W25NXX_QUAD_PROGRAM_DATA_LOAD           0x32
#define W25NXX_RANDOM_QUAD_PROGRAM_DATA_LOAD    0x34
#define W25NXX_PROGRAM_EXECUTE                  0x10
#define W25NXX_PAGE_DATA_READ                   0x13
#define W25NXX_READ                             0x03
#define W25NXX_FAST_READ                        0x0B
#define W25NXX_FAST_READ_4BYTE                  0x0C
#define W25NXX_FAST_READ_DUAL_OUTPUT            0x3B
#define W25NXX_FAST_READ_DUAL_OUTPUT_4BYTE      0x3C
#define W25NXX_FAST_READ_QUAD_OUTPUT            0x6B
#define W25NXX_FAST_READ_QUAD_OUTPUT_4BYTE      0x6C
#define W25NXX_FAST_READ_DUAL_IO                0xBB
#define W25NXX_FAST_READ_DUAL_IO_4BYTE          0xBC
#define W25NXX_FAST_READ_QUAD_IO                0xEB
#define W25NXX_FAST_READ_QUAD_IO_4BYTE          0xEC

#define W25NXX_SR0_ADDR                         0xA0
#define W25NXX_SR1_ADDR                         0xB0
#define W25NXX_SR2_ADDR                         0xC0

#define W25N01GVZEIG_ID                         0x00EFAA21

#define W25NXX_BASE_ADDRESS                     0x00000000
#define W25NXX_PAGE_SIZE                        0x800
#define W25NXX_PAGE_PRE_BLOCK                   0x40

#define W25N01GV_FLASH_SIZE                     0x8000000
#define W25N01GV_BLOCK_SIZE                     0x20000
#define W25N01GV_BLOCK_NUM                      0x400
#define W25N01GV_PAGE_SIZE                      W25NXX_PAGE_SIZE
#define W25N01GV_PAGE_NUM                       65536
#define W25N01GV_BUFFER_MODE_SIZE               W25NXX_PAGE_SIZE
#define W25N0GV_ECC_INFO_SIZE                   0x40

typedef enum
{
    BF_SRP_1 = 0,
    BF_WPE,
    BF_TB,
    BF_BP_0,
    BF_BP_1,
    BF_BP_2,
    BF_BP_3,
    BF_SRP_0,
} DevW25Nxx_SR0_BitField_TypeDef;

typedef enum
{
    BF_RES_SR1 = 0,
    BF_BUF,
    BF_ECC_E,
    BF_SR1_L,
    BF_OTP_E,
    BF_OTP_L,
} DevW25Nxx_SR1_BitField_TypeDef;

typedef enum
{
    BF_BUSY = 0,
    BF_WEL,
    BF_E_FAIL,
    BF_P_FAIL,
    BF_ECC_0,
    BF_ECC_1,
    BF_LUT_F,
    BF_RES_SR2,
} DevW25Nxx_SR2_BitField_TypeDef;

typedef union
{
    uint8_t val;
    struct
    {
        uint8_t SRP_1 : 1;
        uint8_t WPE   : 1;
        uint8_t TB    : 1;
        uint8_t BP_0  : 1;
        uint8_t BP_1  : 1;
        uint8_t BP_2  : 1;
        uint8_t BP_3  : 1;
        uint8_t SRP_0 : 1;
    } bit;
} DevW25Nxx_SR0_TypeDef;

typedef union
{
    uint8_t val;
    struct
    {
        uint8_t RES   : 3;
        uint8_t BUF   : 1;
        uint8_t ECC_E : 1;
        uint8_t SR1_L : 1;
        uint8_t OTP_E : 1;
        uint8_t OTP_L : 1;
    } bit;
} DevW25Nxx_SR1_TypeDef;

typedef union
{
    uint8_t val;
    struct
    {
        uint8_t BUSY   : 1;
        uint8_t WEL    : 1;
        uint8_t E_FAIL : 1;
        uint8_t P_FAIL : 1;
        uint8_t ECC_0  : 1;
        uint8_t ECC_1  : 1;
        uint8_t LUT_F  : 1;
        uint8_t RES    : 1;
    } bit;
} DevW25Nxx_SR2_TypeDef;

typedef enum
{
    DevW25N_None = 0,
    DevW25N_01,
    DevW25N_02,
} DevW25Nxx_ProdType_List;

typedef enum
{
    DevW25Nxx_Ok = 0,
    DevW25Nxx_Error,
    DevW25Nxx_Busy,
    DevW25Nxx_TimeOut,
} DevW25Nxx_Error_List;

typedef struct
{
    DevW25Nxx_ProdType_List prod_type;
    uint16_t prod_code;

    uint32_t start_addr;
    uint32_t flash_size;

    uint16_t page_size;
    uint32_t page_num;

    uint16_t block_num;
    uint32_t block_size;
} DevW25Nxx_DeviceInfo_TypeDef;

typedef struct
{
    DevW25Nxx_ProdType_List prod_type;
    uint32_t prod_code;
    uint8_t *tmp_buf;

    uint16_t (*bus_tx)(uint8_t *p_data, uint16_t len, uint32_t time_out);
    uint16_t (*bus_rx)(uint8_t *p_data, uint16_t len, uint32_t time_out);
    uint16_t (*bus_trans)(uint8_t *tx_data, uint8_t *rx_data, uint16_t len, uint32_t time_out);
    bool (*cs_ctl)(bool state);
    void (*delay_ms)(uint32_t ms);
    uint32_t (*systick)(void);

    DevW25Nxx_Error_List init_state;
} DevW25NxxObj_TypeDef;

typedef struct
{
    DevW25Nxx_Error_List (*init)(DevW25NxxObj_TypeDef *dev);
    DevW25Nxx_Error_List (*write)(DevW25NxxObj_TypeDef *dev, uint32_t addr, uint8_t *tx, uint32_t size);
    DevW25Nxx_Error_List (*read)(DevW25NxxObj_TypeDef *dev, uint32_t addr, uint8_t *rx, uint32_t size);
    DevW25Nxx_Error_List (*erase_page)(DevW25NxxObj_TypeDef *dev, uint32_t addr);
    DevW25Nxx_Error_List (*erase_chip)(DevW25NxxObj_TypeDef *dev);
    DevW25Nxx_DeviceInfo_TypeDef (*info)(DevW25NxxObj_TypeDef *dev);
    uint32_t (*get_page)(DevW25NxxObj_TypeDef *dev, uint32_t addr);
} DevW25Nxx_TypeDef;

extern DevW25Nxx_TypeDef DevW25Nxx;

#ifdef __cplusplus
}
#endif

#endif
