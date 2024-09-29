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

#define W25N01GVZEIG_ID     0x00EFAA21

#define W25NXX_BASE_ADDRESS 0x00000000

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

    uint32_t sector_size;
    uint32_t subsector_size;
    uint16_t page_size;

    uint16_t sector_num;
    uint16_t subsector_num;
    uint16_t page_num;
} DevW25Nxx_DeviceInfo_TypeDef;

typedef struct
{
    DevW25Nxx_ProdType_List prod_type;
    uint16_t prod_code;

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
    DevW25Nxx_ProdType_List (*type)(DevW25NxxObj_TypeDef *dev);
    DevW25Nxx_Error_List (*reset)(DevW25NxxObj_TypeDef *dev);
    DevW25Nxx_Error_List (*write)(DevW25NxxObj_TypeDef *dev, uint32_t addr, uint8_t *tx, uint32_t size);
    DevW25Nxx_Error_List (*read)(DevW25NxxObj_TypeDef *dev, uint32_t addr, uint8_t *rx, uint32_t size);
    DevW25Nxx_Error_List (*erase_sector)(DevW25NxxObj_TypeDef *dev, uint32_t addr);
    DevW25Nxx_Error_List (*erase_chip)(DevW25NxxObj_TypeDef *dev);
    DevW25Nxx_DeviceInfo_TypeDef (*info)(DevW25NxxObj_TypeDef *dev);
    uint32_t (*get_section_start_addr)(DevW25NxxObj_TypeDef *dev, uint32_t addr);
} DevW25Nxx_TypeDef;

extern DevW25Nxx_TypeDef DevW25Nxx;

#ifdef __cplusplus
}
#endif

#endif
