#ifndef __BSP_SDRAM_DEF_H
#define __BSP_SDRAM_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef enum
{
    BspSDRAM_Bank_1 = 0,
    BspSDRAM_Bank_2,
} BspSDRAM_Bank_List;

typedef enum
{
    BspSDRAM_BankNum_2 = 0,
    BspSDRAM_BankNum_4,
} BspSDRAM_BandNum_List;

typedef enum
{
    BspSDRAM_Column_8Bits = 0,
    BspSDRAM_Column_9Bits,
    BspSDRAM_Column_10Bits,
    BspSDRAM_Column_11Bits,
} BspSDRAM_Column_List;

typedef enum
{
    BspSDRAM_Row_11Bits = 0,
    BspSDRAM_Row_12Bits,
    BspSDRAM_Row_13Bits,
} BspSDRAM_Row_List;

typedef enum
{
    BspSDRAM_BusWidth_8 = 0,
    BspSDRAM_BusWidth_16,
    BspSDRAM_BusWidth_32,
} BspSDRAM_BusWidth_List;

typedef struct
{
    bool init_state;
    uint32_t base_addr;
    uint32_t mem_size;

    uint8_t bank_num;
    uint8_t bank_area;
    uint8_t bus_width;
    uint8_t column_bits;
    uint8_t row_bits;

    void *hdl;
} BspSDRAMObj_TypeDef;

bool BspSDRAM_Init(BspSDRAMObj_TypeDef *obj);

#ifdef __cplusplus
}
#endif

#endif
