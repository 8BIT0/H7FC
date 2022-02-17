#ifndef __DEV_W25QXX_H
#define __DEV_W25QXX_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define W25Q128FV_FLASH_SIZE 0x1000000  /* 128 MBits => 16MBytes */
#define W25Q128FV_SECTOR_SIZE 0x10000   /* 256 sectors of 64KBytes */
#define W25Q128FV_SUBSECTOR_SIZE 0x1000 /* 4096 subsectors of 4kBytes */
#define W25Q128FV_PAGE_SIZE 0x100       /* 65536 pages of 256 bytes */

#define W25Q128FV_DUMMY_CYCLES_READ 4
#define W25Q128FV_DUMMY_CYCLES_READ_QUAD 10

#define W25Q128FV_BULK_ERASE_MAX_TIME 250000
#define W25Q128FV_SECTOR_ERASE_MAX_TIME 3000
#define W25Q128FV_SUBSECTOR_ERASE_MAX_TIME 800
#define W25Qx_TIMEOUT_VALUE 1000

/* Reset Operations */
#define RESET_ENABLE_CMD 0x66
#define RESET_MEMORY_CMD 0x99

#define ENTER_QPI_MODE_CMD 0x38
#define EXIT_QPI_MODE_CMD 0xFF

/* Identification Operations */
#define READ_ID_CMD 0x90
#define DUAL_READ_ID_CMD 0x92
#define QUAD_READ_ID_CMD 0x94
#define READ_JEDEC_ID_CMD 0x9F

/* Read Operations */
#define READ_CMD 0x03
#define FAST_READ_CMD 0x0B
#define DUAL_OUT_FAST_READ_CMD 0x3B
#define DUAL_INOUT_FAST_READ_CMD 0xBB
#define QUAD_OUT_FAST_READ_CMD 0x6B
#define QUAD_INOUT_FAST_READ_CMD 0xEB

/* Write Operations */
#define WRITE_ENABLE_CMD 0x06
#define WRITE_DISABLE_CMD 0x04

/* Register Operations */
#define READ_STATUS_REG1_CMD 0x05
#define READ_STATUS_REG2_CMD 0x35
#define READ_STATUS_REG3_CMD 0x15

#define WRITE_STATUS_REG1_CMD 0x01
#define WRITE_STATUS_REG2_CMD 0x31
#define WRITE_STATUS_REG3_CMD 0x11

/* Program Operations */
#define PAGE_PROG_CMD 0x02
#define QUAD_INPUT_PAGE_PROG_CMD 0x32

/* Erase Operations */
#define SECTOR_ERASE_CMD 0x20
#define CHIP_ERASE_CMD 0xC7

#define PROG_ERASE_RESUME_CMD 0x7A
#define PROG_ERASE_SUSPEND_CMD 0x75

/* Flag Status Register */
#define W25Q128FV_FSR_BUSY ((uint8_t)0x01) /*!< busy */
#define W25Q128FV_FSR_WREN ((uint8_t)0x02) /*!< write enable */
#define W25Q128FV_FSR_QE ((uint8_t)0x02)   /*!< quad enable */

#define W25Qx_Enable() HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)
#define W25Qx_Disable() HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)

typedef bool (*bus_init)(void);
typedef bool (*bus_transmit)(void *instance, uint8_t *tx, uint16_t size, uint16_t time_out);
typedef bool (*bus_receive)(void *instance, uint8_t *rx, uint16_t size, uint16_t time_out);
typedef bool (*bus_trans_receive)(void *instance, uint8_t *tx, uint8_t *rx, uint16_t size, uint16_t time_out);
typedef bool (*cs_pin_init)(void);
typedef bool (*cs_pin_ctl)(bool state);
typedef uint32_t (*get_systick)(void);

typedef enum
{
    DevW25Qxx_Norm_SpiBus = 0,
    DevW25Qxx_Quad_SpiBus,
} DevW25Qxx_SpiBustype_List;

typedef enum
{
    DevW25Qxx_Ok = 0,
    DevW25Qxx_Error,
    DevW25Qxx_Busy,
    DevW25Qxx_TimeOut,
} DevW25Qxx_Error_List;

#pragma pack(1)
typedef struct
{
    DevW25Qxx_SpiBustype_List bus_type;
    void *bus_instance;

    bus_init bus_init;
    bus_transmit trans;
    bus_receive receive;
    bus_trans_receive trans_receive;

    cs_pin_init cs_init;
    cs_pin_ctl cs_ctl;

    get_systick systick;
} DevW25QxxObj_TypeDef;
#pragma pack()

typedef struct
{
    DevW25Qxx_Error_List (*init)(DevW25QxxObj_TypeDef obj);
    DevW25Qxx_Error_List (*reset)(DevW25QxxObj_TypeDef obj);
    DevW25Qxx_Error_List (*write)(DevW25QxxObj_TypeDef obj, uint32_t addr, uint8_t *tx, uint32_t size);
    DevW25Qxx_Error_List (*read)(DevW25QxxObj_TypeDef obj, uint32_t addr, uint8_t *rx, uint32_t size);
    DevW25Qxx_Error_List (*erase_block)(DevW25QxxObj_TypeDef obj, uint32_t addr);
    DevW25Qxx_Error_List (*erase_chip)(DevW25QxxObj_TypeDef obj);
} DevW25Qxx_TypeDef;

extern DevW25Qxx_TypeDef DevW25Q64;

#endif
