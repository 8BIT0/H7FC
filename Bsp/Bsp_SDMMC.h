#ifndef __BSP_SDMMC_H
#define __BSP_SDMMC_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_sd.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_ll_sdmmc.h"

// #define SDIO_USE_4BIT true
// #define SDIO_CK_PIN PC12
// #define SDIO_CMD_PIN PD2
// #define SDIO_D0_PIN PC8
// #define SDIO_D1_PIN PC9
// #define SDIO_D2_PIN PC10
// #define SDIO_D3_PIN PC11

#pragma pack(1)
typedef struct
{
    GPIO_TypeDef *D0_Port;
    GPIO_TypeDef *D1_Port;
    GPIO_TypeDef *D2_Port;
    GPIO_TypeDef *D3_Port;
    GPIO_TypeDef *CK_Port;
    GPIO_TypeDef *CMD_Port;

    uint32_t D0_Pin;
    uint32_t D1_Pin;
    uint32_t D2_Pin;
    uint32_t D3_Pin;
    uint32_t CK_Pin;
    uint32_t CMD_Pin;

    uint32_t Alternate;
} BspSDMMC_PinConfig_TypeDef;

typedef struct
{
    BspSDMMC_PinConfig_TypeDef *pin;
    SD_HandleTypeDef hdl;
    SD_TypeDef *instance;
    HAL_SD_CardInfoTypeDef info;
} BspSDMMC_Obj_TypeDef;
#pragma pack()

typedef struct
{
    bool (*init)(BspSDMMC_Obj_TypeDef *obj);
    bool (*read)(BspSDMMC_Obj_TypeDef *obj, uint32_t addr, uint8_t *data, uint32_t size);
    bool (*write)(BspSDMMC_Obj_TypeDef *obj, uint32_t addr, uint8_t *data, uint32_t size);
    bool (*erase)(BspSDMMC_Obj_TypeDef *obj, uint32_t addr, uint32_t start_addr, uint32_t end_addr);
    bool (*status)(BspSDMMC_Obj_TypeDef *obj);
    bool (*info)(BspSDMMC_Obj_TypeDef *obj, HAL_SD_CardInfoTypeDef *info);
} BspSDMMC_TypeDef;

extern BspSDMMC_TypeDef BspSDMMC;

#endif
