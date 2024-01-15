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
#include "stm32h7xx_hal_mdma.h"

#define SDMMC_OPR_RETRY_MAX_CNT 0xFFFFFFFF

// #define SDIO_USE_4BIT true
// #define SDIO_CK_PIN PC12
// #define SDIO_CMD_PIN PD2
// #define SDIO_D0_PIN PC8
// #define SDIO_D1_PIN PC9
// #define SDIO_D2_PIN PC10
// #define SDIO_D3_PIN PC11

typedef uint16_t (*SDMMC_Callback)(uint8_t *p_data, uint16_t size);

typedef struct
{
    SDMMC_Callback Write_Callback;
    SDMMC_Callback Read_Callback;
    SDMMC_Callback Error_Callback;
}BspSDMMC_Callback_List_TypeDef;

typedef enum
{
  BspSDMMC_Opr_State_RESET = 0,
  BspSDMMC_Opr_State_READY,
  BspSDMMC_Opr_State_TIMEOUT,
  BspSDMMC_Opr_State_BUSY,
  BspSDMMC_Opr_State_PROGRAMMING,
  BspSDMMC_Opr_State_RECEIVING,
  BspSDMMC_Opr_State_TRANSFER,
  BspSDMMC_Opr_State_ERROR,
}BspSDMMC_OperationState_List;

typedef enum
{
    BspSDMMC_1_Callback = 0,
    BspSDMMC_2_Callback,
    BspSDMMC_Callback_Index_Sum,
}BspSDMMC_Callback_ListItem_Def;

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

typedef enum
{
    BspSDMMC_Callback_Type_Write = 0,
    BspSDMMC_Callback_Type_Read,
    BspSDMMC_Callback_Type_Error,
}BspSDMMC_Callback_TypeList;

typedef struct
{
    BspSDMMC_PinConfig_TypeDef *pin;
    SD_HandleTypeDef hdl;
    MDMA_HandleTypeDef mdma;
    SD_TypeDef *instance;
    HAL_SD_CardInfoTypeDef info;

    BspSDMMC_Callback_List_TypeDef *ref_callback_item;

    SDMMC_Callback Write_Callback;
    SDMMC_Callback Read_Callback;
    SDMMC_Callback Error_Callback;
} BspSDMMC_Obj_TypeDef;

typedef struct
{
    bool (*init)(BspSDMMC_Obj_TypeDef *obj);
    bool (*read)(BspSDMMC_Obj_TypeDef *obj, uint32_t addr, uint8_t *data, uint32_t size);
    bool (*write)(BspSDMMC_Obj_TypeDef *obj, uint32_t addr, uint8_t *data, uint32_t size);
    bool (*erase)(BspSDMMC_Obj_TypeDef *obj, uint32_t addr, uint32_t start_addr, uint32_t end_addr);
    bool (*card_status)(BspSDMMC_Obj_TypeDef *obj);
    bool (*info)(BspSDMMC_Obj_TypeDef *obj, HAL_SD_CardInfoTypeDef *info);
    void (*set_callback)(BspSDMMC_Obj_TypeDef *obj, BspSDMMC_Callback_TypeList type, SDMMC_Callback cb);
    BspSDMMC_OperationState_List (*get_opr_state)(BspSDMMC_Obj_TypeDef *obj);
} BspSDMMC_TypeDef;

extern BspSDMMC_TypeDef BspSDMMC;

#endif
