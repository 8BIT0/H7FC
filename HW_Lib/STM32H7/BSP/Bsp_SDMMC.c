#include "Bsp_SDMMC.h"

static const GPIO_InitTypeDef BspSDMMC_PinCfg = {
    .Mode = GPIO_MODE_AF_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
};

/* for stm32h743 only have 2 SDMMC bus */
static BspSDMMC_Callback_List_TypeDef BspSCMMC_Callback_List[BspSDMMC_Callback_Index_Sum] = {0};

/* internal function */
static bool BspSDMMC_PortCLK_Init(SDMMC_TypeDef *instance);
static void BspSDMMC_PinCLK_Enable(GPIO_TypeDef *port);

/* external function */
static bool BspSDMMC_Init(BspSDMMC_Obj_TypeDef *obj);
static bool BspSDMMC_Read(BspSDMMC_Obj_TypeDef *obj, uint8_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks);
static bool BspSDMMC_Write(BspSDMMC_Obj_TypeDef *obj, uint8_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks);
static bool BspSDMMC_Erase(BspSDMMC_Obj_TypeDef *obj, uint32_t StartAddr, uint32_t EndAddr);
static bool BspSDMMC_GetCardStatus(BspSDMMC_Obj_TypeDef *obj);
static bool BspSDMMC_GetInfo(BspSDMMC_Obj_TypeDef *obj, HAL_SD_CardInfoTypeDef *info_out);
static void BspSDMMC_Set_Callback(BspSDMMC_Obj_TypeDef *obj, BspSDMMC_Callback_TypeList type, SDMMC_Callback cb);
static BspSDMMC_OperationState_List BspSDMMC_Get_Operation_State(BspSDMMC_Obj_TypeDef *obj);

BspSDMMC_TypeDef BspSDMMC = {
    .init = BspSDMMC_Init,
    .read = BspSDMMC_Read,
    .write = BspSDMMC_Write,
    .erase = BspSDMMC_Erase,
    .card_status = BspSDMMC_GetCardStatus,
    .info = BspSDMMC_GetInfo,
    .set_callback = BspSDMMC_Set_Callback,
    .get_opr_state = BspSDMMC_Get_Operation_State,
};

static void BspSDMMC_PinCLK_Enable(GPIO_TypeDef *port)
{
    if (port == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if (port == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if (port == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if (port == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    else if (port == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
    else if (port == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
    else if (port == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
    else if (port == GPIOH)
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
    else if (port == GPIOJ)
    {
        __HAL_RCC_GPIOJ_CLK_ENABLE();
    }
    else if (port == GPIOK)
    {
        __HAL_RCC_GPIOK_CLK_ENABLE();
    }
}

static bool BspSDMMC_PortCLK_Init(SDMMC_TypeDef *instance)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if (instance == NULL)
        return false;

    if (instance == SDMMC1)
    {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC;
        PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
            return false;

        __HAL_RCC_SDMMC1_CLK_ENABLE();
        return true;
    }

    return true;
}

static bool BspSDMMC_Pin_Init(SD_TypeDef *type, BspSDMMC_PinConfig_TypeDef *obj)
{
    GPIO_InitTypeDef GPIO_InitStruct = BspSDMMC_PinCfg;

    if (obj == NULL)
        return false;

    if (type == SDMMC1)
    {
        GPIO_InitStruct.Alternate = obj->Alternate;

        BspSDMMC_PinCLK_Enable(obj->CK_Port);
        GPIO_InitStruct.Pin = obj->CK_Pin;
        HAL_GPIO_Init(obj->CK_Port, &GPIO_InitStruct);

        BspSDMMC_PinCLK_Enable(obj->CMD_Port);
        GPIO_InitStruct.Pin = obj->CMD_Pin;
        HAL_GPIO_Init(obj->CMD_Port, &GPIO_InitStruct);

        BspSDMMC_PinCLK_Enable(obj->D0_Port);
        GPIO_InitStruct.Pin = obj->D0_Pin;
        HAL_GPIO_Init(obj->D0_Port, &GPIO_InitStruct);

        BspSDMMC_PinCLK_Enable(obj->D1_Port);
        GPIO_InitStruct.Pin = obj->D1_Pin;
        HAL_GPIO_Init(obj->D1_Port, &GPIO_InitStruct);

        BspSDMMC_PinCLK_Enable(obj->D2_Port);
        GPIO_InitStruct.Pin = obj->D2_Pin;
        HAL_GPIO_Init(obj->D2_Port, &GPIO_InitStruct);

        BspSDMMC_PinCLK_Enable(obj->D3_Port);
        GPIO_InitStruct.Pin = obj->D3_Pin;
        HAL_GPIO_Init(obj->D3_Port, &GPIO_InitStruct);

        return true;
    }

    return false;
}

static bool BspSDMMC_MDMA_Init(MDMA_HandleTypeDef *mdma)
{
    if (mdma == NULL)
        return false;

    __HAL_RCC_MDMA_CLK_ENABLE();

    mdma->Instance = MDMA_Channel0;
    mdma->Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
    mdma->Init.Priority = MDMA_PRIORITY_HIGH;
    mdma->Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
    mdma->Init.SourceInc = MDMA_SRC_INC_BYTE;
    mdma->Init.DestinationInc = MDMA_DEST_INC_BYTE;
    mdma->Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
    mdma->Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
    mdma->Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
    mdma->Init.BufferTransferLength = 1;
    mdma->Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
    mdma->Init.DestBurst = MDMA_DEST_BURST_SINGLE;
    mdma->Init.SourceBlockAddressOffset = 0;
    mdma->Init.DestBlockAddressOffset = 0;
    if (HAL_MDMA_Init(mdma) != HAL_OK)
        return false;

    HAL_NVIC_SetPriority(MDMA_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(MDMA_IRQn);

    return true;
}

static bool BspSDMMC_Init(BspSDMMC_Obj_TypeDef *obj)
{
    IRQn_Type irq = SDMMC1_IRQn;

    if (obj == NULL)
        return false;
        
    BspSDMMC_PortCLK_Init(obj->instance);
    BspSDMMC_Pin_Init(obj->instance, obj->pin);

    memset(BspSCMMC_Callback_List, 0, sizeof(BspSCMMC_Callback_List));

    if (obj->instance == SDMMC1)
    {
        irq = SDMMC1_IRQn;
        obj->ref_callback_item = &BspSCMMC_Callback_List[BspSDMMC_1_Callback];
    }
    else if (obj->instance == SDMMC2)
    {
        irq = SDMMC2_IRQn;
        obj->ref_callback_item = &BspSCMMC_Callback_List[BspSDMMC_2_Callback];
    }
    else
        return false;

    obj->hdl.Instance = obj->instance; // SDMMC1;
    obj->hdl.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    obj->hdl.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    obj->hdl.Init.BusWide = SDMMC_BUS_WIDE_4B;
    obj->hdl.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    obj->hdl.Init.ClockDiv = 2;
    obj->hdl.Init.TranceiverPresent = SDMMC_TRANSCEIVER_NOT_PRESENT;

    if (!BspSDMMC_MDMA_Init(&(obj->mdma)) || (HAL_SD_Init(&(obj->hdl)) != HAL_OK))
        return false;

    HAL_NVIC_SetPriority(irq, 4, 0);
    HAL_NVIC_EnableIRQ(irq);

    return true;
}

static bool BspSDMMC_Read(BspSDMMC_Obj_TypeDef *obj, uint8_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
    HAL_StatusTypeDef state = HAL_SD_ReadBlocks_DMA(&(obj->hdl), pData, ReadAddr, NumOfBlocks);
    if(state == HAL_OK)
        return true;

    return false;
}

static bool BspSDMMC_Write(BspSDMMC_Obj_TypeDef *obj, uint8_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
    HAL_StatusTypeDef state = HAL_SD_WriteBlocks_DMA(&(obj->hdl), pData, WriteAddr, NumOfBlocks);
    if(state == HAL_OK)
        return true;

    return false;
}

static bool BspSDMMC_Erase(BspSDMMC_Obj_TypeDef *obj, uint32_t StartAddr, uint32_t EndAddr)
{
    if (HAL_SD_Erase(&(obj->hdl), StartAddr, EndAddr) != HAL_OK)
        return false;

    return true;
}

static BspSDMMC_OperationState_List BspSDMMC_Get_Operation_State(BspSDMMC_Obj_TypeDef *obj)
{
    if(obj)
        return (BspSDMMC_OperationState_List)HAL_SD_GetState(&(obj->hdl));

    return BspSDMMC_Opr_State_ERROR;
}

static bool BspSDMMC_GetCardStatus(BspSDMMC_Obj_TypeDef *obj)
{
    if (obj == NULL)
        return false;

    return ((HAL_SD_GetCardState(&(obj->hdl)) == HAL_SD_CARD_TRANSFER) ? true : false);
}

static bool BspSDMMC_GetInfo(BspSDMMC_Obj_TypeDef *obj, HAL_SD_CardInfoTypeDef *info_out)
{
    if ((obj == NULL) && (info_out == NULL))
        return false;

    memset(&(obj->info), 0, sizeof(obj->info));
    HAL_SD_GetCardInfo(&(obj->hdl), &(obj->info));
    info_out = &(obj->info);

    return true;
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
    if (hsd->Instance == SDMMC1)
    {
        if(BspSCMMC_Callback_List[BspSDMMC_1_Callback].Write_Callback)
        {
            BspSCMMC_Callback_List[BspSDMMC_1_Callback].Write_Callback((uint8_t *)hsd, sizeof(SD_HandleTypeDef));
        }

        __DSB();
    }
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
    if (hsd->Instance == SDMMC1)
    {
        if(BspSCMMC_Callback_List[BspSDMMC_1_Callback].Read_Callback)
        {
            BspSCMMC_Callback_List[BspSDMMC_1_Callback].Read_Callback((uint8_t *)hsd, sizeof(SD_HandleTypeDef));
        }

        __DSB();
    }
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
    if (hsd->Instance == SDMMC1)
    {
        if (hsd->ErrorCode & HAL_SD_ERROR_TX_UNDERRUN)
        {
            HAL_SD_Abort(&hsd);
        }

        if(BspSCMMC_Callback_List[BspSDMMC_1_Callback].Error_Callback)
        {
            BspSCMMC_Callback_List[BspSDMMC_1_Callback].Error_Callback((uint8_t *)hsd, sizeof(SD_HandleTypeDef));
        }
    }
}

static void BspSDMMC_Set_Callback(BspSDMMC_Obj_TypeDef *obj, BspSDMMC_Callback_TypeList type, SDMMC_Callback cb)
{
    if(obj)
    {
        switch((uint8_t)type)
        {
            case BspSDMMC_Callback_Type_Write:
                obj->Write_Callback = cb;
                if(obj->ref_callback_item)
                    obj->ref_callback_item->Write_Callback = cb;
                break;
            
            case BspSDMMC_Callback_Type_Read:
                obj->Read_Callback = cb;
                if(obj->ref_callback_item)
                    obj->ref_callback_item->Read_Callback = cb;
                break;

            case BspSDMMC_Callback_Type_Error:
                obj->Error_Callback = cb;
                if(obj->ref_callback_item)
                    obj->ref_callback_item->Error_Callback = cb;
                break;

            default:
                break;
        }
    }
}
