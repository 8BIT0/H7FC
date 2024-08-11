#ifndef __HW_DEF_H
#define __HW_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Bsp_GPIO.h"
#include "Bsp_DMA.h"
#include "../../AT32F435/bsp/Bsp_Uart.h"
#include "Bsp_IIC.h"
#include "Bsp_SPI.h"
#include "Bsp_Flash.h"
#include "Dev_Led.h"
#include "debug_util.h"
#include "util.h"
#include "../../common/gen_physic_def/imu_data.h"

/* device support */
#include "Dev_W25Qxx.h"

#define RECEIVER_PORT USART3
#define RECEIVER_CRSF_RX_DMA Bsp_DMA_1
#define RECEIVER_CRSF_RX_DMA_STREAM Bsp_DMA_Stream_5
#define RECEIVER_CRSF_TX_DMA Bsp_DMA_1
#define RECEIVER_CRSF_TX_DMA_STREAM Bsp_DMA_Stream_6

#define RECEIVER_SBUS_RX_DMA Bsp_DMA_1
#define RECEIVER_SBUS_RX_DMA_STREAM Bsp_DMA_Stream_5
#define RECEIVER_SBUS_TX_DMA Bsp_DMA_1
#define RECEIVER_SBUS_TX_DMA_STREAM Bsp_DMA_Stream_6

#define CRSF_TX_PIN Uart3_TxPin
#define CRSF_RX_PIN Uart3_RxPin

#define SBUS_TX_PIN Uart3_TxPin
#define SBUS_RX_PIN Uart3_RxPin

#ifdef __cplusplus
}
#endif

#endif

