#include "stm32h7xx_it.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal_gpio.h"
#include "IO_Definition.h"
#include "debug_util.h"
#include "DIskIO.h"
#include "DataPipe.h"
#include "Bsp_DMA.h"
#include "Bsp_IIC.h"
#include "Bsp_Uart.h" 
#include "Bsp_Timer.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DevCard_Obj_TypeDef DevTFCard_Obj;
extern DMA_HandleTypeDef DataPipe_DMA;

void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void DebugMon_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
}

void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

void EXTI2_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

void EXTI3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}

void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}

void SDMMC1_IRQHandler(void)
{
  HAL_SD_IRQHandler(&DevTFCard_Obj.SDMMC_Obj.hdl);
}

void MDMA_IRQHandler(void)
{
  HAL_MDMA_IRQHandler(&DevTFCard_Obj.SDMMC_Obj.mdma);
}

void DMA2_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&DataPipe_DMA);
}

void DMA1_Stream0_IRQHandler(void)
{
  DMA_HandleTypeDef *hdl = NULL;
  hdl = BspDMA.get_handle(Bsp_DMA_1, Bsp_DMA_Stream_0);

  if (hdl)
    HAL_DMA_IRQHandler(hdl);
}

void DMA1_Stream1_IRQHandler(void)
{
  DMA_HandleTypeDef *hdl = NULL;
  hdl = BspDMA.get_handle(Bsp_DMA_1, Bsp_DMA_Stream_1);

  if (hdl)
    HAL_DMA_IRQHandler(hdl);
}

void DMA1_Stream2_IRQHandler(void)
{
  DMA_HandleTypeDef *hdl = NULL;
  hdl = BspDMA.get_handle(Bsp_DMA_1, Bsp_DMA_Stream_2);

  if (hdl)
    HAL_DMA_IRQHandler(hdl);
}

void DMA1_Stream3_IRQHandler(void)
{
  DMA_HandleTypeDef *hdl = NULL;
  hdl = BspDMA.get_handle(Bsp_DMA_1, Bsp_DMA_Stream_3);

  if (hdl)
    HAL_DMA_IRQHandler(hdl);
}

void DMA1_Stream4_IRQHandler(void)
{
  DMA_HandleTypeDef *hdl = NULL;
  hdl = BspDMA.get_handle(Bsp_DMA_1, Bsp_DMA_Stream_4);

  if (hdl)
    HAL_DMA_IRQHandler(hdl);
}

void DMA1_Stream5_IRQHandler(void)
{
  DMA_HandleTypeDef *hdl = NULL;
  hdl = BspDMA.get_handle(Bsp_DMA_1, Bsp_DMA_Stream_5);

  if (hdl)
    HAL_DMA_IRQHandler(hdl);
}

void UART4_IRQHandler(void)
{
  UART_HandleTypeDef *hdl = NULL;
  UART_IRQ_Callback(BspUART_Port_4);
  hdl = BspUart_GetObj_Handle(BspUART_Port_4);

  if (hdl && (hdl->Instance == UART4))
    HAL_UART_IRQHandler(hdl);
}

void USART6_IRQHandler(void)
{
  UART_HandleTypeDef *hdl = NULL;
  UART_IRQ_Callback(BspUART_Port_6);
  hdl = BspUart_GetObj_Handle(BspUART_Port_6);

  if (hdl && (hdl->Instance == USART6))
    HAL_UART_IRQHandler(hdl);
}

void UART7_IRQHandler(void)
{
  UART_HandleTypeDef *hdl = NULL;
  UART_IRQ_Callback(BspUART_Port_7);
  hdl = BspUart_GetObj_Handle(BspUART_Port_7);

  if (hdl && (hdl->Instance == UART7))
    HAL_UART_IRQHandler(hdl);
}

void I2C2_ER_IRQHandler(void)
{
  I2C_HandleTypeDef *hdl = NULL;
  hdl = BspIIC_Get_HandlePtr(BspIIC_Instance_I2C_2);

  if(hdl)
    HAL_I2C_ER_IRQHandler(hdl);
}

void TIM7_IRQHandler(void)
{
  TIM_HandleTypeDef *hdl;
  hdl = BspTimer_Get_Tick_HandlePtr(BspTimer_7);

  if(hdl)
    HAL_TIM_IRQHandler(hdl);
}
