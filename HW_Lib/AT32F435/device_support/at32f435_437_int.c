/**
  **************************************************************************
  * @file     at32f435_437_int.c
  * @version  v2.1.0
  * @date     2022-08-16
  * @brief    main interrupt service routines.
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "at32f435_437_int.h"
#include "at32f435_437.h"
#include "Bsp_GPIO.h"
#include "Bsp_DMA.h"
#include "Bsp_Uart.h"
#include "Bsp_USB.h"
#include "Bsp_Timer.h"
#include "FreeRTOS.h"
#include "task.h"

/** @addtogroup UTILITIES_examples
  * @{
  */

/** @addtogroup FreeRTOS_demo
  * @{
  */

/**
  * @brief  this function handles nmi exception.
  * @param  none
  * @retval none
  */
void NMI_Handler(void)
{
}

/**
  * @brief  this function handles hard fault exception.
  * @param  none
  * @retval none
  */
void HardFault_Handler(void)
{
  /* go to infinite loop when hard fault exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles memory manage exception.
  * @param  none
  * @retval none
  */
void MemManage_Handler(void)
{
  /* go to infinite loop when memory manage exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles bus fault exception.
  * @param  none
  * @retval none
  */
void BusFault_Handler(void)
{
  /* go to infinite loop when bus fault exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles usage fault exception.
  * @param  none
  * @retval none
  */
void UsageFault_Handler(void)
{
  /* go to infinite loop when usage fault exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles svcall exception.
  * @param  none
  * @retval none
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  this function handles debug monitor exception.
  * @param  none
  * @retval none
  */
void DebugMon_Handler(void)
{
}

void EXINT0_IRQHandler(void)
{
  BspGPIO_IRQ_Polling(EXINT_LINE_0);
}

void EXINT1_IRQHandler(void)
{
  BspGPIO_IRQ_Polling(EXINT_LINE_1);
}

void EXINT2_IRQHandler(void)
{
  BspGPIO_IRQ_Polling(EXINT_LINE_2);
}

void EXINT3_IRQHandler(void)
{
  BspGPIO_IRQ_Polling(EXINT_LINE_3);
}

void EXINT4_IRQHandler(void)
{
  BspGPIO_IRQ_Polling(EXINT_LINE_4);
}

void EXINT9_5_IRQHandler(void)
{
  BspGPIO_IRQ_Polling(EXINT_LINE_5);
  BspGPIO_IRQ_Polling(EXINT_LINE_6);
  BspGPIO_IRQ_Polling(EXINT_LINE_7);
  BspGPIO_IRQ_Polling(EXINT_LINE_8);
  BspGPIO_IRQ_Polling(EXINT_LINE_9);
}

void EXINT15_10_IRQHandler(void)
{
  BspGPIO_IRQ_Polling(EXINT_LINE_10);
  BspGPIO_IRQ_Polling(EXINT_LINE_11);
  BspGPIO_IRQ_Polling(EXINT_LINE_12);
  BspGPIO_IRQ_Polling(EXINT_LINE_13);
  BspGPIO_IRQ_Polling(EXINT_LINE_14);
  BspGPIO_IRQ_Polling(EXINT_LINE_15);
}

void DMA1_Channel1_IRQHandler(void)
{
  if(dma_flag_get(DMA1_FDT1_FLAG))
  {
    BspDMA_Irq_Callback((void *)DMA1_CHANNEL1);
    dma_flag_clear(DMA1_FDT1_FLAG);
  }
}

void DMA1_Channel2_IRQHandler(void)
{
  if(dma_flag_get(DMA1_FDT2_FLAG))
  {
    BspDMA_Irq_Callback((void *)DMA1_CHANNEL2);
    dma_flag_clear(DMA1_FDT2_FLAG);
  }
}

void DMA1_Channel3_IRQHandler(void)
{
  if(dma_flag_get(DMA1_FDT3_FLAG))
  {
    BspDMA_Irq_Callback((void *)DMA1_CHANNEL3);
    dma_flag_clear(DMA1_FDT3_FLAG);
  }
}

void DMA1_Channel4_IRQHandler(void)
{
  if(dma_flag_get(DMA1_FDT4_FLAG))
  {
    BspDMA_Irq_Callback((void *)DMA1_CHANNEL4);
    dma_flag_clear(DMA1_FDT4_FLAG);
  }
}

void DMA1_Channel5_IRQHandler(void)
{
  if(dma_flag_get(DMA1_FDT5_FLAG))
  {
    BspDMA_Irq_Callback((void *)DMA1_CHANNEL5);
    dma_flag_clear(DMA1_FDT5_FLAG);
  }
}

void DMA1_Channel6_IRQHandler(void)
{
  if(dma_flag_get(DMA1_FDT6_FLAG))
  {
    BspDMA_Irq_Callback((void *)DMA1_CHANNEL6);
    dma_flag_clear(DMA1_FDT6_FLAG);
  }
}

void DMA1_Channel7_IRQHandler(void)
{
  if(dma_flag_get(DMA1_FDT7_FLAG))
  {
    BspDMA_Irq_Callback((void *)DMA1_CHANNEL7);
    dma_flag_clear(DMA1_FDT7_FLAG);
  }
}

void DMA2_Channel1_IRQHandler(void)
{
  if(dma_flag_get(DMA2_FDT1_FLAG) != RESET)
  {
    BspDMA_Irq_Callback((void *)DMA2_CHANNEL1);
    dma_flag_clear(DMA2_FDT1_FLAG);
  }
}

void DMA2_Channel2_IRQHandler(void)
{
  if(dma_flag_get(DMA2_FDT2_FLAG) != RESET)
  {
    BspDMA_Irq_Callback((void *)DMA2_CHANNEL2);
    dma_flag_clear(DMA2_FDT2_FLAG);
  }
}

void DMA2_Channel3_IRQHandler(void)
{
  if(dma_flag_get(DMA2_FDT3_FLAG) != RESET)
  {
    BspDMA_Irq_Callback((void *)DMA2_CHANNEL3);
    dma_flag_clear(DMA2_FDT3_FLAG);
  }
}

void DMA2_Channel4_IRQHandler(void)
{
  if(dma_flag_get(DMA2_FDT4_FLAG) != RESET)
  {
    BspDMA_Irq_Callback((void *)DMA2_CHANNEL4);
    dma_flag_clear(DMA2_FDT4_FLAG);
  }
}

void DMA2_Channel5_IRQHandler(void)
{
  if(dma_flag_get(DMA2_FDT5_FLAG) != RESET)
  {
    BspDMA_Irq_Callback((void *)DMA2_CHANNEL5);
    dma_flag_clear(DMA2_FDT5_FLAG);
  }
}

void DMA2_Channel6_IRQHandler(void)
{
    BspDMA_Irq_Callback((void *)DMA2_CHANNEL6);
  if(dma_flag_get(DMA2_FDT6_FLAG) != RESET)
  {
    dma_flag_clear(DMA2_FDT6_FLAG);
  }
}

void DMA2_Channel7_IRQHandler(void)
{
  if(dma_flag_get(DMA2_FDT7_FLAG) != RESET)
  {
    BspDMA_Pipe_Irq_Callback();
    dma_flag_clear(DMA2_FDT7_FLAG);
  }
}

void USART1_IRQHandler(void)
{
  if(usart_flag_get(USART1, USART_IDLEF_FLAG) != RESET)
  {
    BspUart_Irq_Callback(USART1);
    usart_flag_clear(USART1, USART_IDLEF_FLAG);
  }
  else if(usart_flag_get(USART1, USART_RDBF_FLAG) != RESET)
  {
    BspUart_Irq_Callback(USART1);
    usart_flag_clear(USART1, USART_RDBF_FLAG);
  }
}

void USART2_IRQHandler(void)
{
  if(usart_flag_get(USART2, USART_IDLEF_FLAG) != RESET)
  {
    BspUart_Irq_Callback(USART2);
    usart_flag_clear(USART2, USART_IDLEF_FLAG);
  }
  else if(usart_flag_get(USART2, USART_RDBF_FLAG) != RESET)
  {
    BspUart_Irq_Callback(USART2);
    usart_flag_clear(USART2, USART_RDBF_FLAG);
  }
}

void USART3_IRQHandler(void)
{
  if(usart_flag_get(USART3, USART_IDLEF_FLAG) != RESET)
  {
    BspUart_Irq_Callback(USART3);
    usart_flag_clear(USART3, USART_IDLEF_FLAG);
  }
  else if(usart_flag_get(USART3, USART_RDBF_FLAG) != RESET)
  {
    BspUart_Irq_Callback(USART3);
    usart_flag_clear(USART3, USART_RDBF_FLAG);
  }
}

void UART4_IRQHandler(void)
{
  if(usart_flag_get(UART4, USART_IDLEF_FLAG) != RESET)
  {
    BspUart_Irq_Callback(UART4);
    usart_flag_clear(UART4, USART_IDLEF_FLAG);
  }
  else if(usart_flag_get(UART4, USART_RDBF_FLAG) != RESET)
  {
    BspUart_Irq_Callback(UART4);
    usart_flag_clear(UART4, USART_RDBF_FLAG);
  }
}

void UART5_IRQHandler(void)
{
  if(usart_flag_get(UART5, USART_IDLEF_FLAG) != RESET)
  {
    BspUart_Irq_Callback(UART5);
    usart_flag_clear(UART5, USART_IDLEF_FLAG);
  }
  else if(usart_flag_get(UART5, USART_RDBF_FLAG) != RESET)
  {
    BspUart_Irq_Callback(UART5);
    usart_flag_clear(UART5, USART_RDBF_FLAG);
  }
}

void USART6_IRQHandler(void)
{
  if(usart_flag_get(USART6, USART_IDLEF_FLAG) != RESET)
  {
    BspUart_Irq_Callback(USART6);
    usart_flag_clear(USART6, USART_IDLEF_FLAG);
  }
  else if(usart_flag_get(USART6, USART_RDBF_FLAG) != RESET)
  {
    BspUart_Irq_Callback(USART6);
    usart_flag_clear(USART6, USART_RDBF_FLAG);
  }
}

void UART7_IRQHandler(void)
{
  if(usart_flag_get(UART7, USART_IDLEF_FLAG) != RESET)
  {
    BspUart_Irq_Callback(UART7);
    usart_flag_clear(UART7, USART_IDLEF_FLAG);
  }
  else if(usart_flag_get(UART7, USART_RDBF_FLAG) != RESET)
  {
    BspUart_Irq_Callback(UART7);
    usart_flag_clear(UART7, USART_RDBF_FLAG);
  }
}

void UART8_IRQHandler(void)
{
  if(usart_flag_get(UART8, USART_IDLEF_FLAG) != RESET)
  {
    BspUart_Irq_Callback(UART8);
    usart_flag_clear(UART8, USART_IDLEF_FLAG);
  }
  else if(usart_flag_get(UART8, USART_RDBF_FLAG) != RESET)
  {
    BspUart_Irq_Callback(UART8);
    usart_flag_clear(UART8, USART_RDBF_FLAG);
  }
}

void OTGFS1_IRQHandler(void)
{
  BspUSB_Irq_Callback();
}

/**
  * @brief  this function handles pendsv_handler exception.
  * @param  none
  * @retval none
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  this function handles systick handler.
  * @param  none
  * @retval none
  */
void SysTick_Handler(void)
{

}

/* os timer */
void TMR20_OVF_IRQHandler(void)
{
  if(tmr_flag_get(TMR20, TMR_OVF_FLAG) == SET)
  {
#if (INCLUDE_xTaskGetSchedulerState == 1 )
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
#endif /* INCLUDE_xTaskGetSchedulerState */
      xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
    }
#endif /* INCLUDE_xTaskGetSchedulerState */

    /* increase tick for bsp time out compare */
    System_Tick();

    tmr_flag_clear(TMR20, TMR_OVF_FLAG);
  }
}

/**
  * @}
  */

/**
  * @}
  */
