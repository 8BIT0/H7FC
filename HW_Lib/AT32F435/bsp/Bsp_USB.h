#ifndef __BSP_USB_H
#define __BSP_USB_H

#include "Bsp_USB_Port_Def.h"

#define USB_RX_BUFF_SIZE 512

void BspUSB_Irq_Callback(void);

extern BspUSB_VCP_TypeDef BspUSB_VCP;

#endif
