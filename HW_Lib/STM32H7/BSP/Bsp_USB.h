#ifndef __BSP_USB_H
#define __BSP_USB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "Bsp_USB_Port_Def.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"

extern BspUSB_VCP_TypeDef BspUSB_VCP;

#ifdef __cplusplus
}
#endif

#endif
